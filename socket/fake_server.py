"""
Fake Dobot Magician Server
==========================
Drop-in replacement for dobot_server.py. Speaks the exact same socket
protocol but simulates the robot internally — no hardware needed.

Protocol (matches dobot_server.py exactly):
    0  start_homing          cmd only, no response
    1  get_joint_state       → <4f> j1,j2,j3,j4
    2  is_goal_valid j1..j4  → <1B> bool
    3  set_joint_ptp j1..j4  cmd only, no response
    4  suction_cup ON        cmd only, no response
    5  suction_cup OFF       cmd only, no response
    6  stop_current_action   cmd only, no response

Simulation behaviour:
    - Joints move at a realistic configurable speed (deg/s)
    - Motion is preempted correctly when stop or a new goal arrives
    - Homing drives all joints to 0 at homing speed
    - State is shared across all concurrent connections (as real hardware is)
    - All commands and state changes are printed with timestamps
"""

import math
import selectors
import socket
import struct
import threading
import time

# ──────────────────────────────────────────────────────────────────────────────
# Joint limits (mirror dobot_client.py)
# ──────────────────────────────────────────────────────────────────────────────
J1MIN, J1MAX = -90.0,  90.0
J2MIN, J2MAX =   0.0,  70.0
J3MIN, J3MAX = -20.0,  60.0
J4MIN, J4MAX = -90.0,  90.0

LIMITS = [(J1MIN, J1MAX), (J2MIN, J2MAX), (J3MIN, J3MAX), (J4MIN, J4MAX)]

# ──────────────────────────────────────────────────────────────────────────────
# Motion parameters
# ──────────────────────────────────────────────────────────────────────────────
JOINT_SPEED_DEG_S  = 30.0   # matches velocity set in DobotClient.__init__
HOMING_SPEED_DEG_S = 20.0
MOTION_TICK_S      = 0.02   # 50 Hz simulation tick

CMD_STRUCT = "<1B4f"
CMD_SIZE   = struct.calcsize(CMD_STRUCT)


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def is_valid(j1, j2, j3, j4):
    vals = (j1, j2, j3, j4)
    return all(lo <= v <= hi for v, (lo, hi) in zip(vals, LIMITS))


def clamp_goal(j1, j2, j3, j4):
    return tuple(clip(v, lo, hi) for v, (lo, hi) in zip((j1, j2, j3, j4), LIMITS))


def ts():
    return time.strftime("%H:%M:%S")


# ──────────────────────────────────────────────────────────────────────────────
# Shared robot state
# ──────────────────────────────────────────────────────────────────────────────
class RobotState:
    def __init__(self):
        self._lock     = threading.Lock()
        self.joints    = [0.0, 0.0, 0.0, 0.0]   # current joint angles
        self.goal      = None                     # (j1,j2,j3,j4) or None
        self.moving    = False
        self.suction   = False
        self._stop_evt = threading.Event()

        # Start the motion simulation thread
        t = threading.Thread(target=self._motion_loop, daemon=True)
        t.start()

    # ── Motion loop ────────────────────────────────────────────────────────────
    def _motion_loop(self):
        while True:
            time.sleep(MOTION_TICK_S)
            with self._lock:
                if self.goal is None or self._stop_evt.is_set():
                    self.moving = False
                    continue

                step = JOINT_SPEED_DEG_S * MOTION_TICK_S
                new_joints = list(self.joints)
                arrived = True

                for i in range(4):
                    diff = self.goal[i] - self.joints[i]
                    if abs(diff) <= step:
                        new_joints[i] = self.goal[i]
                    else:
                        new_joints[i] = self.joints[i] + math.copysign(step, diff)
                        arrived = False

                self.joints = new_joints
                self.moving = not arrived

                if arrived:
                    print(
                        f"[{ts()}] ✓ ARRIVED  joints="
                        f"[{new_joints[0]:.2f}, {new_joints[1]:.2f}, "
                        f"{new_joints[2]:.2f}, {new_joints[3]:.2f}]"
                    )
                    self.goal = None

    # ── Public API (called from socket handler threads) ────────────────────────
    def start_homing(self):
        with self._lock:
            self._stop_evt.clear()
            self.goal = (0.0, 0.0, 0.0, 0.0)
            self.moving = True
        print(f"[{ts()}] ⌂  HOMING  → (0, 0, 0, 0)")

    def get_joint_state(self):
        with self._lock:
            return tuple(self.joints)

    def is_goal_valid(self, j1, j2, j3, j4):
        return is_valid(j1, j2, j3, j4)

    def set_joint_ptp(self, j1, j2, j3, j4):
        j1, j2, j3, j4 = clamp_goal(j1, j2, j3, j4)
        with self._lock:
            self._stop_evt.clear()
            self.goal = (j1, j2, j3, j4)
            self.moving = True
        print(
            f"[{ts()}] → PTP     goal=({j1:.2f}, {j2:.2f}, {j3:.2f}, {j4:.2f})"
        )

    def set_suction_cup(self, enable: bool):
        with self._lock:
            self.suction = enable
        state = "ON 🟢" if enable else "OFF ⚫"
        print(f"[{ts()}] ⊙  SUCTION {state}")

    def stop_current_action(self):
        with self._lock:
            self._stop_evt.set()
            self.goal = None
            self.moving = False
        print(f"[{ts()}] ■  STOP")


# ──────────────────────────────────────────────────────────────────────────────
# Socket server
# ──────────────────────────────────────────────────────────────────────────────
robot = RobotState()
sel   = selectors.DefaultSelector()


def handle_read(conn, mask):
    data = conn.recv(CMD_SIZE)
    if not data:
        print(f"[{ts()}]    client disconnected")
        sel.unregister(conn)
        conn.close()
        return

    if len(data) < CMD_SIZE:
        return  # partial packet — ignore

    cmd, j1, j2, j3, j4 = struct.unpack(CMD_STRUCT, data)

    if cmd == 0:
        robot.start_homing()

    elif cmd == 1:
        j = robot.get_joint_state()
        conn.send(struct.pack("<4f", *j))

    elif cmd == 2:
        valid = robot.is_goal_valid(j1, j2, j3, j4)
        conn.send(struct.pack("<1B", int(valid)))

    elif cmd == 3:
        robot.set_joint_ptp(j1, j2, j3, j4)

    elif cmd == 4:
        robot.set_suction_cup(True)

    elif cmd == 5:
        robot.set_suction_cup(False)

    elif cmd == 6:
        robot.stop_current_action()

    else:
        print(f"[{ts()}] ⚠  unknown command id={cmd}")


def handle_accept(sock, mask):
    conn, addr = sock.accept()
    conn.setblocking(True)
    print(f"[{ts()}]    client connected from {addr}")
    sel.register(conn, selectors.EVENT_READ, handle_read)


# ──────────────────────────────────────────────────────────────────────────────
# Status ticker — prints robot state every second so you can see it moving
# ──────────────────────────────────────────────────────────────────────────────
def status_ticker():
    while True:
        time.sleep(1.0)
        j = robot.get_joint_state()
        suc = "ON" if robot.suction else "OFF"
        mov = "MOVING" if robot.moving else "idle  "
        print(
            f"[{ts()}] ≈  {mov}  "
            f"j=[{j[0]:6.2f} {j[1]:6.2f} {j[2]:6.2f} {j[3]:6.2f}]  "
            f"suction={suc}"
        )


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
server_sock = socket.socket()
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind(("localhost", 0))
port = server_sock.getsockname()[1]

with open("/tmp/DOBOT_MAGICIAN_PORT", "w") as f:
    f.write(str(port))

server_sock.listen(16)   # allow many concurrent node connections
server_sock.setblocking(False)
sel.register(server_sock, selectors.EVENT_READ, handle_accept)

threading.Thread(target=status_ticker, daemon=True).start()

print("=" * 60)
print("  Fake Dobot Magician Server")
print(f"  Listening on localhost:{port}")
print(f"  Port written to /tmp/DOBOT_MAGICIAN_PORT")
print(f"  Joint limits: J1[{J1MIN},{J1MAX}] J2[{J2MIN},{J2MAX}]"
      f" J3[{J3MIN},{J3MAX}] J4[{J4MIN},{J4MAX}]")
print(f"  Joint speed: {JOINT_SPEED_DEG_S} deg/s")
print("=" * 60)

try:
    while True:
        events = sel.select(timeout=0.1)
        for key, mask in events:
            key.data(key.fileobj, mask)
except KeyboardInterrupt:
    print(f"\n[{ts()}] Shutting down.")
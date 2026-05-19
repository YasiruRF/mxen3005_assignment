"""
DobotClient — socket edition
=============================
Drop-in replacement for dobot/dobot/dobot_client.py.

Instead of opening a serial port it connects to the server at the port
recorded in /tmp/DOBOT_MAGICIAN_PORT (written by fake_dobot_server.py
or by the real dobot_server.py).

Each instance opens its own persistent TCP connection, matching the
per-node connection pattern used in the original code.

Usage — swap the import in any node:
    # original:  from .dobot_client import DobotClient
    # replace:   from dobot_client_socket import DobotClient
or simply overwrite dobot/dobot/dobot_client.py with this file.
"""

import socket
import struct

PORT_FILE = "/tmp/DOBOT_MAGICIAN_PORT"

CMD_STRUCT  = "<1B4f"
CMD_SIZE    = struct.calcsize(CMD_STRUCT)


def _read_port() -> int:
    with open(PORT_FILE) as f:
        return int(f.read().strip())


class DobotClient:
    """Socket-backed Dobot client — identical public API to the serial version."""

    def __init__(self):
        port = _read_port()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect(("localhost", port))
        self._sock.setblocking(True)

    # ── Internal helpers ────────────────────────────────────────────────────
    def _send(self, cmd_id: int, j1=0.0, j2=0.0, j3=0.0, j4=0.0):
        packet = struct.pack(CMD_STRUCT, cmd_id, j1, j2, j3, j4)
        self._sock.sendall(packet)

    def _recv_floats(self, n: int):
        fmt  = f"<{n}f"
        size = struct.calcsize(fmt)
        buf  = b""
        while len(buf) < size:
            chunk = self._sock.recv(size - len(buf))
            if not chunk:
                raise ConnectionError("Server closed connection during recv")
            buf += chunk
        return struct.unpack(fmt, buf)

    def _recv_bool(self) -> bool:
        buf = self._sock.recv(1)
        return bool(struct.unpack("<1B", buf)[0])

    # ── Public API (matches dobot_client.py exactly) ────────────────────────
    def start_homing(self):
        self._send(0)

    def get_joint_state(self):
        """Returns (j1, j2, j3, j4) in degrees."""
        self._send(1)
        return self._recv_floats(4)

    def is_goal_valid(self, j1, j2, j3, j4) -> bool:
        self._send(2, j1, j2, j3, j4)
        return self._recv_bool()

    def set_joint_ptp(self, j1, j2, j3, j4):
        self._send(3, j1, j2, j3, j4)

    def set_suction_cup(self, enable: bool):
        self._send(4 if enable else 5)

    def stop_current_action(self):
        self._send(6)

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass

    def __del__(self):
        self.close()
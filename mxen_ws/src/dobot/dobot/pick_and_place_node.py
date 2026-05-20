import rclpy
from rclpy.node import Node
from .dobot_client import DobotClient
from .dobot_kinematics import inverse_kinematics
from dobot_interface.srv import PickAndPlace
import time

CLEARANCE_Z = 200.0

# ─────────────────────────────────────────────
#  MOVE TIMING  – tweak these if robot stops mid-air
# ─────────────────────────────────────────────
SLEEP_LONG_MOVE  = 5.0   # after large XY travel (pick↔place)
SLEEP_SHORT_MOVE = 2.5   # after small vertical move (above↔pick/place z)
SLEEP_SUCTION_ON = 1.0   # wait for suction to grip
SLEEP_SUCTION_OFF= 1.0   # wait for suction to release
SLEEP_SETTLE     = 1.0   # wait after descending before releasing block

# ─────────────────────────────────────────────
#  GRID PICK POSITIONS  (3×3, block=35mm, gap=5mm, step=40mm)
#  Centre block: x=92, y=-201, z=35, r=0
#
#        x=52    x=92    x=132
# y=-161  [P1]    [P2]    [P3]
# y=-201  [P4]    [P5]    [P6]   ← centre row
# y=-241  [P7]    [P8]    [P9]
# ─────────────────────────────────────────────

P1 = ( 52, -161, 35, 0)
P2 = ( 92, -161, 35, 0)
P3 = (132, -161, 35, 0)

P4 = ( 52, -201, 35, 0)
P5 = ( 92, -201, 35, 0)   # centre block
P6 = (132, -201, 35, 0)

P7 = ( 52, -241, 35, 0)
P8 = ( 92, -241, 35, 0)
P9 = (132, -241, 35, 0)

# ─────────────────────────────────────────────
#  PYRAMID PLACE POSITIONS
#  First block: x=135, y=226, z=35, r=0
#  Block = 35mm → +40mm in x per block in same layer
#               → +35mm in z per layer up
#               → +17.5mm x offset per layer for centering
#
#  Bottom (z=35 ):  D1  D2  D3  D4
#  Middle (z=70 ):    D5  D6  D7
#  Top    (z=105):      D8  D9
# ─────────────────────────────────────────────

D1 = (135,   226,  35, 0)
D2 = (175,   226,  35, 0)
D3 = (215,   226,  35, 0)
D4 = (255,   226,  35, 0)

D5 = (152.5, 226,  70, 0)
D6 = (192.5, 226,  70, 0)
D7 = (232.5, 226,  70, 0)

D8 = (175,   226, 105, 0)
D9 = (215,   226, 105, 0)


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place_node")
        self.dobot = DobotClient()
        self.service = self.create_service(
            PickAndPlace,
            "pick_and_place",
            self.service_callback,
        )
        self.get_logger().info("Pick-and-place service ready.")

    def move_to(self, x, y, z, r, label="", sleep=SLEEP_SHORT_MOVE):
        """Convert to joints and move. DobotClient handles limit checking."""
        j1, j2, j3, j4 = inverse_kinematics(x, y, z, r)
        self.get_logger().info(
            f"  → {label} ({x},{y},{z}) | J1={j1:.1f} J2={j2:.1f} J3={j3:.1f} J4={j4:.1f}"
        )
        if not self.dobot.is_goal_valid(j1, j2, j3, j4):
            raise ValueError(f"Invalid goal for {label}: ({j1}, {j2}, {j3}, {j4})")
        self.dobot.set_joint_ptp(j1, j2, j3, j4)
        time.sleep(sleep)   # give robot enough time to finish the move

    def pick(self, x, y, z, r, name):
        self.get_logger().info(f"=== PICK {name} ===")
        self.move_to(x, y, z + CLEARANCE_Z, r, f"{name} above",  SLEEP_SHORT_MOVE) # 1. above pick
        self.move_to(x, y, z,               r, f"{name} down",   SLEEP_SHORT_MOVE) # 2. descend
        self.dobot.set_suction_cup(True)                                             # 3. grip
        time.sleep(SLEEP_SUCTION_ON)                                                 #    wait for suction
        self.move_to(x, y, z + CLEARANCE_Z, r, f"{name} raise",  SLEEP_SHORT_MOVE) # 4. raise

    def place(self, x, y, z, r, name):
        self.get_logger().info(f"=== PLACE {name} ===")
        self.move_to(x, y, z + CLEARANCE_Z, r, f"{name} above",  SLEEP_LONG_MOVE)  # 1. travel to place (long move)
        self.move_to(x, y, z,               r, f"{name} down",   SLEEP_SHORT_MOVE) # 2. descend
        time.sleep(SLEEP_SETTLE)                                                      #    settle before releasing
        self.dobot.set_suction_cup(False)                                             # 3. release
        time.sleep(SLEEP_SUCTION_OFF)                                                 #    wait for suction off
        self.move_to(x, y, z + CLEARANCE_Z, r, f"{name} raise",  SLEEP_SHORT_MOVE) # 4. raise

    def service_callback(self, request, response):
        try:

            self.get_logger().info("Homing robot...")
            self.dobot.set_joint_ptp(0, 0, 0, 0)
            time.sleep(2.0)

            self.pick (*P5, "P5");  self.place(*D1, "D1-bottom")
            self.pick (*P8, "P8");  self.place(*D2, "D2-bottom")
            self.pick (*P7, "P7");  self.place(*D3, "D3-bottom")
            self.pick (*P4, "P4");  self.place(*D4, "D4-bottom")
            self.pick (*P1, "P1");  self.place(*D5, "D5-middle")
            self.pick (*P2, "P2");  self.place(*D6, "D6-middle")
            self.pick (*P3, "P3");  self.place(*D7, "D7-middle")
            self.pick (*P6, "P6");  self.place(*D8, "D8-top")
            self.pick (*P9, "P9");  self.place(*D9, "D9-top")

            self.get_logger().info("Done! Returning home.")
            self.dobot.set_joint_ptp(0, 0, 0, 0)
            time.sleep(2.0)

            response.success = True
            response.message = "Pyramid built: 4 + 3 + 2 blocks."

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = PickAndPlaceNode()
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from .dobot_client import DobotClient
from .dobot_kinematics import inverse_kinematics
from dobot_interface.srv import PickAndPlace
import time

CLEARANCE_Z = 100.0

# ─────────────────────────────────────────────
#  GRID PICK POSITIONS (3x3, block=35mm, gap=5mm, step=40mm)
#  Centre block: x=92, y=-201, z=35
#
#       y=-241   y=-201   y=-161
# x=52  [P1]     [P2]     [P3]
# x=92  [P4]     [P5]     [P6]   ← centre row
# x=132 [P7]     [P8]     [P9]
# ─────────────────────────────────────────────

# Row 1 (x=52)
P1  = ( 52, -161, 35, 0)
P2  = ( 92, -161, 35, 0)
P3  = (132, -161, 35, 0)

# Column 2 (x=92)
P4  = ( 52, -201, 35, 0)
P5  = ( 92, -201, 35, 0)# centre block
P6  = (132, -201, 35, 0)

# Column 3 (x=132)
P7  = ( 52, -241, 35, 0)
P8  = ( 92, -241, 35, 0)
P9  = (132, -241, 35, 0)


# ─────────────────────────────────────────────
#  PYRAMID PLACE POSITIONS
#  First block of bottom row: x=135, y=226, z=35
#  Block size = 35mm → each next block in same layer: +35mm in x
#  Each layer up: +35mm in z
#  Each layer is centred over the one below: +17.5mm x offset per layer
#
#  Bottom layer (z=35):  D1  D2  D3  D4
#  Middle layer (z=70):   D5  D6  D7
#  Top    layer (z=105):    D8  D9
# ─────────────────────────────────────────────

# Bottom layer - 4 blocks (z=35)
D1 = (135,   226, 35,  0)
D2 = (170,   226, 35,  0)
D3 = (205,   226, 35,  0)
D4 = (240,   226, 35,  0)

# Middle layer - 3 blocks (z=70, +17.5 offset)
D5 = (152.5, 226, 70,  0)
D6 = (187.5, 226, 70,  0)
D7 = (222.5, 226, 70,  0)

# Top layer - 2 blocks (z=105, +35 offset)
D8 = (170,   226, 105, 0)
D9 = (205,   226, 105, 0)


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

    def move_to(self, x, y, z, r):
        j1, j2, j3, j4 = inverse_kinematics(x, y, z, r)
        if not self.dobot.is_goal_valid(j1, j2, j3, j4):
            raise ValueError(f"Target pose ({x}, {y}, {z}, {r}) is out of reach.")
        self.dobot.set_joint_ptp(j1, j2, j3, j4)

    def pick(self, x, y, z, r, name):
        self.get_logger().info(f"Picking {name} at ({x}, {y}, {z})")
        self.move_to(x, y, z + CLEARANCE_Z, r)   # move above
        self.move_to(x, y, z, r)                  # descend
        self.dobot.set_suction_cup(True)           # grip
        time.sleep(0.4)
        self.move_to(x, y, z + CLEARANCE_Z, r)   # raise

    def place(self, x, y, z, r, name):
        self.get_logger().info(f"Placing {name} at ({x}, {y}, {z})")
        self.move_to(x, y, z + CLEARANCE_Z, r)   # move above
        self.move_to(x, y, z, r)                  # descend
        self.dobot.set_suction_cup(False)          # release
        time.sleep(0.4)
        self.move_to(x, y, z + CLEARANCE_Z, r)   # raise

    def service_callback(self, request, response):
        try:
            self.dobot = DobotClient()

            # Home
            self.get_logger().info("Homing robot...")
            self.dobot.set_joint_ptp(0, 0, 0, 0)
            time.sleep(0.5)

            # ── Block 1 ──
            self.pick (*P1, "P1")
            self.place(*D1, "D1 - bottom layer")

            # ── Block 2 ──
            self.pick (*P2, "P2")
            self.place(*D2, "D2 - bottom layer")

            # ── Block 3 ──
            self.pick (*P3, "P3")
            self.place(*D3, "D3 - bottom layer")

            # ── Block 4 ──
            self.pick (*P4, "P4")
            self.place(*D4, "D4 - bottom layer")

            # ── Block 5 ──
            self.pick (*P5, "P5")
            self.place(*D5, "D5 - middle layer")

            # ── Block 6 ──
            self.pick (*P6, "P6")
            self.place(*D6, "D6 - middle layer")

            # ── Block 7 ──
            self.pick (*P7, "P7")
            self.place(*D7, "D7 - middle layer")

            # ── Block 8 ──
            self.pick (*P8, "P8")
            self.place(*D8, "D8 - top layer")

            # ── Block 9 ──
            self.pick (*P9, "P9")
            self.place(*D9, "D9 - top layer")

            # Return home
            self.get_logger().info("Done! Returning home.")
            self.dobot.set_joint_ptp(0, 0, 0, 0)

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
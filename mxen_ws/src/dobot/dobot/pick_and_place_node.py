import time

import rclpy
from rclpy.node import Node

from .dobot_client import DobotClient
from .dobot_kinematics import inverse_kinematics

from dobot_interface.srv import PickAndPlace


# =========================================================
# CONFIG
# =========================================================

CLEARANCE_Z = 100.0

BLOCK_SPACING = 10.0
BLOCK_HEIGHT = 35.0


# =========================================================
# BASE PICK POSITION
# =========================================================

px = 92.0
py = -201.0
pz = 35.0
pr = 0.0


# =========================================================
# BASE PLACE POSITION
# =========================================================

dx = 135.0
dy = 226.0
dz = 35.0
dr = 0.0


class PickAndPlaceNode(Node):

    def __init__(self):

        super().__init__("pick_and_place_node")

        # -------------------------------------------------
        # Dobot Client
        # -------------------------------------------------

        self.dobot = DobotClient()

        # -------------------------------------------------
        # ROS2 Service
        # -------------------------------------------------

        self.service = self.create_service(
            PickAndPlace,
            "pick_and_place",
            self.service_callback,
        )

        self.get_logger().info(
            "Pick-and-place service server is ready."
        )

        # =================================================
        # PICK POSITIONS (3x3 GRID)
        # =================================================

        self.pick_positions = [

            (px, py, pz, pr),

            (px + BLOCK_SPACING, py, pz, pr),

            (px + BLOCK_SPACING,
             py - BLOCK_SPACING,
             pz,
             pr),

            (px,
             py - BLOCK_SPACING,
             pz,
             pr),

            (px - BLOCK_SPACING,
             py - BLOCK_SPACING,
             pz,
             pr),

            (px - BLOCK_SPACING,
             py,
             pz,
             pr),

            (px - BLOCK_SPACING,
             py + BLOCK_SPACING,
             pz,
             pr),

            (px,
             py + BLOCK_SPACING,
             pz,
             pr),

            (px + BLOCK_SPACING,
             py + BLOCK_SPACING,
             pz,
             pr),
        ]

        # =================================================
        # PLACE POSITIONS (PYRAMID STACK)
        # =================================================

        self.place_positions = [

            # Bottom Layer
            (dx, dy, dz, dr),

            (dx + BLOCK_SPACING,
             dy,
             dz,
             dr),

            (dx + (2 * BLOCK_SPACING),
             dy,
             dz,
             dr),

            (dx + (3 * BLOCK_SPACING),
             dy,
             dz,
             dr),

            # Middle Layer
            (dx + (0.9375 * BLOCK_SPACING),
             dy,
             dz + BLOCK_HEIGHT,
             dr),

            (dx + (1.875 * BLOCK_SPACING),
             dy,
             dz + BLOCK_HEIGHT,
             dr),

            (dx + (2.8125 * BLOCK_SPACING),
             dy,
             dz + BLOCK_HEIGHT,
             dr),

            # Top Layer
            (dx + BLOCK_SPACING,
             dy,
             dz + (2 * BLOCK_HEIGHT),
             dr),

            (dx + (2 * BLOCK_SPACING),
             dy,
             dz + (2 * BLOCK_HEIGHT),
             dr),
        ]

    # =====================================================
    # INVERSE KINEMATICS
    # =====================================================

    def pose_to_joints(self, x, y, z, r):

        return inverse_kinematics(x, y, z, r)

    # =====================================================
    # MOVE HELPER
    # =====================================================

    def move_pose(self, x, y, z, r):

        theta1, theta2, theta3, theta4 = (
            self.pose_to_joints(x, y, z, r)
        )

        self.dobot.set_joint_ptp(
            theta1,
            theta2,
            theta3,
            theta4,
        )

        time.sleep(0.7)

    # =====================================================
    # VALIDATE GOAL
    # =====================================================

    def goal_valid(self, x, y, z, r):

        try:

            theta1, theta2, theta3, theta4 = (
                self.pose_to_joints(x, y, z, r)
            )

            return self.dobot.is_goal_valid(
                theta1,
                theta2,
                theta3,
                theta4,
            )

        except Exception:

            return False

    # =====================================================
    # VALIDATE ALL POSES
    # =====================================================

    def validate_all_positions(self):

        # Validate Pick Positions
        for pose in self.pick_positions:

            x, y, z, r = pose

            if not self.goal_valid(x, y, z, r):

                raise ValueError(
                    f"Invalid pick pose: "
                    f"({x}, {y}, {z}, {r})"
                )

        # Validate Place Positions
        for pose in self.place_positions:

            x, y, z, r = pose

            if not self.goal_valid(x, y, z, r):

                raise ValueError(
                    f"Invalid place pose: "
                    f"({x}, {y}, {z}, {r})"
                )

    # =====================================================
    # PICK BLOCK
    # =====================================================

    def pick_block(self, x, y, z, r, block_number):

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Move above pick"
        )

        self.move_pose(
            x,
            y,
            z + CLEARANCE_Z,
            r,
        )

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Move to pick"
        )

        self.move_pose(
            x,
            y,
            z,
            r,
        )

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Suction ON"
        )

        self.dobot.set_suction_cup(True)

        time.sleep(0.7)

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Lift block"
        )

        self.move_pose(
            x,
            y,
            z + CLEARANCE_Z,
            r,
        )

    # =====================================================
    # PLACE BLOCK
    # =====================================================

    def place_block(self, x, y, z, r, block_number):

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Move above place"
        )

        self.move_pose(
            x,
            y,
            z + CLEARANCE_Z,
            r,
        )

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Move to place"
        )

        self.move_pose(
            x,
            y,
            z,
            r,
        )

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Suction OFF"
        )

        self.dobot.set_suction_cup(False)

        time.sleep(0.7)

        self.get_logger().info(
            f"[Block {block_number}] "
            f"Lift after place"
        )

        self.move_pose(
            x,
            y,
            z + CLEARANCE_Z,
            r,
        )

    # =====================================================
    # SERVICE CALLBACK
    # =====================================================

    def service_callback(self, request, response):

        try:

            self.get_logger().info(
                "Validating all poses..."
            )

            self.validate_all_positions()

            self.get_logger().info(
                "All poses are valid."
            )

            # -------------------------------------------------
            # HOME POSITION
            # -------------------------------------------------

            self.get_logger().info(
                "Moving to home position..."
            )

            self.dobot.set_joint_ptp(
                0,
                0,
                0,
                0,
            )

            time.sleep(1.0)

            # -------------------------------------------------
            # MAIN LOOP
            # -------------------------------------------------

            for i in range(len(self.pick_positions)):

                pick_pose = self.pick_positions[i]
                place_pose = self.place_positions[i]

                self.get_logger().info(
                    f"========== "
                    f"BLOCK {i + 1} "
                    f"=========="
                )

                # PICK
                self.pick_block(
                    *pick_pose,
                    block_number=i + 1,
                )

                # PLACE
                self.place_block(
                    *place_pose,
                    block_number=i + 1,
                )

            # -------------------------------------------------
            # RETURN HOME
            # -------------------------------------------------

            self.get_logger().info(
                "Returning to home..."
            )

            self.dobot.set_joint_ptp(
                0,
                0,
                0,
                0,
            )

            response.success = True

            response.message = (
                "Multi-block pick-and-place "
                "completed successfully."
            )

            self.get_logger().info(
                response.message
            )

        except Exception as e:

            response.success = False

            response.message = (
                f"Error during operation: {str(e)}"
            )

            self.get_logger().error(
                response.message
            )

        return response


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = PickAndPlaceNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
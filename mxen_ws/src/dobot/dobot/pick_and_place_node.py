from urllib import response

import rclpy
from rclpy.node import Node
from .dobot_client import DobotClient

from .dobot_kinematics import forward_kinematics, inverse_kinematics
from dobot_interface.srv import PickAndPlace  # custom service interface

import time

CLEARANCE_Z = 100.0

px = 92
py = -201
pz = 60
pr = 0

dx = 135
dy = 226
dz = 60
dr = 0


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place_node")
        self.dobot = None

        self.service = self.create_service(
            PickAndPlace,
            "pick_and_place",
            self.service_callback,
        )
        self.get_logger().info("Pick-and-place service server is ready.")

    
    def pose_to_joints(self, x, y, z, r):
        theta1, theta2, theta3, theta4 = inverse_kinematics(x, y, z, r)
        return theta1, theta2, theta3, theta4
    
    def goal_valid(self, x, y, z, r):
        self.dobot = DobotClient()
        theta1, theta2, theta3, theta4 = self.pose_to_joints(x, y, z, r)
        return self.dobot.is_goal_valid(theta1, theta2, theta3, theta4)

    def service_callback(self, response):

        if not self.goal_valid(px, py, pz, pr):
            response.success = False
            response.message = (
                f"Pick pose ({px}, {py}, {pz}, {pr}) is outside the reachable workspace."
            )
            self.get_logger().warn(response.message)
            return response

        if not self.goal_valid(dx, dy, dz, dr):
            response.success = False
            response.message = (
                f"Place pose ({dx}, {dy}, {dz}, {dr}) is outside the reachable workspace."
            )
            self.get_logger().warn(response.message)
            return response

        try:
            self.get_logger().info(
                f"Starting pick-and-place: "
                f"pick=({px},{py},{pz},{pr})  place=({dx},{dy},{dz},{dr})"
            )

            self.dobot = DobotClient()

            self.get_logger().info("Starting")
            self.dobot.set_joint_ptp(0, 0, 0, 0)
            time.sleep(0.5)

            self.get_logger().info("Move 1")
            theta1, theta2, theta3, theta4 = self.pose_to_joints(px, py, pz, pr)
            self.dobot.set_joint_ptp(theta1, theta2, theta3, theta4)
            time.sleep(0.5)

            self.get_logger().info("Move 2")
            theta1, theta2, theta3, theta4 = self.pose_to_joints(px, py, pz + CLEARANCE_Z, pr)
            self.dobot.set_joint_ptp(theta1, theta2, theta3, theta4)

            self.get_logger().info("Move 3")
            self.dobot.set_suction_cup(True)
            time.sleep(0.5)

            self.get_logger().info("Move 4")
            theta1, theta2, theta3, theta4 = self.pose_to_joints(dx, dy, dz + CLEARANCE_Z, dr)
            self.dobot.set_joint_ptp(theta1, theta2, theta3, theta4)

            self.get_logger().info("Move 5")
            theta1, theta2, theta3, theta4 = self.pose_to_joints(dx, dy, dz, dr)
            self.dobot.set_joint_ptp(theta1, theta2, theta3, theta4)
            time.sleep(0.5)

            self.get_logger().info("Move 6")
            self.dobot.set_suction_cup(False)
            time.sleep(0.5)

            self.get_logger().info("Move 7")
            self.dobot.set_joint_ptp(0, 0, 0, 0)

            response.success = True
            response.message = "Pick-and-place completed successfully"

        except Exception as e:
            self.dobot = None
            response.success = False
            response.message = f"Error during pick-and-place: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = PickAndPlaceNode()
            rclpy.spin(node)
    except (KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
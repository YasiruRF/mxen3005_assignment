import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from dobot.dobot_client import DobotClient

from dobot_interface.action import JointPTP
from dobot_interface.action import PosePTP
from dobot_interface.srv import PickAndPlace


class TeleopControlNode(Node):

    def __init__(self):

        super().__init__("teleop_node")

        self.dobot = DobotClient()

        self.mode = "joint"

        self.current_joints = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.current_pose = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.suction_on = False

        # Prevent duplicate commands while key held
        self.key_active = set()

        # Joint limits
        self.J1MIN = -90.0
        self.J1MAX = 90.0

        self.J2MIN = 0.0
        self.J2MAX = 70.0

        self.J3MIN = -15.0
        self.J3MAX = 40.0

        self.J4MIN = -90.0
        self.J4MAX = 90.0

        # Cartesian limits
        self.X_MIN = 50.0
        self.X_MAX = 270.0

        self.Y_MIN = -250.0
        self.Y_MAX = 250.0

        self.Z_MIN = 0.0
        self.Z_MAX = 220.0

        self.R_MIN = -180.0
        self.R_MAX = 180.0

        # Action clients
        self.joint_client = ActionClient(
            self,
            JointPTP,
            "set_joint_ptp"
        )

        self.cartesian_client = ActionClient(
            self,
            PosePTP,
            "set_cartesian_ptp"
        )

        self.pick_and_place_client = self.create_client(
            PickAndPlace,
            "pick_and_place"
        )

        self.get_logger().info(
            "Waiting for action servers..."
        )

        self.joint_client.wait_for_server()
        self.cartesian_client.wait_for_server()

        self.get_logger().info(
            "Action servers connected"
        )

        self.get_logger().info(
            "Waiting for pick-and-place service..."
        )

        # Subscribers
        self.keyboard_subscriber = (
            self.create_subscription(
                String,
                "/keyboard",
                self.keyboard_callback,
                10
            )
        )

        self.joint_subscriber = (
            self.create_subscription(
                JointState,
                "joint_state",
                self.joint_callback,
                10
            )
        )

        self.pose_subscriber = (
            self.create_subscription(
                Pose,
                "pose",
                self.pose_callback,
                10
            )
        )

        self.get_logger().info(
            "Teleop control node started"
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def joint_callback(self, msg):

        self.current_joints = list(msg.position)

    def pose_callback(self, msg):

        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        qz = msg.orientation.z
        qw = msg.orientation.w

        r_rad = (
            2.0 * math.atan2(qz, qw)
        )

        r_deg = math.degrees(r_rad)

        self.current_pose = [
            x,
            y,
            z,
            r_deg
        ]

    # =========================================================
    # KEYBOARD CALLBACK
    # =========================================================

    def keyboard_callback(self, msg):

        key = msg.data

        # -----------------------------------------------------
        # MODE TOGGLE
        # -----------------------------------------------------

        if key == "MODE_TOGGLE":

            if self.mode == "joint":
                self.mode = "cartesian"
            else:
                self.mode = "joint"

            self.get_logger().info(
                f"Mode switched to: {self.mode}"
            )

            return

        # -----------------------------------------------------
        # HOME
        # -----------------------------------------------------

        if key == "HOME":

            self.get_logger().info(
                "Starting homing"
            )

            self.dobot.start_homing()

            return

        # -----------------------------------------------------
        # SUCTION
        # -----------------------------------------------------

        if key == "SUCTION_TOGGLE":

            self.suction_on = (
                not self.suction_on
            )

            self.dobot.set_suction_cup(
                self.suction_on
            )

            self.get_logger().info(
                f"Suction: {self.suction_on}"
            )

            return

        if key == "AUTO_STACK":

            self.call_pick_and_place_service()

            return

        # =====================================================
        # KEY DOWN
        # =====================================================

        if key.endswith("_DOWN"):

            if key in self.key_active:
                return

            self.key_active.add(key)

            if self.mode == "joint":
                self.handle_joint_key_down(key)

            elif self.mode == "cartesian":
                self.handle_cartesian_key_down(key)

        # =====================================================
        # KEY UP
        # =====================================================

        elif key.endswith("_UP"):

            down_key = key.replace(
                "_UP",
                "_DOWN"
            )

            self.key_active.discard(
                down_key
            )

            self.stop_motion()

    # =========================================================
    # JOINT MODE
    # =========================================================

    def handle_joint_key_down(self, key):

        j1, j2, j3, j4 = (
            self.current_joints
        )

        # J1
        if key == "LEFT_DOWN":

            self.send_joint_goal(
                self.J1MIN,
                j2,
                j3,
                j4
            )

        elif key == "RIGHT_DOWN":

            self.send_joint_goal(
                self.J1MAX,
                j2,
                j3,
                j4
            )

        # J2
        elif key == "UP_DOWN":

            self.send_joint_goal(
                j1,
                self.J2MAX,
                j3,
                j4
            )

        elif key == "DOWN_DOWN":

            self.send_joint_goal(
                j1,
                self.J2MIN,
                j3,
                j4
            )

        # J3
        elif key == "W_DOWN":

            self.send_joint_goal(
                j1,
                j2,
                self.J3MAX,
                j4
            )

        elif key == "S_DOWN":

            self.send_joint_goal(
                j1,
                j2,
                self.J3MIN,
                j4
            )

        # J4
        elif key == "A_DOWN":

            self.send_joint_goal(
                j1,
                j2,
                j3,
                self.J4MIN
            )

        elif key == "D_DOWN":

            self.send_joint_goal(
                j1,
                j2,
                j3,
                self.J4MAX
            )

    # =========================================================
    # CARTESIAN MODE
    # =========================================================

    def handle_cartesian_key_down(self, key):

        x, y, z, r = (
            self.current_pose
        )

        # X
        if key == "UP_DOWN":

            self.send_cartesian_goal(
                self.X_MAX,
                y,
                z,
                r
            )

        elif key == "DOWN_DOWN":

            self.send_cartesian_goal(
                self.X_MIN,
                y,
                z,
                r
            )

        # Y
        elif key == "LEFT_DOWN":

            self.send_cartesian_goal(
                x,
                self.Y_MAX,
                z,
                r
            )

        elif key == "RIGHT_DOWN":

            self.send_cartesian_goal(
                x,
                self.Y_MIN,
                z,
                r
            )

        # Z
        elif key == "W_DOWN":

            self.send_cartesian_goal(
                x,
                y,
                self.Z_MAX,
                r
            )

        elif key == "S_DOWN":

            self.send_cartesian_goal(
                x,
                y,
                self.Z_MIN,
                r
            )

        # R
        elif key == "A_DOWN":

            self.send_cartesian_goal(
                x,
                y,
                z,
                self.R_MIN
            )

        elif key == "D_DOWN":

            self.send_cartesian_goal(
                x,
                y,
                z,
                self.R_MAX
            )

    # =========================================================
    # STOP MOTION
    # =========================================================

    def stop_motion(self):

        self.get_logger().info(
            "Stopping motion"
        )

        self.dobot.stop_current_action()

    # =========================================================
    # ACTIONS
    # =========================================================

    def send_joint_goal(
        self,
        j1,
        j2,
        j3,
        j4
    ):

        goal_msg = JointPTP.Goal()

        goal_msg.joint_goal = [
            float(j1),
            float(j2),
            float(j3),
            float(j4)
        ]

        self.get_logger().info(
            f"Joint goal: "
            f"{goal_msg.joint_goal}"
        )

        self.joint_client.send_goal_async(
            goal_msg
        )

    def send_cartesian_goal(
        self,
        x,
        y,
        z,
        r
    ):

        goal_msg = PosePTP.Goal()

        goal_msg.pose_goal = [
            float(x),
            float(y),
            float(z),
            float(r)
        ]

        self.get_logger().info(
            f"Cartesian goal: "
            f"{goal_msg.pose_goal}"
        )

        self.cartesian_client.send_goal_async(
            goal_msg
        )

    def call_pick_and_place_service(self):

        if not self.pick_and_place_client.wait_for_service(timeout_sec=0.5):

            self.get_logger().warn(
                "Pick-and-place service not available"
            )

            return

        req = PickAndPlace.Request()
        req.pick_pose = [92.0, -201.0, 35.0, 0.0]
        req.place_pose = [135.0, 226.0, 35.0, 0.0]

        future = self.pick_and_place_client.call_async(req)
        future.add_done_callback(self.pick_callback)

    def pick_callback(self, future):

        result = future.result()

        if result:

            self.get_logger().info(
                f"Pick-and-place: {result.message}"
            )

        else:

            self.get_logger().warn(
                "Pick-and-place failed"
            )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = TeleopControlNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()

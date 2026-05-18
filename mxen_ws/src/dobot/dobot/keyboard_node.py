import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math

from .keyboard import Keyboard
from dobot_interface.action import JointPTP, PosePTP

motion_map = {
                #  key          axis  direction
                "KEY_LEFT":  (0, +1),  # j1 / y  +
                "KEY_RIGHT": (0, -1),  # j1 / y  -
                "KEY_UP":    (1, +1),  # j2 / x  +
                "KEY_DOWN":  (1, -1),  # j2 / x  -
                "KEY_W":     (2, +1),  # j3 / z  +
                "KEY_S":     (2, -1),  # j3 / z  -
                "KEY_A":     (3, +1),  # j4 / r  +
                "KEY_D":     (3, -1),  # j4 / r  -
            }

class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")
        self.kb = Keyboard()

        # State flags
        self.keyboard_enabled = False
        self.cartesian_mode = False

        # The key currently held down (for continuous motion)
        self.active_motion_key = None
        self.step = 1.0  # degrees (joint) or mm (cartesian)

        # Working goals — seeded from live robot state via subscriptions
        self.joint_goal = [0.0, 0.0, 0.0, 0.0]
        self.pose_goal  = [0.0, 0.0, 0.0, 0.0]

        # Track whether we have received an initial state yet
        self.joint_state_received = False
        self.pose_received = False

        # ── Subscriptions ──────────────────────────────────────────────
        self.joint_state_sub = self.create_subscription(
            JointState,
            "joint_state",
            self.joint_state_callback,
            10,
        )
        self.pose_sub = self.create_subscription(
            Pose,
            "pose",
            self.pose_callback,
            10,
        )

        # ── Action clients ─────────────────────────────────────────────
        self.joint_client = ActionClient(self, JointPTP, "set_joint_ptp")
        self.pose_client  = ActionClient(self, PosePTP,  "set_cartesian_ptp")

        # ── Service clients ────────────────────────────────────────────
        self.homing_client  = self.create_client(Trigger, "homing")
        self.suction_client = self.create_client(Trigger, "suction_cup")
        self.pick_and_place_client = self.create_client(Trigger, "pick_and_place")

        # ── 20 Hz keyboard poll timer ──────────────────────────────────
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            "Keyboard node ready. Press 8 to enable, M to toggle joint/cartesian mode."
        )

    # ------------------------------------------------------------------ #
    #  Subscriptions — keep goals in sync with real robot state            #
    # ------------------------------------------------------------------ #

    def joint_state_callback(self, msg: JointState):
        """Seed joint_goal from live joint_state so first keypress is relative."""
        if len(msg.position) >= 4:
            self.joint_goal = list(msg.position[:4])
            if not self.joint_state_received:
                self.joint_state_received = True
                self.get_logger().info(
                    f"Initial joint state received: {[f'{v:.2f}' for v in self.joint_goal]}"
                )

    def pose_callback(self, msg: Pose):
        """Seed pose_goal from live pose so first keypress is relative."""
        # Convert quaternion back to yaw for the r component
        yaw = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.pose_goal = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            math.degrees(yaw),
        ]
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info(
                f"Initial pose received: {[f'{v:.2f}' for v in self.pose_goal]}"
            )

    @staticmethod
    def quaternion_to_yaw(qx, qy, qz, qw) -> float:
        """Extract yaw (rotation about Z) from a quaternion."""
        return math.atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz))

    # ------------------------------------------------------------------ #
    #  Timer — read one key event per tick and repeat motion if held       #
    # ------------------------------------------------------------------ #

    def timer_callback(self):
        keycode, keystate = self.kb.read_key()

        KEY_DOWN = 1
        KEY_UP   = 0

        # ── Handle a real key event ────────────────────────────────────
        if keycode is not None:
            # Toggle keys — only fire on press
            if keystate == KEY_DOWN:
                if keycode == "KEY_8":
                    self.keyboard_enabled = not self.keyboard_enabled
                    state = "ENABLED" if self.keyboard_enabled else "DISABLED"
                    self.get_logger().info(f"Keyboard control {state}")
                    return

                if not self.keyboard_enabled:
                    return

                if keycode == "KEY_M":
                    self.cartesian_mode = not self.cartesian_mode
                    mode = "CARTESIAN" if self.cartesian_mode else "JOINT"
                    self.get_logger().info(f"Switched to {mode} mode")
                    return

                if keycode == "KEY_H":
                    self.call_trigger(self.homing_client, "homing")
                    return

                if keycode == "KEY_E":
                    self.call_trigger(self.suction_client, "suction_cup")
                    return
                
                if keycode == "KEY_P":
                    self.call_trigger(self.pick_and_place_client, "pick_and_place")
                    return

            if not self.keyboard_enabled:
                return

            # Motion keys — track press/release for continuous motion
           

            if keycode in motion_map:
                if keystate == KEY_DOWN:
                    self.active_motion_key = keycode
                elif keystate == KEY_UP:
                    if self.active_motion_key == keycode:
                        self.active_motion_key = None
                return

        # ── Repeat motion while key is held ───────────────────────────
        if self.keyboard_enabled and self.active_motion_key is not None:
            axis, direction = motion_map[self.active_motion_key]
            self.apply_step(axis, direction)

    # ------------------------------------------------------------------ #
    #  Step application                                                    #
    # ------------------------------------------------------------------ #

    def apply_step(self, axis: int, direction: int):
        delta = self.step * direction

        if self.cartesian_mode:
            self.pose_goal[axis] += delta
            self.get_logger().info(
                f"Cartesian goal → "
                f"x={self.pose_goal[0]:.1f}  y={self.pose_goal[1]:.1f}  "
                f"z={self.pose_goal[2]:.1f}  r={self.pose_goal[3]:.1f}"
            )
            self.send_cartesian_goal()
        else:
            self.joint_goal[axis] += delta
            self.get_logger().info(
                f"Joint goal → "
                f"j1={self.joint_goal[0]:.1f}  j2={self.joint_goal[1]:.1f}  "
                f"j3={self.joint_goal[2]:.1f}  j4={self.joint_goal[3]:.1f}"
            )
            self.send_joint_goal()

    # ------------------------------------------------------------------ #
    #  Action goal senders                                                 #
    # ------------------------------------------------------------------ #

    def send_joint_goal(self):
        if not self.joint_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("JointPTP action server not available")
            return

        goal = JointPTP.Goal()
        goal.joint_goal = self.joint_goal
        self.joint_client.send_goal_async(
            goal,
            feedback_callback=self.joint_feedback_callback,
        )

    def send_cartesian_goal(self):
        if not self.pose_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("PosePTP action server not available")
            return

        goal = PosePTP.Goal()
        goal.pose_goal = self.pose_goal
        self.pose_client.send_goal_async(
            goal,
            feedback_callback=self.cartesian_feedback_callback,
        )

    # ------------------------------------------------------------------ #
    #  Feedback callbacks                                                  #
    # ------------------------------------------------------------------ #

    def joint_feedback_callback(self, feedback_handle):
        fb = feedback_handle.feedback
        self.get_logger().debug(f"Joint feedback: {fb.joint_present}")

    def cartesian_feedback_callback(self, feedback_handle):
        fb = feedback_handle.feedback
        self.get_logger().debug(f"Cartesian feedback: {fb.present_pose}")

    # ------------------------------------------------------------------ #
    #  Service helper                                                      #
    # ------------------------------------------------------------------ #

    def call_trigger(self, client, name: str):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"Service '{name}' not available")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(f"{name}: {f.result().message}")
        )


# --------------------------------------------------------------------------- #

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = KeyboardNode()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
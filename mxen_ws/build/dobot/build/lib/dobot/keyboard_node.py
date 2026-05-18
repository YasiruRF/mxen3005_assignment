import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger

from .keyboard import Keyboard
from dobot_interface.action import JointPTP, PosePTP


class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")
        self.kb = Keyboard()

        self.keyboard_enabled = False
        self.cartesian_mode = False
        self.suction_enabled = False

        self.active_motion_key = None
        self.step = 1.0

        # Working goals — updated incrementally on each key tick
        self.joint_goal = [0.0, 0.0, 0.0, 0.0]
        self.pose_goal  = [0.0, 0.0, 0.0, 0.0]

        # Action clients
        self.joint_client = ActionClient(self, JointPTP, "set_joint_ptp")
        self.pose_client  = ActionClient(self, PosePTP,  "set_cartesian_ptp")

        # Service clients
        self.homing_client  = self.create_client(Trigger, "homing")
        self.suction_client = self.create_client(Trigger, "suction_cup")

        # Poll the keyboard at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            "Keyboard node ready. Press 8 to enable control, m to toggle mode."
        )

    # ------------------------------------------------------------------ #
    #  Timer — read one key event per tick                                 #
    # ------------------------------------------------------------------ #

    def timer_callback(self):
        keycode, keystate = self.kb.read_key()
        if keycode is None:
            return

        KEY_DOWN = 1
        KEY_UP   = 0

        # --- Toggle keys (act on press only) ---
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

        if not self.keyboard_enabled:
            return

        # --- Motion keys (continuous while held) ---
        #
        # Mapping:
        #   Joint mode      : j1=left/right  j2=up/down  j3=w/s  j4=a/d
        #   Cartesian mode  : y =left/right  x =up/down  z =w/s  r =a/d
        #
        # keystate 1 = pressed, 0 = released

        motion_map = {
            "KEY_LEFT":  ( 0, +1),   # axis 0 (j1 / y) positive
            "KEY_RIGHT": ( 0, -1),   # axis 0 (j1 / y) negative
            "KEY_UP":    ( 1, +1),   # axis 1 (j2 / x) positive
            "KEY_DOWN":  ( 1, -1),   # axis 1 (j2 / x) negative
            "KEY_W":     ( 2, +1),   # axis 2 (j3 / z) positive
            "KEY_S":     ( 2, -1),   # axis 2 (j3 / z) negative
            "KEY_A":     ( 3, +1),   # axis 3 (j4 / r) positive
            "KEY_D":     ( 3, -1),   # axis 3 (j4 / r) negative
        }

        if keycode not in motion_map:
            return

        axis, direction = motion_map[keycode]

        if keystate == KEY_UP:
            # Key released — stop tracking this key
            if self.active_motion_key == keycode:
                self.active_motion_key = None
            return

        if keystate == KEY_DOWN:
            self.active_motion_key = keycode
            self.apply_step(axis, direction)

    # ------------------------------------------------------------------ #
    #  Step application                                                    #
    # ------------------------------------------------------------------ #

    def apply_step(self, axis: int, direction: int):
        delta = self.step * direction

        if self.cartesian_mode:
            self.pose_goal[axis] += delta
            self.get_logger().info(
                f"Cartesian goal: x={self.pose_goal[0]:.1f}  "
                f"y={self.pose_goal[1]:.1f}  "
                f"z={self.pose_goal[2]:.1f}  "
                f"r={self.pose_goal[3]:.1f}"
            )
            self.send_cartesian_goal()
        else:
            self.joint_goal[axis] += delta
            self.get_logger().info(
                f"Joint goal: j1={self.joint_goal[0]:.1f}  "
                f"j2={self.joint_goal[1]:.1f}  "
                f"j3={self.joint_goal[2]:.1f}  "
                f"j4={self.joint_goal[3]:.1f}"
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
            lambda f: self.get_logger().info(
                f"{name}: {f.result().message}"
            )
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
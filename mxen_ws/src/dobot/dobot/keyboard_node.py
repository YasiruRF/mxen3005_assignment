import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger

from ...dobot_teleop.dobot_teleop.keyboard import Keyboard
from dobot_interface.srv import PickAndPlace


class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")
        self.kb = Keyboard()

        # State
        self.keyboard_enabled = False

        # Service clients
        self.homing_client = self.create_client(Trigger, "homing")
        self.suction_client = self.create_client(Trigger, "suction_cup")
        self.pick_and_place_client = self.create_client(PickAndPlace, "pick_and_place")

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Keyboard ready (8=enable, H=home, E=suction, P=pick-place)")

    def timer_callback(self):
        keycode, keystate = self.kb.read_key()

        KEY_DOWN = 1

        if keycode is None:
            return

        # Enable toggle always allowed
        if keystate == KEY_DOWN:
            if keycode == "KEY_8":
                self.keyboard_enabled = not self.keyboard_enabled
                self.get_logger().info(f"Keyboard enabled: {self.keyboard_enabled}")
                return

            if not self.keyboard_enabled:
                return

            if keycode == "KEY_H":
                self.call_trigger(self.homing_client, "homing")
                return

            if keycode == "KEY_E":
                self.call_trigger(self.suction_client, "suction")
                return

            if keycode == "KEY_P":
                self.call_pick_and_place_service()
                return

    def call_trigger(self, client, name: str):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{name} service not available")
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"{name}: {f.result().message if f.result() else 'no response'}"
            )
        )

    def call_pick_and_place_service(self):
        if not self.pick_and_place_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("Pick-and-place service not available")
            return

        req = PickAndPlace.Request()
        req.pick_pose = [92.0, -201.0, 35.0, 0.0]
        req.place_pose = [135.0, 226.0, 35.0, 0.0]

        future = self.pick_and_place_client.call_async(req)
        future.add_done_callback(self.pick_callback)

    def pick_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info(f"Pick-and-place: {result.message}")
        else:
            self.get_logger().warn("Pick-and-place failed")


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = KeyboardNode()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
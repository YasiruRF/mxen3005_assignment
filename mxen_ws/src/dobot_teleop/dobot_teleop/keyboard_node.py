import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from dobot_teleop.keyboard import Keyboard


class KeyboardTeleopNode(Node):

    KEYBOARD_ENABLE_KEY = ("KEY_8", 1)

    MOMENTARY_COMMANDS = {
        ("KEY_W", 1): "W_DOWN",
        ("KEY_W", 0): "W_UP",
        ("KEY_S", 1): "S_DOWN",
        ("KEY_S", 0): "S_UP",
        ("KEY_A", 1): "A_DOWN",
        ("KEY_A", 0): "A_UP",
        ("KEY_D", 1): "D_DOWN",
        ("KEY_D", 0): "D_UP",
        ("KEY_UP", 1): "UP_DOWN",
        ("KEY_UP", 0): "UP_UP",
        ("KEY_DOWN", 1): "DOWN_DOWN",
        ("KEY_DOWN", 0): "DOWN_UP",
        ("KEY_LEFT", 1): "LEFT_DOWN",
        ("KEY_LEFT", 0): "LEFT_UP",
        ("KEY_RIGHT", 1): "RIGHT_DOWN",
        ("KEY_RIGHT", 0): "RIGHT_UP",
    }

    TOGGLE_COMMANDS = {
        "KEY_E": "SUCTION_TOGGLE",
        "KEY_H": "HOME",
        "KEY_M": "MODE_TOGGLE",
        "KEY_P": "AUTO_STACK",
    }

    def __init__(self):
        super().__init__("keyboard_node")

        self.kb = Keyboard()

        self.publisher = self.create_publisher(
            String,
            "/keyboard",
            10,
        )
        self.keyboard_enabled = False
        self.toggle_keys_pressed = set()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            "Keyboard controls: 8 enable/disable, W/S/A/D and arrows move, "
            "M toggles mode, H homes, E toggles suction, P runs auto stack"
        )

    def timer_callback(self):
        key = self.kb.read_key()
        if key == (None, None):
            return

        if key == self.KEYBOARD_ENABLE_KEY:
            self.keyboard_enabled = not self.keyboard_enabled
            self.get_logger().info(
                f"Keyboard enabled: {self.keyboard_enabled}"
            )
            return

        if not self.keyboard_enabled:
            return

        if key in self.MOMENTARY_COMMANDS:
            self.publish_command(self.MOMENTARY_COMMANDS[key])
            return

        keycode, keystate = key

        if keycode in self.TOGGLE_COMMANDS:
            if keystate == 1:
                if keycode in self.toggle_keys_pressed:
                    return

                self.toggle_keys_pressed.add(keycode)
                self.publish_command(self.TOGGLE_COMMANDS[keycode])
                return

            if keystate == 0:
                self.toggle_keys_pressed.discard(keycode)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):

    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dobot_teleop.keyboard import Keyboard

class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__("keyboard_teleop_node")

        self.kb = Keyboard()

        self.publisher = self.create_publisher(
            String,
            "/keyboard",
            10
        )
        self.keyboard_enabled = False
        self.timer = self.create_timer(0.05,self.timer_callback)

    def timer_callback(self):
        key = self.kb.read_key()
        if key == (None, None):
            return

        if key == ('KEY_8', 1):
            self.keyboard_enabled = not self.keyboard_enabled

            self.get_logger().info(
                f"Keyboard enabled: {self.keyboard_enabled}")
            return
        if not self.keyboard_enabled:
            return

        msg = String()

      
        if key == ('KEY_W', 1):
            msg.data = "W_DOWN"
       
        elif key == ('KEY_W', 0):
            msg.data = "W_UP"

        elif key == ('KEY_S', 1):
            msg.data = "S_DOWN"

        elif key == ('KEY_S', 0):
            msg.data = "S_UP"     

        elif key == ('KEY_A', 1):
            msg.data = "A_DOWN"

        elif key == ('KEY_A', 0):
            msg.data = "A_UP"

        elif key == ('KEY_D', 1):
            msg.data = "D_DOWN"

        elif key == ('KEY_D', 0):
            msg.data = "D_UP"  

        elif key == ('KEY_UP', 1):
            msg.data = "UP_DOWN"

        elif key == ('KEY_UP', 0):
            msg.data = "UP_UP"

        elif key == ('KEY_DOWN', 1):
            msg.data = "DOWN_DOWN"

        elif key == ('KEY_DOWN', 0):
            msg.data = "DOWN_UP"

        elif key == ('KEY_LEFT', 1):
            msg.data = "LEFT_DOWN"

        elif key == ('KEY_LEFT', 0):
            msg.data = "LEFT_UP"

        elif key == ('KEY_RIGHT', 1):
            msg.data = "RIGHT_DOWN"

        elif key == ('KEY_RIGHT', 0):
            msg.data = "RIGHT_UP"

        elif key == ('KEY_E', 1):
            msg.data = "SUCTION_TOGGLE"

        elif key == ('KEY_H', 1):
            msg.data = "HOME"

        elif key == ('KEY_M', 1):
            msg.data = "MODE_TOGGLE"

        elif key == ('KEY_P', 1):
            msg.data = "AUTO_STACK"

        else:
            return

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

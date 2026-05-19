import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import unittest
import threading

class TestKeyboardCommunication(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_topic_publication(self):
        """Verify that a node can receive messages on the /keyboard topic"""
        node = rclpy.create_node('test_listener')
        received_messages = []

        def cb(msg):
            received_messages.append(msg.data)

        node.create_subscription(String, '/keyboard', cb, 10)
        
        # Create a mock publisher
        pub = node.create_publisher(String, '/keyboard', 10)
        
        # Spin for a moment in a thread
        msg = String()
        msg.data = "KEY_W_1"
        
        # Give it a few tries to propagate
        for _ in range(5):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            if "KEY_W_1" in received_messages:
                break

        self.assertIn("KEY_W_1", received_messages)
        node.destroy_node()

if __name__ == '__main__':
    unittest.main()

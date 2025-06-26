import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class TestCommands(Node):
    def __init__(self):
        super().__init__('test_commmands')
        self.publisher = self.create_publisher(String, 'serial_commands', 10)

        self.subscription = self.create_subscription(
            String,
            'stepper_position',
            self.position_callback,
            10
        )

        self.get_logger().info("Test command publisher is ready. Enter a command:")

        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")

    def position_callback(self, msg):
        self.get_logger().info(f"Last Stepper Motor Position: {msg.data}")

    def command_input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Enter a command or 'exit' to quit: ").strip()
                if user_input.lower() == 'exit':
                    rclpy.shutdown()
                    break
                self.publish_command(user_input)
            except KeyboardInterrupt:
                rclpy.shutdown()
                break

def main(args=None):
    rclpy.init(args=args)
    node = TestCommands()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
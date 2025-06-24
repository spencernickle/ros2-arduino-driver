import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestCommands(Node):
    def __init__(self):
        super().__init__('test_commmands')
        self.publisher = self.create_publisher(String, 'serial_commands', 10)

        self.get_logger().info("Test command publisher is ready. Enter a command:")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")

def main(args=None):
    rclpy.init(args=args)

    test_commands_publisher = TestCommands()

    while rclpy.ok():
        try:
            user_input = input("enter a command or 'exit' to quit: ")

            if user_input.lower() == 'exit':
                break

            test_commands_publisher.publish_command(user_input)

        except KeyboardInterrupt:
            break

    test_commands_publisher.destroy_node()
    rclpy.shutdown()
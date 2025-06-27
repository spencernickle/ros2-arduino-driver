import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class TestCommands(Node):
    def __init__(self):
        super().__init__('test_commands')
        self.publisher = self.create_publisher(String, 'serial_commands', 10)

        self.subscription = self.create_subscription(
            String,
            'serial_reading',
            self.serial_reading_callback,
            10
        )


        self.running = True
        self.input_thread = None
        
    def start_input_thread(self):
        self.get_logger().info("Test command publisher is ready. Enter a command:")
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")

    def serial_reading_callback(self, line):
        self.get_logger().info(f"{line.data}")

    def command_input_loop(self):
        while self.running and rclpy.ok():
            try:
                user_input = input("Enter a command or 'exit' to quit: ").strip()
                if user_input.lower() == 'exit':
                    self.running = False
                    rclpy.shutdown()
                    break
                self.publish_command(user_input)
            except KeyboardInterrupt:
                self.running = False
                rclpy.shutdown()
                break

def main(args=None):
    rclpy.init(args=args)
    node = TestCommands()
    node.start_input_thread()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node)
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down from KeyboardInterrupt.")
    finally:
        node.running = False
        if node.input_thread is not None:
            node.input_thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
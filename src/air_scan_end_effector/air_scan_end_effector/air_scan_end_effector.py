import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import serial

class SerialHandler:
    def __init__(self, port, baud, timeout):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.connect()

    def connect(self):
        self.arduino = serial.Serial(port = self.port, baudrate = self.baud, timeout = self.timeout)

    def write(self, command, value = ""):
        cmd = f'{command} {value} '
        print(cmd)
        self.arduino.write(cmd.encode())

class AirScanEndEffector(Node):
	def __init__(self):
		super().__init__('air_scan_driver')

		self.declare_parameters(
			namespace="",
			parameters=[
				("baud_rate", 115200),
                ("serial_port", "/dev/ttyACM0"),
                ("timeout", 1.0)
			]
		)

		self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
		self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
		self.timeout = self.get_parameter("timeout").get_parameter_value().double_value
		
		self.serial_handler = SerialHandler(self.serial_port, self.baud_rate, self.timeout)

		self.subscription = self.create_subscription(
            String,
            'serial_commands',
            self.command_callback,
            10
        )
		self.position_publisher = self.create_publisher(String, 'stepper_position', 10)
        
		self.get_logger().info("Node is ready.")
		time.sleep(1)
            
	def command_callback(self, msg):
		command_data = msg.data.strip()
		parts = command_data.split(maxsplit=1)
		command = parts[0]
		value = parts[1] if len(parts) > 1 else ""
		if command == "readPOS":
			target_prefix = f"MOTOR {value.strip()} POSITION"
			max_attempts = 10
			found_line = None

			for _ in range(max_attempts):
				line = self.serial_handler.arduino.readline().decode().strip()
				if line.startswith(target_prefix):
					found_line = line
					break

			if found_line:
				self.get_logger().info(f"{found_line}")
				msg = String()
				msg.data = found_line
				self.position_publisher.publish(msg)
		else:
			self.serial_handler.write(command, value)
		

def main(args=None):
	rclpy.init(args=args)

	air_scan_end_effector = AirScanEndEffector()

	rate = air_scan_end_effector.create_rate(20)
	while rclpy.ok():
		rclpy.spin(air_scan_end_effector)

	air_scan_end_effector.destroy_node()
	rclpy.shutdown()

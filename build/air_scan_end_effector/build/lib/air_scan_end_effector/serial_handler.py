import serial
import time

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










if "__main__" == __name__:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    while True:
        msg = input("Enter command ")
        if msg.lower() == 'exit':
            break
        ser.write(msg.encode())
    
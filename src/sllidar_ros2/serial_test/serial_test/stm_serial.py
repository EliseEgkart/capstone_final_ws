import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Battery:
    def __init__(self, port, baudrate, device_id):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        self.device_id = device_id
    def create_packet(self, command, data_length, data=None):
        packet = [
            0xAA, 0x00,          # Start Sequence
            self.device_id,      # Device ID
            command,             # Command
            data_length,         # Data Length
        ]
        if data:
            packet.append(data)
        checksum = 0
        for byte in packet[2:]:
            checksum ^= byte
        packet.append(checksum)   # Checksum
        packet.extend([0x00, 0xAA])  # End Sequence
        return bytes(packet)
    def send_command(self, command, data_length, data=None):
        packet = self.create_packet(command, data_length, data)
        self.ser.write(packet)
        print(f"send packet: {packet}")
        response = self.ser.read(28)  # Read the expected response length
        return response
    def read_battery_status(self):
        command = 0x53  # Command for BMS status information
        response = self.send_command(command, 0x00)
        print(f"Battery response: {response}")
        if response:
            return self.parse_response(response)
        return None
    def set_emergency(self, state):
        command = 0x46  # Command for setting emergency state
        data = state & 0x01  # Only the first bit is used
        response = self.send_command(command, 0x01, data)
        print(f"Emergency response: {response}")
        if response:
            return self.parse_emergency_response(response)
        return None
    def parse_response(self, response):
        if response[0:2] == b'\xAA\x00' and response[-2:] == b'\x00\xAA':
            device_id = response[2]
            command = response[3]
            length = response[4]
            data = response[5:-3]
            checksum = response[-3]
            calc_checksum = 0
            for byte in response[2:-3]:
                calc_checksum ^= byte
            print(f"Device ID: {device_id}")
            print(f"Command: {command}")
            print(f"Length: {length}")
            print(f"Data: {data}")
            print(f"Checksum: {checksum}")
            print(f"Calculated Checksum: {calc_checksum}")
            if checksum == calc_checksum:
                return self.extract_battery_data(data)
        return None
    def parse_emergency_response(self, response):
        if response[0:2] == b'\xAA\x00' and response[-2:] == b'\x00\xAA':
            device_id = response[2]
            command = response[3]
            length = response[4]
            data = response[5:-3]
            checksum = response[-3]
            calc_checksum = 0
            for byte in response[2:-3]:
                calc_checksum ^= byte
            print(f"Device ID: {device_id}")
            print(f"Command: {command}")
            print(f"Length: {length}")
            print(f"Data: {data}")
            print(f"Checksum: {checksum}")
            print(f"Calculated Checksum: {calc_checksum}")
            if checksum == calc_checksum:
                return {
                    "Emergency State": data[0] & 0x01
                }
        return None
    def extract_battery_data(self, data):
        return {
            "SOC": int.from_bytes(data[0:2], byteorder='big') / 100,
            "SOH": int.from_bytes(data[2:4], byteorder='big') / 100,
            "Voltage": int.from_bytes(data[4:6], byteorder='big'),
            "Charging State": data[6],
            "Current": int.from_bytes(data[7:9], byteorder='big') / 100,  # Assume current is in 0.01A
            "Cell Balancing": int.from_bytes(data[9:11], byteorder='big'),
            "Cell Temperature": data[11],
            "PCB Temperature": data[12],
            "Warning": int.from_bytes(data[13:15], byteorder='big'),
            "Error": int.from_bytes(data[15:17], byteorder='big'),
            "Setting": int.from_bytes(data[17:19], byteorder='big')
        }
class BatteryController(Node):
    def __init__(self):
        super().__init__('battery_controller')
        self.declare_parameter('port', '/dev/ttyUSB_RS485')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('device_id', 0x00)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.battery = Battery(port, baudrate, device_id)
        self.subscription = self.create_subscription(
            String,
            'battery_cmd',
            self.command_callback,
            10)
    def command_callback(self, msg):
        command = msg.data.lower()
        if command == "status":
            status = self.battery.read_battery_status()
            if status:
                self.get_logger().info("Battery Status:")
                for key, value in status.items():
                    self.get_logger().info(f"{key}: {value}")
            else:
                self.get_logger().info("Failed to read battery status.")
        elif command == "set_emergency":
            response = self.battery.set_emergency(1)
            if response:
                self.get_logger().info("Emergency set.")
            else:
                self.get_logger().info("Failed to set emergency.")
        elif command == "clear_emergency":
            response = self.battery.set_emergency(0)
            if response:
                self.get_logger().info("Emergency cleared.")
            else:
                self.get_logger().info("Failed to clear emergency.")
        else:
            self.get_logger().info("Unknown command")
def main(args=None):
    rclpy.init(args=args)
    battery_controller = BatteryController()
    rclpy.spin(battery_controller)
    battery_controller.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
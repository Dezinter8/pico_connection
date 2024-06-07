import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point32
import serial
import threading
import time

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(Point32, 'imu_data', qos_profile)
        self.serial_device_path = '/dev/serial/by-id/usb-Raspberry_Pi_Pico_E660D4A0A772982F-if00'
        self.serial_port = None
        self.open_serial_port()

        thread = threading.Thread(target=self.read_and_publish)
        thread.daemon = True
        thread.start()

    def open_serial_port(self):
        while self.serial_port is None:
            try:
                self.serial_port = serial.Serial(self.serial_device_path, 115200, timeout=None)
                self.get_logger().info('Serial port opened successfully.')
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to open serial port: {str(e)}. Retrying...')
                time.sleep(2)  # Czekaj 2 sekundy przed ponowną próbą


    def read_and_publish(self):
        while not self.serial_port or not self.serial_port.is_open:
            self.open_serial_port()

        while rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line.startswith('ypr:'):
                        parts = line.replace('ypr:', '').strip().split(',')
                        point_msg = Point32()
                        point_msg.z = float(parts[0]) * -1
                        point_msg.y = float(parts[1]) * -1
                        point_msg.x = float(parts[2])
                        self.publisher_.publish(point_msg)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {str(e)}')
                self.reset_serial_connection()
            except Exception as e:
                self.get_logger().error(f'Failed to parse serial data: "{str(e)}"')
                if 'Input/output error' in str(e):
                    self.reset_serial_connection()

    def reset_serial_connection(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None
            self.get_logger().info('Serial port reset successfully.')
        self.open_serial_port()


    def destroy_node(self):
        super().destroy_node()
        if self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
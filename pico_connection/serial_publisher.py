import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point32
import serial
import time

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.publisher_ = self.create_publisher(Point32, 'imu_data', qos_profile)
        self.serial_port = None
        self.serial_device_path = '/dev/serial/by-id/usb-Raspberry_Pi_Pico_E660D4A0A772982F-if00'
        self.open_serial_port()
        self.timer = self.create_timer(0.05, self.read_and_publish)

    def open_serial_port(self):
        while self.serial_port is None:
            try:
                self.serial_port = serial.Serial(self.serial_device_path, 115200, timeout=None)
                self.get_logger().info('Serial port opened successfully.')
            except serial.SerialException as e:
                self.get_logger().warn('Failed to open serial port: %s. Retrying...' % str(e))
                time.sleep(2)  # Czekaj 2 sekundy przed ponowną próbą

    def read_and_publish(self):
        if self.serial_port is not None and self.serial_port.is_open:
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
                        # self.get_logger().info('Publishing: yaw=%f, pitch=%f, roll=%f' % (point_msg.z, point_msg.y, point_msg.x))
            except serial.SerialException as e:
                self.get_logger().error('Serial error: %s' % str(e))
                self.reset_serial_connection()
            except Exception as e:
                self.get_logger().error('Failed to parse serial data: "%s"' % str(e))
                if 'Input/output error' in str(e):
                    self.reset_serial_connection()
        else:
            self.get_logger().warn('Serial port not open. Attempting to reconnect...')
            self.open_serial_port()

    def reset_serial_connection(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None
        self.open_serial_port()


def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = SerialSubscriber()
    rclpy.spin(serial_subscriber)
    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

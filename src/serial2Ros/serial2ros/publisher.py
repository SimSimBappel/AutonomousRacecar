import rclpy
from std_msgs.msg import String
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)


def read_serial():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        return line
    return None


def main():
    rclpy.init()
    node = rclpy.create_node('external_input_publisher')

    publisher = node.create_publisher(String, 'external_input_topic', 10)

    while rclpy.ok():
        try:
            user_input = read_serial()
            time.sleep(0.01)  # Add a short delay to prevent busy-waiting
            if user_input is not None:
                msg = String()
                msg.data = user_input

                node.get_logger().info('Publishing: "%s"' % msg.data)
                publisher.publish(msg)
        except KeyboardInterrupt:
            ser.close()

        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




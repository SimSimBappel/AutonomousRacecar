from rclpy.node import Node
from jtop import jtop
import rclpy
import os


class PowerMonitor(Node):
    def __init__(self):
        super().__init__('Power_monitor_node')

        with jtop() as jetson:
            while jetson.ok():
                voltage = jetson.power['rail']['CPU']['volt']/1000
                if voltage < 10.0:
                    self.get_logger().warn(f"Voltage is too low: {voltage}V")
                    if voltage < 9.3:
                        os.system('sudo shutdown now')


def main(args=None):
    rclpy.init(args=args)

    power_monitor_node = PowerMonitor()

    rclpy.spin(power_monitor_node)

    power_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
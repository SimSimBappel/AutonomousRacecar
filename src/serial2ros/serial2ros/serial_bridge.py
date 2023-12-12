import rclpy
from geometry_msgs.msg import TransformStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf_transformations
import tf2_ros
import serial
import serial.tools.list_ports
import math
# from jtop import jtop


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odometry_publisher = self.create_publisher(Odometry, 'odometry', 10)
        
        
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        self.update_frequency = 100  # Hz
        self.update_timer = self.create_timer(1.0 / self.update_frequency, self.update_callback)

        port = self.list_serial_ports()

        self.ser = serial.Serial(port, 115200)

    def list_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]

        if len(port_list) == 1:
                return port_list[0]
        elif len(port_list) > 1:
            self.get_logger().warn("Too many serial pors")
            for i, port in enumerate(port_list):
                self.get_logger().info(f"Port{i}:{port}")
        else:
            self.get_logger().error("No serial ports found.")
            exit(1)


    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            return line
        return None

    def broadcast_transform(self):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = "track"
        transform_msg.child_frame_id = "base_link"

        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        transform_msg.transform.rotation.x = quaternion[0]
        transform_msg.transform.rotation.y = quaternion[1]
        transform_msg.transform.rotation.z = quaternion[2]
        transform_msg.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(transform_msg)

    def publish_odometry(self):
        odometry_msg = Odometry()
        odometry_msg.header.frame_id = 'track'
        odometry_msg.child_frame_id = 'base_link'

        odometry_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        odometry_msg.pose.pose.orientation.x = quaternion[0]
        odometry_msg.pose.pose.orientation.y = quaternion[1]
        odometry_msg.pose.pose.orientation.z = quaternion[2]
        odometry_msg.pose.pose.orientation.w = quaternion[3]
        odometry_msg.twist.twist.linear = Vector3(x=1.0, y=0.0, z=0.0)
        odometry_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.odometry_publisher.publish(odometry_msg)

    def update_pose(self, displacement, heading):
        self.theta += heading
        displacement = displacement / 1000  # From mm to m #has to be 1000
        self.x += displacement * math.cos(self.theta)
        self.y += displacement * math.sin(self.theta)

    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if angular_z > math.pi/4:
            angular_z = 110
        elif angular_z < -math.pi/4:
            angular_z = 20
        else:
            angular_z = self.map_value(angular_z, -math.pi/4, math.pi/4, 20, 110)

        command = f"{linear_x:.2f},{angular_z:.2f};\n"

        # with jtop() as jetson:
        #     if jetson.power['rail']['CPU']['volt']/1000 < 10.0:
        #         self.get_logger().error(f"Voltage is too low: {jetson.power['rail']['CPU']['volt']/1000}V")
        #         self.ser.write(f"{0.0},{0.0};\n")
        #     else:
        try:
            self.ser.write(command.encode())
        except Exception as e:
            self.get_logger().warn(e)
            pass


    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min

    def update_callback(self):
        try:
            serial_data = self.read_serial()
            if serial_data is not None :
                displacement, heading = map(float, serial_data.split(' '))
                self.update_pose(displacement, heading)
                self.publish_odometry()
                # self.broadcast_transform()
        except KeyboardInterrupt:
            self.ser.close()

def main():
    rclpy.init()
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


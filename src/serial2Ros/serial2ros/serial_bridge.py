import rclpy
from geometry_msgs.msg import TransformStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf_transformations
import tf2_ros
import serial
import math

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odometry_publisher = self.create_publisher(Odometry, 'odometry', 10)
        self.odometry_msg = Odometry()

        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        self.update_frequency = 100  # Hz
        self.update_timer = self.create_timer(1.0 / self.update_frequency, self.update_callback)

        self.ser = serial.Serial("/dev/ttyUSB0", 115200)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            return line
        return None

    def broadcast_transform(self):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = "map"
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
        self.odometry_msg.header.frame_id = 'map'
        self.odometry_msg.child_frame_id = 'base'

        self.odometry_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odometry_msg.pose.pose.orientation.x = quaternion[0]
        self.odometry_msg.pose.pose.orientation.y = quaternion[1]
        self.odometry_msg.pose.pose.orientation.z = quaternion[2]
        self.odometry_msg.pose.pose.orientation.w = quaternion[3]

        self.odometry_msg.twist.twist.linear = Vector3(x=1.0, y=0.0, z=0.0)
        self.odometry_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.odometry_publisher.publish(self.odometry_msg)

    def update_pose(self, displacement, heading):
        self.theta += heading
        displacement = displacement / 1000  # From mm to m
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

        self.ser.write(command.encode())

    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min

    def update_callback(self):
        try:
            serial_data = self.read_serial()
            if serial_data is not None :
                displacement, heading = map(float, serial_data.split(' '))
                self.update_pose(displacement, heading)
                self.publish_odometry()
                self.broadcast_transform()
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


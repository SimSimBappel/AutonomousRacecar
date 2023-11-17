import rclpy
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import serial
import math

# Constants
SERIAL_PORT = "/dev/ttyUSB0"  # Update with your Arduino serial port
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

def read_serial():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        return line
    return None

def broadcast_transform(x, y, theta, tf_broadcaster):
    # Create a TransformStamped message
    transform_msg = TransformStamped()
    transform_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    transform_msg.header.frame_id = "odom"
    transform_msg.child_frame_id = "base_link"

    # Set translation based on calculated position
    transform_msg.transform.translation.x = x
    transform_msg.transform.translation.y = y
    transform_msg.transform.translation.z = 0.0

    # Set rotation based on calculated orientation
    quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, theta)
    transform_msg.transform.rotation.x = quaternion[0]
    transform_msg.transform.rotation.y = quaternion[1]
    transform_msg.transform.rotation.z = quaternion[2]
    transform_msg.transform.rotation.w = quaternion[3]

    # Broadcast the transform
    tf_broadcaster.sendTransform(transform_msg)

def update_pose(x, y, theta, displacement, heading):
    # Update the robot's pose based on odometry data
    theta += heading
    x += displacement * math.cos(theta)
    y += displacement * math.sin(theta)

    return x, y, theta

def main():
    rclpy.init()

    # Create a TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster(rclpy.create_node("odometry_listener"))

    # Initial pose of the robot
    x, y, theta = 0.0, 0.0, 0.0

    while rclpy.ok():
        # Read data from the serial port (adjust parsing logic based on your format)
        try:
            serial_data = read_serial()
            if serial_data is not None:
                displacement, heading = map(float, serial_data.split(' '))

                # Update the robot's pose
                x, y, theta = update_pose(x, y, theta, displacement, heading)

                # Broadcast the transform with the updated pose
                broadcast_transform(x, y, theta, tf_broadcaster)

        except KeyboardInterrupt:
            ser.close()

    # Close the serial port when exiting
    ser.close()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

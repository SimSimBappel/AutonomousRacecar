import rclpy
import serial
import time
from sensor_msgs.msg import Imu
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf_transformations import quaternion_about_axis

ser = serial.Serial('/dev/ttyUSB0', 115200)

IMU_FRAME = 'IMU'
data = ''

type = ['IMU', 'ENC']

def read_serial():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        return line
    return None




def main():
    rclpy.init()
    node = rclpy.create_node('IMU_publisher')

    publisher = node.create_publisher(Imu, 'IMU_publisher', 10)

    while rclpy.ok():
        try:
            user_input = read_serial()
            time.sleep(0.01)  # Add a short delay to prevent busy-waiting
            if user_input is not None:
                data = user_input.split(",")
                #print(type.index(data[0]))

                if type.index(data[0]) == 0:
                    imu_msg = Imu()
                    imu_msg.header.frame_id = IMU_FRAME
                    #convert data to int, and convert unit to g
                    acc = [int(x) / 16384 for x in data[1:4]]
                    #convert g to m/s²
                    acc[:] = [x * 9.8 for x in acc]
                    

                    imu_msg.linear_acceleration.x = acc[0]
                    imu_msg.linear_acceleration.y = acc[1]
                    imu_msg.linear_acceleration.z = acc[2]


                    gy = [int(x) / 131 for x in data[4:]]

                    imu_msg.angular_velocity.x = gy[0]*0.0174
                    imu_msg.angular_velocity.y = gy[1]*0.0174
                    imu_msg.angular_velocity.z = gy[2]*0.0174

                    imu_msg.header.stamp = node.get_clock().now().to_msg()

                    publisher.publish(imu_msg)


                    accel = acc[0], acc[1], acc[2]
                    ref = np.array([0, 0, 1])
                    acceln = accel / np.linalg.norm(accel)
                    axis = np.cross(acceln, ref)
                    angle = np.arccos(np.dot(acceln, ref))
                    orientation = quaternion_about_axis(angle, axis)
                    br = tf2_ros.TransformBroadcaster(node)
                    t = TransformStamped()

                    t.header.stamp = node.get_clock().now().to_msg()
                    t.header.frame_id = "plane"
                    t.child_frame_id = "imu_link"
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = orientation[0]
                    t.transform.rotation.y = orientation[1]
                    t.transform.rotation.z = orientation[2]
                    t.transform.rotation.w = orientation[3]

                    br.sendTransform(t)
                    
        except KeyboardInterrupt:
            ser.close()

    # node.destroy_node()
    # rclpy.shutdown()




if __name__ == '__main__':
    main()



# // #!/usr/bin/env python  
# // import rospy
# // import tf_conversions
# // import tf2_ros
# // import geometry_msgs.msg
# // from sensor_msgs.msg import Temperature, Imu


# // def handle_imu_pose(msg):
# //     br = tf2_ros.TransformBroadcaster()
# //     t = geometry_msgs.msg.TransformStamped()

# //     t.header.stamp = rospy.Time.now()
# //     t.header.frame_id = "plane"
# //     t.child_frame_id = "imu_link"
# //     t.transform.translation.x = 0
# //     t.transform.translation.y = 0
# //     t.transform.translation.z = 0
# //     t.transform.rotation.x = msg.orientation.x
# //     t.transform.rotation.y = msg.orientation.y
# //     t.transform.rotation.z = msg.orientation.z
# //     t.transform.rotation.w = msg.orientation.w

# //     br.sendTransform(t)

# // if __name__ == '__main__':
# //       rospy.init_node('tf_broadcaster_imu')
# //       rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
# //       rospy.spin()

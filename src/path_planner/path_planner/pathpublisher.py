#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
from fsd_path_planning.utils.utils import Timer
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
import numpy as np
from nav_msgs.msg import Odometry
from math import pi, atan2
import time
from custom_sys_msgs.msg import Cone, ConeDetectionStamped


class MarkerArraySubscriber(Node):
    cone_observations = [[
    np.array([], dtype=np.float64).reshape(0, 2),
    np.array([], dtype=np.float64).reshape(0, 2), # Bluecones
    np.array([], dtype=np.float64).reshape(0, 2), # Yellowcones
    np.array([], dtype=np.float64).reshape(0, 2),
    np.array([])
    ]]
    blue_recieved = False#
    yellow_recieved = False#
    cones_recieved = False
    odometry_recieved = False


    def __init__(self):
        super().__init__('marker_array_subscriber')
        self.path_planner = PathPlanner(MissionTypes.trackdrive)

        # self.odometry_subscription = self.create_subscription(
        #     Odometry, 
        #     'odometry', 
        #     self.odometry_callback, 
        #     1
        # )
        # self.odometry_subscription 

        self.pose_subscription = self.create_subscription(
            PoseStamped, 
            'robot_pose', 
            self.pose_callback, 
            1
        )
        self.pose_subscription 

        self.pathPublisher = self.create_publisher(PoseArray, 'path', 10)
        self.pose_array = PoseArray()

        self.twistPublisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()

        # self.bluesubscription = self.create_subscription(#
        #     MarkerArray,
        #     'blue_cones',
        #     self.blue_callback,
        #     1
        # )
        # self.bluesubscription  

        # self.yellowsubscription = self.create_subscription(#
        #     MarkerArray,
        #     'yellow_cones',
        #     self.yellow_callback,
        #     1
        # )
        # self.yellowsubscription 

        self.cones_subscription = self.create_subscription(
            ConeDetectionStamped,
            "cone_detections",
            self.cones_callback,
            1
        )


    def reset(self):
    #     # self.cone_observations = [[
    #     #     np.array([], dtype=np.float64).reshape(0, 2),
    #     #     np.array([], dtype=np.float64).reshape(0, 2),#bluecones
    #     #     np.array([], dtype=np.float64).reshape(0, 2),#yellowcones
    #     #     np.array([], dtype=np.float64).reshape(0, 2),
    #     #     np.array([])
    #     # ]]
    #     # self.blue_recieved = False
    #     # self.yellow_recieved = False
        self.cones_recieved = False
        self.odometry_recieved = False
        

    # def odometry_callback(self, msg):
    #     self.car_position = np.array([[
    #                 msg.pose.pose.position.x,
    #                 msg.pose.pose.position.y
    #     ]])

    #     cos_theta = 1 - 2 * (msg.pose.pose.orientation.z**2)
    #     sin_theta = 2 * (msg.pose.pose.orientation.z * msg.pose.pose.orientation.w)
    #     self.car_direction = np.array([[
    #                 cos_theta,
    #                 sin_theta
    #     ]])
    #     self.odometry_recieved = True

    def pose_callback(self, msg):
        self.car_position = np.array([[
                    msg.pose.position.x,
                    msg.pose.position.y
        ]])

        cos_theta = 1 - 2 * (msg.pose.orientation.z**2)
        sin_theta = 2 * (msg.pose.orientation.z * msg.pose.orientation.w)
        self.car_direction = np.array([[
                    cos_theta,
                    sin_theta
        ]])
        self.odometry_recieved = True


    # def blue_callback(self, msg):#
    #     self.get_logger().info('Received MarkerArray message')
    #     self.cone_observations[0][1] = np.array([]).reshape(0, 2)

    #     for marker in msg.markers:
    #         # Extract x and y coordinates from the marker pose
    #         x = marker.pose.position.x
    #         y = marker.pose.position.y
    #         self.cone_observations[0][1] = np.vstack((self.cone_observations[0][1], np.array([x, y])))
    #     self.blue_recieved = True

    
    # def yellow_callback(self, msg):#
    #     self.get_logger().info('Received MarkerArray message')
    #     self.cone_observations[0][2] = np.array([]).reshape(0, 2)
    #     for marker in msg.markers:
    #         # Extract x and y coordinates from the marker pose
    #         x = marker.pose.position.x
    #         y = marker.pose.position.y
    #         self.cone_observations[0][2] = np.vstack((self.cone_observations[0][2], np.array([x, y])))
    #     self.yellow_recieved = True


    def cones_callback(self, msg):
        self.cone_observations[0][1] = np.array([]).reshape(0, 2)
        self.cone_observations[0][2] = np.array([]).reshape(0, 2)
        # self.get_logger().info("Recieved cone array")
        for cone in msg.cones_with_cov:
            x = cone.cone.location.x
            y = cone.cone.location.y

            if cone.cone.color == 1:
                self.cone_observations[0][1] = np.vstack((self.cone_observations[0][1], np.array([x, y])))
            elif cone.cone.color == 0:
                self.cone_observations[0][2] = np.vstack((self.cone_observations[0][2], np.array([x, y])))
        self.cones_recieved = True


    def path_publisher(self, x, y):
        self.pose_array.poses = []

        for i in range(len(x) - 1):
            x1, y1 = x[i], y[i]
            x2, y2 = x[i + 1], y[i + 1]

            # Calculate heading using np.arctan2
            angle = np.arctan2(y2 - y1, x2 - x1)
            # Create PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x[i]
            pose_stamped.pose.position.y = y[i]
            pose_stamped.pose.orientation.z = np.sin(angle / 2) 
            pose_stamped.pose.orientation.w = np.cos(angle / 2)

            if i == 0:
                z = angle
            self.pose_array.poses.append(pose_stamped.pose)

        self.pose_array.header.frame_id = 'track'
        self.pathPublisher.publish(self.pose_array)

        return z


    def twist_publisher(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        self.twistPublisher.publish(twist_msg)


    def calculate_heading(self, x, y, pathdir, position, direction):
        angle = np.arctan2(y[0] - position[1], x[0] - position[0]) # Angle from car pos to path pos
        car_orientation = atan2(direction[1], direction[0])
        diff = pathdir - car_orientation # Difference of car heading and path heading
        angle = angle - car_orientation # Angle from car to path pos

        if diff < -pi:
            diff += 2*pi

        heading = (diff + angle) / 2

        return heading


    def plan_path(self):
        print(f"self.cones_recieved: {self.cones_recieved}, self.odometry_recieved: {self.odometry_recieved}")
        if  self.cones_recieved and self.odometry_recieved: #self.blue_recieved and self.yellow_recieved and
            self.get_logger().info("calculating path")
            cones = self.cone_observations[0]
            position = self.car_position[0]
            direction = self.car_direction[0]
            timer = Timer(noprint=True)
            try:
                with timer:
                    out = self.path_planner.calculate_path_in_global_frame(
                        cones,
                        position,
                        direction,
                        return_intermediate_results=False,
                    )
            except Exception as e:
                self.get_logger().warn(f"Error at frame {e}")
                raise

            if timer.intervals[-1] > 0.1:
                        self.get_logger().info(f"Frame took {timer.intervals[-1]:.4f} seconds")

            lookahead = 3
            x = out[lookahead:, 1]
            y = out[lookahead:, 2] 

            pathdir = self.path_publisher(x,y)
            heading = self.calculate_heading(x, y, pathdir, position, direction)       
            self.twist_publisher(0.5, heading)
            # self.reset()


def main(args=None):
    rclpy.init(args=args)
    marker_array_subscriber = MarkerArraySubscriber()

    lasttime = 0.0
    while rclpy.ok():
        rclpy.spin_once(marker_array_subscriber, timeout_sec=0.1)

        if lasttime + 0.1 < time.perf_counter():
            lasttime = time.perf_counter()
            marker_array_subscriber.plan_path()
            
    marker_array_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

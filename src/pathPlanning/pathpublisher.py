#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
from fsd_path_planning.utils.utils import Timer
from geometry_msgs.msg import PoseStamped, PoseArray
import numpy as np
from nav_msgs.msg import Odometry
from math import pi

plot = False

class MarkerArraySubscriber(Node):
    cone_observations = [[
    np.array([], dtype=np.float64).reshape(0, 2),
    np.array([], dtype=np.float64).reshape(0, 2),#bluecones
    np.array([], dtype=np.float64).reshape(0, 2),#yellowcones
    np.array([], dtype=np.float64).reshape(0, 2),
    np.array([[1.87387085, 6.18708801],
              [-1.18862915, 6.37458801],
              [1.74887085, 5.62458801],
              [-1.25112915, 5.93708801]])
    ]]
    blue_recieved = False
    yellow_recieved = False


    def __init__(self):
        super().__init__('marker_array_subscriber')
        self.path_planner = PathPlanner(MissionTypes.trackdrive)

        self.odometry_subscription = self.create_subscription(
            Odometry, 
            'odometry', 
            self.odometry_callback, 
            1
        )
        self.odometry_subscription # prevent unused variable warning

        self.pathPublisher = self.create_publisher(PoseArray, 'path', 10)
        self.pose_array = PoseArray()

        self.bluesubscription = self.create_subscription(
            MarkerArray,
            'blue_cones',
            self.blue_callback,
            1
        )
        self.bluesubscription  

        self.yellowsubscription = self.create_subscription(
            MarkerArray,
            'yellow_cones',
            self.yellow_callback,
            1
        )
        self.yellowsubscription 

    def reset(self):
        # self.cone_observations = [[
        #     np.array([], dtype=np.float64).reshape(0, 2),
        #     np.array([], dtype=np.float64).reshape(0, 2),#bluecones
        #     np.array([], dtype=np.float64).reshape(0, 2),#yellowcones
        #     np.array([], dtype=np.float64).reshape(0, 2),
        #     np.array([])
        # ]]
        # self.blue_recieved = False
        # self.yellow_recieved = False
        l=1
        

    def odometry_callback(self, msg):
        self.car_position = np.array([[
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y
        ]])

        cos_theta = 1 - 2 * (msg.pose.pose.orientation.z**2)
        sin_theta = 2 * (msg.pose.pose.orientation.z * msg.pose.pose.orientation.w)
        self.car_direction = np.array([[
                    cos_theta,
                    sin_theta
        ]])
        self.plan_path()


    def blue_callback(self, msg):
        self.get_logger().info('Received MarkerArray message')

        for i, marker in enumerate(msg.markers):
            # Extract x and y coordinates from the marker pose
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.cone_observations[0][1] = np.vstack((self.cone_observations[0][1], np.array([x, y])))
        self.blue_recieved = True
        # self.plan_path()

    
    def yellow_callback(self, msg):
        self.get_logger().info('Received MarkerArray message')
        
        for marker in msg.markers:
            # Extract x and y coordinates from the marker pose
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.cone_observations[0][2] = np.vstack((self.cone_observations[0][2], np.array([x, y])))
        self.yellow_recieved = True
        # self.plan_path()


    def path_publisher(self, x, y):
        self.pose_array.poses = []

        for i in range(len(x) - 1):
            x1, y1 = x[i], y[i]
            x2, y2 = x[i + 1], y[i + 1]

            # Calculate heading using np.arctan2
            angle = np.arctan2(y2 - y1, x2 - x1)
            # Create PoseStamped message
            pose_stamped = PoseStamped() #cant just put in new reference frame needs to be calculated
            pose_stamped.pose.position.x = x[i]
            pose_stamped.pose.position.y = y[i]
            pose_stamped.pose.orientation.z = np.sin(angle / 2) 
            pose_stamped.pose.orientation.w = np.cos(angle / 2)

            self.pose_array.poses.append(pose_stamped.pose)

        self.pose_array.header.frame_id = 'map'
        # Publish the PoseArray
        self.pathPublisher.publish(self.pose_array)
        # self.get_logger().info('Path published')


    def plan_path(self):
        if self.blue_recieved and self.yellow_recieved:
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
                print(f"Error at frame {e}")
                raise
                # results.append(out)

            if timer.intervals[-1] > 0.1:
                        print(f"Frame took {timer.intervals[-1]:.4f} seconds")

            x = out[3:, 1]
            y = out[3:, 2] 


            self.path_publisher(x,y)

            if plot:
                bluex_cones = cones[1][:, 0]
                bluey_cones = cones[1][:, 1] 

                yellowx_cones = cones[2][:, 0]  
                yellowy_cones = cones[2][:, 1] 

                # fig, ax = plt.subplots(figsize=(10, 8))
                # fig.clf()
                plt.clf()
                plt.scatter(bluex_cones, bluey_cones, c='b', marker='o', label='Cones')
                plt.scatter(yellowx_cones, yellowy_cones, c='y', marker='o', label='Cones')
                plt.scatter(x, y, c='g', label='Path')
                plt.xlabel('X-axis')
                plt.ylabel('Y-axis')
                plt.title('Path in Global Frame')
                plt.legend()
                # ax.set_aspect('equal')
                plt.grid(True)
                plt.pause(0.1)
            self.reset()


def main(args=None):
    rclpy.init(args=args)
    # path_planner = PathPlanner(MissionTypes.trackdrive)
    marker_array_subscriber = MarkerArraySubscriber()
    # MarkerArraySubscriber.reset(marker_array_subscriber)

    while rclpy.ok():
        rclpy.spin_once(marker_array_subscriber, timeout_sec=0.1)

    marker_array_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
import time
import json
from pathlib import Path
import numpy as np
from math import atan2, sin, cos, pi
from nav_msgs.msg import Odometry
from ament_index_python import get_package_share_directory

include_odom = False

conecolor={
    "blue": 1,
    "yellow": 2,
}


class data_simulation(Node):
    def __init__(self):
        super().__init__('data_simulation')

        self.cone_observations_json, self.car_position, self.car_direction = self.load_data() 
        print(self.car_direction[0])
       
        self.blueCones = self.create_publisher(MarkerArray, 'blue_cones', 10)
        self.yellowCones = self.create_publisher(MarkerArray, 'yellow_cones', 10)

        self.publish_marker_array()

        self.odometry_publisher = self.create_publisher(Odometry, 'odometry', 10)
        self.odometry_msg = Odometry()

        if include_odom:
            self.i = 0
            while rclpy.ok():
                self.pub_odom()
                time.sleep(0.1)
                
                if self.i >= 439:
                    self.i = 0
                else:
                    self.i +=1



    def load_data(self):   
        package_path = get_package_share_directory('path_planner')
        data_path = Path(package_path).parents[3] / "resources" / "fsg_19_2_laps.json"
        data = json.loads(data_path.read_text())[:]

        data_path = Path(package_path).parents[3] / "resources" / "test.json" 
        data2 = json.loads(data_path.read_text())[:]

        positions = np.array([d["car_position"] for d in data])
        directions = np.array([d["car_direction"] for d in data])
        cone_observations = [
            [np.array(c).reshape(-1, 2) for c in d["slam_cones"]] for d in data2
        ]

        return cone_observations, positions, directions
        

    def create_marker(self, color, x_coord, y_coord, i):
        marker = Marker()
        marker.header = Header()
        marker.id = i
        marker.header.frame_id = "map"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = Pose(position=Point(x=x_coord, y=y_coord, z=0.5), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
        if color == conecolor["blue"]:
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        else:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        return marker


    def publish_marker_array(self):
        # while rclpy.ok():
            # Create a MarkerArray message
        blueArray = MarkerArray()

        # Create a Marker message
        for i, (x,y) in enumerate(self.cone_observations_json[0][1]):
            blueArray.markers.append(self.create_marker(1,x,y,i))

        # Update timestamps for the MarkerArray and each Marker
        for marker in blueArray.markers:
            marker.header.stamp = self.get_clock().now().to_msg()

        # Publish the MarkerArray
        self.blueCones.publish(blueArray)

        yellowArray = MarkerArray()

        # Create a Marker message
        for i, (x,y) in enumerate(self.cone_observations_json[0][2]):
            yellowArray.markers.append(self.create_marker(2,x,y,i))

        # Update timestamps for the MarkerArray and each Marker
        for marker in yellowArray.markers:
            marker.header.stamp = self.get_clock().now().to_msg()

        # Publish the MarkerArray
        self.yellowCones.publish(yellowArray)

    def pub_odom(self):
        print(f"Iteration: {self.i}")
        self.odometry_msg.header.frame_id = 'map'
        self.odometry_msg.child_frame_id = 'base'

        # Set the pose information 
        self.odometry_msg.pose.pose.position = Point(x=self.car_position[self.i][0], y=self.car_position[self.i][1], z=0.0)
        
        orientation = atan2(self.car_direction[self.i][1], self.car_direction[self.i][0]) 
        self.odometry_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sin(orientation/2), w=cos(orientation/2))

        # Set the twist information
        self.odometry_msg.twist.twist.linear = Vector3(x=1.0, y=0.0, z=0.0)
        self.odometry_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        # Set the timestamp
        self.odometry_msg.header.stamp = self.get_clock().now().to_msg()

        self.odometry_publisher.publish(self.odometry_msg)



def main(args=None):
    rclpy.init(args=args)
    datasim = data_simulation()

    

    while rclpy.ok():
        rclpy.spin_once(datasim, timeout_sec=0.1)

        time.sleep(1)

    datasim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

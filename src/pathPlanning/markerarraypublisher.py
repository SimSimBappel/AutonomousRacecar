#!/usr/bin/env python

import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
import time
import json
from pathlib import Path
import numpy as np

conecolor={
    "blue": 1,
    "yellow": 2,
}


def create_marker(color, x_coord, y_coord, i):
    marker = Marker()
    marker.header = Header()
    marker.id = i
    marker.header.frame_id = "map"
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose = Pose(position=Point(x=x_coord, y=y_coord, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
    if color == conecolor["blue"]:
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
    else:
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

    return marker


def load_data():   
    data_path = Path(__file__).parent / "test.json"
        # data_path = Path(__file__).parent / "fss_19_4_laps.json"
        #format yellow,blue,unknown
        # Must give all global cones x and y

        
    # extract data
    data = json.loads(data_path.read_text())[:]

    positions = np.array([d["car_position"] for d in data])
    directions = np.array([d["car_direction"] for d in data])
    cone_observations = [
        [np.array(c).reshape(-1, 2) for c in d["slam_cones"]] for d in data
    ]
    # print(cone_observations)

    # print(directions)

    return cone_observations, positions, directions
    

def publish_marker_array():
    # Initialize the ROS 2 node
    rclpy.init()
    node = rclpy.create_node('marker_array_publisher')

    # Create a publisher for the MarkerArray topic
    blueCones = node.create_publisher(MarkerArray, 'blue_cones', 10)
    yellowCones = node.create_publisher(MarkerArray, 'yellow_cones', 10)

    # Set the loop rate
    rate = node.create_rate(1)  # 1 Hz

    cone_observations_json, car_position, car_direction = load_data() 


    # while rclpy.ok():
        # Create a MarkerArray message
    blueArray = MarkerArray()

    # Create a Marker message
    # for i in enumerate(zip())
    for i, (x,y) in enumerate(cone_observations_json[0][1]):
        # print(x,y)
        blueArray.markers.append(create_marker(1,x,y,i))

    # Update timestamps for the MarkerArray and each Marker
    # marker_array.header.stamp = node.get_clock().now().to_msg()
    for marker in blueArray.markers:
        marker.header.stamp = node.get_clock().now().to_msg()

    # Publish the MarkerArray
    blueCones.publish(blueArray)



    yellowArray = MarkerArray()

    # Create a Marker message
    # for i in enumerate(zip())
    for i, (x,y) in enumerate(cone_observations_json[0][2]):
        # print(x,y)
        yellowArray.markers.append(create_marker(2,x,y,i))

    # Update timestamps for the MarkerArray and each Marker
    # marker_array.header.stamp = node.get_clock().now().to_msg()
    for marker in yellowArray.markers:
        marker.header.stamp = node.get_clock().now().to_msg()

    # Publish the MarkerArray
    yellowCones.publish(yellowArray)


        # Sleep to maintain the loop rate
        # rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        publish_marker_array()
    except KeyboardInterrupt:
        pass

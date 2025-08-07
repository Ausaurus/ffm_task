#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import String
from map_search.srv import *

def load_waypoints(room_no,
                   filename="/home/i_h8_ros/ffm_ws/src/map_search/waypoints.txt"):
    # Define the structured data type for our waypoints
    dt = np.dtype([('x', float), ('y', float), ('z', float)])

    # Read all lines from the file
    with open(filename, 'r') as f:
        lines = f.readlines()

    # Process each line into a tuple of floats
    data = []
    parts = lines[room_no].strip().split()
    if len(parts) >= 3:
        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2])
        data.append((x, y, z))
    # Create numpy array with the specified dtype
    return np.array(data, dtype=dt)

def send_waypoints(client, waypoint):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose = waypoint

    rospy.loginfo(f"Sending waypoint {waypoint}")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(40.0))  # Wait for a max of 10 seconds
    rospy.loginfo(f"Waypoint {waypoint} reached or timeout reached")

def callback(req):
    waypoint = load_waypoints(room_no=req.room)
    point = Point(waypoint['x'], waypoint['y'], waypoint['z'])
    goal = Pose(position=point, orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
    send_waypoints(client=client, waypoint=goal)
    return waypointResponse(True)

if __name__ == "__main__":
    # Load waypoints from file
    rospy.init_node("read_waypoints")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    s = rospy.Service('go_to_waypoint', waypoint, callback)
    rospy.spin()

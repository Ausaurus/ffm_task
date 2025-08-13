#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from map_search.srv import *
from config import DIR
from std_srvs.srv import SetBool

class WaypointExecutor:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        self.current_goal = None
        self.retry_count = 0
        self.max_retries = 3  # Maximum retry attempts

    def send_waypoint(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint

        self.retry_count = 0
        success = False

        while not rospy.is_shutdown() and self.retry_count < self.max_retries:
            rospy.loginfo(f"Sending waypoint (attempt {self.retry_count+1}/{self.max_retries}): {waypoint.position}")
            self.client.send_goal(goal)
            finished = self.client.wait_for_result(rospy.Duration.from_sec(30.0))

            if finished:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Waypoint reached successfully!")
                    success = True
                    break
                else:
                    status_text = GoalStatus.to_string(state)
                    rospy.logwarn(f"Goal failed with status: {status_text}")
            else:
                rospy.logwarn("Action server timed out")

            self.retry_count += 1
            rospy.sleep(1.0)  # Brief pause before retry

        return success

def load_waypoints(room_no, filename=f"/home/{DIR}/src/ffm_task/waypoints.txt"):
    # Define the structured data type for our waypoints
    dt = np.dtype([('x', float), ('y', float), ('z', float)])

    # Read all lines from the file
    with open(filename, 'r') as f:
        lines = f.readlines()

    # Process the line for the given room number
    data = []
    parts = lines[room_no].strip().split()
    if len(parts) >= 3:
        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2])
        data.append((x, y, z))
    # Create numpy array with the specified dtype
    return np.array(data, dtype=dt)[0]  # Return first element

def callback(req, executor):
    waypoint_data = load_waypoints(room_no=req.room)
    point = Point(waypoint_data['x'], waypoint_data['y'], waypoint_data['z'])
    goal_pose = Pose(position=point, orientation=Quaternion(0.0, 0.0, 0.0, 1.0))

    success = executor.send_waypoint(goal_pose)

    # Notify way_done service about completion status
    try:
        way_done_proxy = rospy.ServiceProxy('way_done', SetBool)
        way_done_proxy(success)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to way_done failed: {e}")

    return waypointResponse(success)

if __name__ == "__main__":
    rospy.init_node("read_waypoints")
    executor = WaypointExecutor()
    s = rospy.Service('go_to_waypoint', waypoint, lambda req: callback(req, executor))
    rospy.spin()

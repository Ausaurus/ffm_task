#!/usr/bin/env python3
import rospy
import subprocess
import os
import json
import actionlib
from map_search.srv import *
from std_srvs.srv import *
from config import DIR
from topic_tools.srv import MuxSelect
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus

class start_scripts:
    def __init__(self):
        rospy.init_node("start_scripts")
        rospy.loginfo("waiting for service")
        rospy.wait_for_service("go_to_waypoint")
        self.go = rospy.ServiceProxy('go_to_waypoint', waypoint)
        self.service = rospy.Service('next_way', SetBool, self.rotation_done)
        self.way_done = rospy.Service('way_done', SetBool, self.waypoint_done)
        rospy.loginfo("start servicee")
        self.next_waypoint = True
        self.arrive = False
        self.rotate = f"/home/{DIR}/src/ffm_task/scripts/resumable_rotation_fz.py"
        self.room_number = 0
        self.customers_file = "/home/i_h8_ros/ffm_ws/src/ffm_task/scripts/customers_database.json"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def load_customers_database(self):
        """Load customer database from JSON file"""
        if os.path.exists(self.customers_file):
            try:
                with open(self.customers_file, 'r') as f:
                    data = json.load(f)
                rospy.loginfo(f"📂 Loaded {len(data)} customers from database")
                return len(data)
            except Exception as e:
                rospy.logerr(f"❌ Failed to load customers database: {e}")
        return 0

    def safe_terminate(self, process):
        if process and process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                rospy.logwarn("Process didn't terminate gracefully, killing")
                process.kill()
                process.wait()

    def run_script(self, path):
        exe = "/usr/bin/python3"
        self.process = subprocess.Popen([exe, path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def switch_mux(self, target):
        rospy.loginfo("wait for vel_mux")
        rospy.wait_for_service('/vel_mux/select')
        try:
            select = rospy.ServiceProxy('/vel_mux/select', MuxSelect)
            select(target)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to switch mux: {e}")


    def waypoint_done(self, req):
        self.arrive = req.data
        return SetBoolResponse(success=True, message="waypoint status updated")

    def rotation_done(self, req):
        rospy.loginfo("done rotation")
        self.safe_terminate(self.process)
        rospy.loginfo("killed resumable_rotation_fz")
        customer_num = self.load_customers_database()
        self.next_waypoint = req.data
        if customer_num >= 3:
            rospy.loginfo("went back to origin")
            self.next_waypoint = False
            self.go_to_origin()
        if self.next_waypoint:
            if self.room_number > 1:
                self.go_to_origin()
            self.switch_mux('move_base_cmd')
            rospy.loginfo("done change to move_base_cmd")
            reached = self.go(self.room_number)
            self.room_number = self.room_number + 1
            self.next_waypoint = False
            self.arrive = reached.success
        # self.process.terminate()
        return SetBoolResponse(success=True, message="rotation status updated")

    def go_to_origin(self):
        self.switch_mux('move_base_cmd')
        point = Point(0, 0, 0)
        goal = Pose(position=point, orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        self.send_waypoints(client=self.client, waypoint=goal)
        rospy.loginfo("run report.py")
        path = "/home/i_h8_ros/ffm_ws/src/report/script/report.py"
        subprocess.run(['python3', path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        rospy.signal_shutdown("done report")

    def send_waypoints(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint

        self.retry_count = 0
        self.max_retries = 3
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

    def run(self):
        self.switch_mux('move_base_cmd')
        rospy.loginfo("done change to move_base_cmd")
        reached = self.go(self.room_number)
        self.room_number =+ 1
        self.next_waypoint = False
        self.arrive = reached.success
        while not rospy.is_shutdown():
            if self.arrive:
                self.run_script(self.rotate)
                self.arrive = False

if __name__ == "__main__":
    start = start_scripts()
    start.run()

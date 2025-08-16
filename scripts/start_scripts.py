#!/usr/bin/env python3
import rospy
import subprocess
import os
import json
import actionlib
import threading
from map_search.srv import *
from std_srvs.srv import *
from std_msgs.msg import Int32, String, Bool
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
        self.pub_report = rospy.Publisher('report', Bool, queue_size=10)
        self.report_done = rospy.Subscriber('rep_done', Bool, self.report_state)
        self.spin_pub = rospy.Publisher('/rotator/control', String, queue_size=10)
        self.spin_sub = rospy.Subscriber('/rotator/status', String,
                                         self.spin_status)
        self.name_pub = rospy.Publisher('/name_what', String, queue_size=10)
        self.feature_sub = rospy.Subscriber('/feature_done', Bool,
                                            self.got_feature)
        self.next_waypoint = True
        self.customer_number = 1
        self.skip = 0
        self.arrive = False
        self.rot_done = False
        self.rep_done = True
        self.centered = False
        self.feature = False
        self.rotate = "/home/i_h8_ros/ffm_ws/src/nav/scripts/spin.py"
        self.room_number = 0
        self.customers_file = "/home/i_h8_ros/ffm_ws/src/ffm_task/scripts/customers_database.json"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.rate = rospy.Rate(10)

    def got_feature(self, msg):
        if msg.data:
            rospy.loginfo("got feature")
            self.feature = True

    def spin_status(self, msg):
        if msg.data == "person_detected":
            rospy.loginfo("got centered")
            self.centered = True

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
        rospy.loginfo(f"service way_done: {self.arrive}")
        return SetBoolResponse(success=True, message="waypoint status updated")

    def rotation_done(self, req):
        rospy.loginfo("done rotation")
        self.safe_terminate(self.process)
        rospy.loginfo("killed resumable_rotation_fz")
        self.rot_done = True
        return SetBoolResponse(success=True, message="rotation status updated")

    def go_to_origin(self):
        self.switch_mux('move_base_cmd')
        point = Point(0, 0, 0)
        goal = Pose(position=point, orientation=Quaternion(0, 0, 0.8939967, -0.4480736))
        self.send_waypoints(waypoint=goal)
        self.rep_done = False
        self.pub_report.publish(True)

    def go_to_livingroom(self):
        self.switch_mux('move_base_cmd')
        reached = self.go(2)

    def report_state(self, msg):
        self.rep_done = msg

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
        self.switch_mux("move_base_cmd")
        while self.customer_number <= 3:
            skipped = 0

            rospy.loginfo("waiting for report finish")
            while not self.rep_done:
                self.rate.sleep()

            rospy.loginfo("going to living room")
            self.arrive = self.go(0)

            if self.arrive:
                self.switch_mux("resume_cmd")
                rospy.loginfo("start spinning")
                self.spin_pub.publish("start")

            if self.skip == 1:
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

                rospy.loginfo("skip current customer")
                self.centered = False
                self.spin_pub.publish("continue")
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

            elif self.skip == 2:
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

                rospy.loginfo("skip current customer")
                self.centered = False
                self.spin_pub.publish("continue")
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

                rospy.loginfo("skip current customer")
                self.centered = False
                self.spin_pub.publish("continue")
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

            elif self.skip == 3:
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

                rospy.loginfo("skip current customer")
                self.centered = False
                self.spin_pub.publish("continue")
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

                rospy.loginfo("skip current customer")
                self.centered = False
                self.spin_pub.publish("continue")
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

                rospy.loginfo("skip current customer")
                self.centered = False
                self.spin_pub.publish("continue")
                rospy.loginfo("waiting for person to be centered")
                while not self.centered:
                    self.rate.sleep()

            else:
                rospy.loginfo("waiting for person to be ceneterd")
                while not self.centered:
                    self.rate.sleep()

            rospy.loginfo("run feature scan")
            self.name_pub.publish("What's ya name")

            rospy.loginfo("waiting for feature scan to finish")
            while not self.feature:
                self.rate.sleep()

            self.skip += 1
            self.centered = False
            self.feature = False

            rospy.loginfo("going back to origin")
            self.go_to_origin()
            self.customer_number += 1

        rospy.signal_shutdown("done")



if __name__ == "__main__":
    start = start_scripts()
    start.run()
    rospy.spin()

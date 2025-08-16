#!/usr/bin/env python3

import rospy
import subprocess
import os
import socket
import json
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans
from std_srvs.srv import *
from topic_tools.srv import MuxSelect

from config import BASE_PATH, CHAR_PATH

class MainGreeting:
    def __init__(self):
        rospy.init_node('main_greeting', anonymous=True)
        rospy.Service('person_centered', SetBool, self.person_centered_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/resume_cmd', Twist, queue_size=10)
        self.activate_pub = rospy.Publisher('/activate_model', Bool, queue_size=10)
        self.resume_rotation_sub = rospy.Subscriber('/resume_rotation', Bool, self.resume_rotation_callback)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)

        # Remove the old topic-based communication
        # self.pub_report = rospy.Publisher('report', String, queue_size=10)
        # self.pub_sub = rospy.Subscriber('report', String, self.enough)

        self.person_centered = False
        self.initial_check_done = False
        self.start_yaw = 0
        self.current_yaw = 0
        self.angle_turned = 0
        self.resume_rotation = True
        self.guest_count = 0
        self.current_room = "Unknown Room"  # Initialize current_room
        self.capture = False
        self.launched = True
        self.ran_interact = False
        self.ask_out = False  # Initialize ask_out with default value

        launch_file = os.path.join(BASE_PATH, "launch/general.launch")
        self.general_launch_process = subprocess.Popen(["roslaunch", launch_file])
        rospy.loginfo("Launched general.launch")

        rospy.loginfo("waiting for capture_image service")
        rospy.wait_for_service("capture_image")
        rospy.loginfo("waiting for next_way service")
        # rospy.wait_for_service("next_way")
        rospy.loginfo("wait for shut_depth")
        rospy.wait_for_service("shut_depth")

        # Wait for the report update service
        rospy.loginfo("waiting for report_update service")
        # rospy.wait_for_service("report_update")

        # Initialize service proxies
        self.shutdown_depth = rospy.ServiceProxy("shut_depth", Trigger)
        self.capture_image = rospy.ServiceProxy("capture_image", Trigger)
        self.go_to_next_way = rospy.ServiceProxy("next_way", SetBool)
        self.report_update_service = rospy.ServiceProxy("report_update", SetBool)

        self.rate = rospy.Rate(10)

        # Launch the general.launch file

        # Clear the JSON file at the start of the session
        self.clear_json_file()

    def check_enough_people(self):
        """Check if there are enough people using the service call"""
        try:
            rospy.loginfo("Calling report_update service to check people count...")
            # Call the service with dummy data (the service doesn't use the request data)
            response = self.report_update_service(True)

            if response.success:
                rospy.loginfo(f"Service response: {response.message}")
                self.ask_out = False  # Enough people, don't ask to come out
                return True
            else:
                rospy.loginfo(f"Service response: {response.message}")
                self.ask_out = True   # Not enough people, need to ask to come out
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call report_update service: {e}")
            self.ask_out = True  # Default to asking people to come out on error
            return False

    # Remove the old topic-based method
    # def enough(self, msg):
    #     if msg == "enough":
    #         self.ask_out = False
    #     if msg == "not enough":
    #         self.ask_out = True

    def safe_terminate(self, process):
        if process and process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                rospy.logwarn("Process didn't terminate gracefully, killing")
                process.kill()
                process.wait()

    def person_centered_callback(self, msg):
        self.person_centered = msg.data
        self.initial_check_done = True
        return SetBoolResponse(success=True, message="update person centered")

    def resume_rotation_callback(self, msg):
        self.resume_rotation = msg
        if hasattr(self, 'interact_launch_process'):
            self.interact_launch_process.terminate()

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf_trans.euler_from_quaternion(orientation_list)

        self.current_yaw = yaw

        if self.start_yaw is None:
            self.start_yaw = yaw

    def calculate_angle_turned(self, start_yaw, current_yaw):
        angle = (current_yaw - start_yaw) * (180 / 3.14159)
        if angle < -2:
            angle += 360
        return angle

    def clear_json_file(self):
        json_file_path = os.path.join(BASE_PATH, "guest_details.json")
        with open(json_file_path, 'w') as file:
            json.dump([], file)  # Write an empty list to clear the file
        rospy.loginfo("Cleared guest details JSON file.")

    def switch_mux(self, target):
        rospy.wait_for_service('/vel_mux/select')
        try:
            select = rospy.ServiceProxy('/vel_mux/select', MuxSelect)
            select(target)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to switch mux: {e}")

    def run(self):
        twist = Twist()
        timeout = rospy.Time.now() + rospy.Duration(5)
        self.switch_mux("resume_cmd")
        while not self.initial_check_done and rospy.Time.now() < timeout:
            rospy.loginfo("Waiting for the initial person detection status...")
            rospy.sleep(0.5)
        if not self.initial_check_done:
            rospy.logwarn("No person detection data received. Exiting...")
            return

        rospy.loginfo("Start publishing angular velocity")
        acc_start_yaw = self.current_yaw
        acc_total_angle = 0
        while (not rospy.is_shutdown()) and acc_total_angle < 345:
            twist.angular.z = 0.5
            self.vel_pub.publish(twist)
            if self.person_centered:
                # Use service call instead of topic publish
                enough_people = self.check_enough_people()
                rospy.loginfo(f"Enough people check result: {enough_people}, ask_out: {self.ask_out}")

                self.person_centered = False
                twist.angular.z = 0
                self.vel_pub.publish(twist)
                self.start_yaw = self.current_yaw

                self.activate_pub.publish(True)
                rospy.loginfo("Published True to /activate_model")

                self.guest_count += 1

                rospy.loginfo("Waiting for gemini_photo.py to complete...")
                # self.trigger_gemini_photo()  # Call the gemini_photo.py script
                # self.run_comparison()
                response_img = self.capture_image()
                self.capture = response_img.success

                if self.capture and (not self.ask_out):
                    self.safe_terminate(self.general_launch_process)
                    launch_file = os.path.join(BASE_PATH, "launch/interact.launch")
                    self.interact_launch_process = subprocess.Popen(["roslaunch", launch_file])
                    rospy.loginfo("Launched interact.launch")
                    self.ran_interact = True
                    self.capture = False
                    self.resume_rotation = False

                else:
                    subprocess.run(["rosrun", "ffm_task", "name.py"])

                # Wait until the resume rotation signal is received
                rospy.loginfo("Waiting for /resume_rotation to be True...")
                while (not self.resume_rotation and self.ask_out) and not rospy.is_shutdown():
                    self.rate.sleep()

                rospy.loginfo("Resuming rotation after receiving /resume_rotation signal.")

                angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                while angle < 5:
                    rospy.loginfo(f"Start forced rotation, {angle}")
                    twist.angular.z = 0.5
                    self.vel_pub.publish(twist)
                    angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                    self.rate.sleep()
                if not self.ask_out:
                    launch_file = os.path.join(BASE_PATH, "launch/general.launch")
                    self.general_launch_process = subprocess.Popen(["roslaunch", launch_file])
                    rospy.sleep(1.0)
            self.rate.sleep()
            acc_current_yaw = self.current_yaw
            acc_total_angle = self.calculate_angle_turned(acc_start_yaw, acc_current_yaw)
        if self.ran_interact:
            self.safe_terminate(self.interact_launch_process)
        self.safe_terminate(self.general_launch_process)
        self.go_to_next_way(True)
        #self.move_to_operator()
        # self.run_comparison()

    def rotate_robot(self, angular_velocity):
        twist = Twist()
        twist.angular.z = angular_velocity
        self.vel_pub.publish(twist)

    def __del__(self):
        # Ensure the general_launch_process is terminated when the node is shut down
        self.safe_terminate(self.general_launch_process)
        if hasattr(self, 'interact_launch_process'):
            self.safe_terminate(self.interact_launch_process)

if __name__ == '__main__':
    try:
        main_greeting = MainGreeting()
        main_greeting.run()
    except rospy.ROSInterruptException:
        pass

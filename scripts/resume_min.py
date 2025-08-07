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

from tts_utils import play_audio
from config import BASE_PATH, CHAR_PATH

class MainGreeting:
    def __init__(self):
        rospy.init_node('main_greeting', anonymous=True)
        self.person_centered_sub = rospy.Subscriber('/person_centered', Bool, self.person_centered_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.activate_pub = rospy.Publisher('/activate_model', Bool, queue_size=10)
        self.resume_rotation_sub = rospy.Subscriber('/resume_rotation', Bool, self.resume_rotation_callback)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)

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

        launch_file = os.path.join(BASE_PATH, "launch/general.launch")
        self.general_launch_process = subprocess.Popen(["roslaunch", launch_file])
        rospy.loginfo("Launched general.launch")

        rospy.loginfo("waiting for capture_image service")
        rospy.wait_for_service("capture_image")
        rospy.loginfo("waiting for next_way service")
        rospy.wait_for_service("next_way")
        rospy.loginfo("wait for shut_yolo service")
        rospy.wait_for_service("shut_depth")
        self.shutdown_depth = rospy.ServiceProxy("shut_depth", Trigger)
        self.capture_image = rospy.ServiceProxy("capture_image", Trigger)
        self.go_to_next_way = rospy.ServiceProxy("next_way", SetBool)


        self.rate = rospy.Rate(10)

        # Launch the general.launch file

        # Clear the JSON file at the start of the session
        self.clear_json_file()

    def person_centered_callback(self, msg):
        self.person_centered = msg.data
        self.initial_check_done = True

    def resume_rotation_callback(self, msg):
        self.resume_rotation = msg
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

    def check_internet_connection(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

    def clear_json_file(self):
        json_file_path = os.path.join(BASE_PATH, "guest_details.json")
        with open(json_file_path, 'w') as file:
            json.dump([], file)  # Write an empty list to clear the file
        rospy.loginfo("Cleared guest details JSON file.")

    def trigger_gemini_photo(self):
        gemini_photo_path = os.path.join(BASE_PATH, "scripts/gemini_photo.py")
        rospy.loginfo(f"Guest {self.guest_count} detected! Triggering gemini_photo.py...")

        play_audio("guest_detected.wav", BASE_PATH)

        # Set GUEST_ID environment variable to pass it to gemini_photo.py
        env = os.environ.copy()
        env['GUEST_ID'] = str(self.guest_count)  # Use the guest_count as GUEST_ID
        env['CURRENT_ROOM'] = self.current_room  # Pass the room name as an environment variable

        # Trigger the gemini_photo script
        rospy.loginfo("Triggering gemini_photo.py...")
        subprocess.run([gemini_photo_path], env=env)
        rospy.loginfo("gemini_photo.py process completed.")


    def move_to_operator(self):
        """
        Placeholder function to simulate moving to the operator.
        """
        rospy.loginfo("Stopping guest detection. Moving to the operator...")
        print("Simulating the robot moving to the operator...")

    def run_comparison(self):
        has_internet = self.check_internet_connection()

        if has_internet:
            compare_script_path = os.path.join(BASE_PATH, "scripts/gemini_compare.py")
            rospy.loginfo("Running online guest comparison...")
        else:
            compare_script_path = os.path.join(BASE_PATH, "scripts/offline_compare.py")
            rospy.logwarn("No internet connection. Running offline guest comparison...")

        subprocess.run([compare_script_path])

    def run(self):
        twist = Twist()
        timeout = rospy.Time.now() + rospy.Duration(5)
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
            twist.angular.z = 0.3
            self.vel_pub.publish(twist)
            if self.person_centered:
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
                if self.capture:
                    self.general_launch_process.terminate()
                    launch_file = os.path.join(CHAR_PATH, "launch/interact.launch")
                    self.interact_launch_process = subprocess.Popen(["roslaunch", launch_file])
                    rospy.loginfo("Launched general.launch")
                    self.capture = False
                    self.resume_rotation = False

                # Wait until the resume rotation signal is received
                rospy.loginfo("Waiting for /resume_rotation to be True...")
                while not self.resume_rotation and not rospy.is_shutdown():
                    self.rate.sleep()

                rospy.loginfo("Resuming rotation after receiving /resume_rotation signal.")

                angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                while angle < 15:
                    rospy.loginfo(f"Start forced rotation, {angle}")
                    twist.angular.z = 0.31
                    self.vel_pub.publish(twist)
                    angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                    self.rate.sleep()
                rospy.sleep(1.0)
            self.rate.sleep()
            acc_current_yaw = self.current_yaw
            acc_total_angle = self.calculate_angle_turned(acc_start_yaw, acc_current_yaw)
        self.go_to_next_way(True)
        #self.move_to_operator()
        # self.run_comparison()

    def rotate_robot(self, angular_velocity):
        twist = Twist()
        twist.angular.z = angular_velocity
        self.vel_pub.publish(twist)

    def __del__(self):
        # Ensure the general_launch_process is terminated when the node is shut down
        if self.general_launch_process:
            self.general_launch_process.terminate()
            self.general_launch_process.wait()

if __name__ == '__main__':
    try:
        main_greeting = MainGreeting()
        main_greeting.run()
    except rospy.ROSInterruptException:
        pass

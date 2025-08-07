#!/usr/bin/env python3
import rospy
import subprocess
from map_search.srv import *
from std_srvs.srv import *

class start_scripts:
    def __init__(self):
        rospy.init_node("start_scripts")
        rospy.loginfo("waiting for service")
        # rospy.wait_for_service("go_to_waypoint")
        # self.go = rospy.ServiceProxy('go_to_waypoint', waypoint)
        self.service = rospy.Service('next_way', SetBool, self.rotation_done)
        self.way_done = rospy.Service('way_done', SetBool, self.waypoint_done)
        rospy.loginfo("start servicee")
        self.next_waypoint = True
        self.arrive = False
        self.rotate = "/home/i_h8_ros/ffm_ws/src/ffm_pkg/scripts/resume_min.py"
        self.room_number = 0

    def run_script(self, path):
        exe = "/usr/bin/python3"
        self.process = subprocess.Popen([exe, path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def waypoint_done(self, req):
        self.arrive = req.data
        return SetBoolResponse(success=True, message="waypoint status updated")

    def rotation_done(self, req):
        self.next_waypoint = req.data
        self.process.terminate()
        return SetBoolResponse(success=True, message="rotation status updated")

    def run(self):
        while not rospy.is_shutdown():
            if self.next_waypoint:
                # reached = self.go(self.room_number)
                self.room_number =+ 1
                self.next_waypoint = False
                # self.arrive = reached.success
            if self.arrive:
                self.run_script(self.rotate)
                self.arrive = False

if __name__ == "__main__":
    start = start_scripts()
    start.run()

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
import numpy as np
from config import DIR

class appendWaypoints:
    def __init__(self):
        rospy.init_node("append_waypoints")
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self.dt = np.dtype([('x', float), ('y', float), ('z', float)])

    def callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        wp = np.array([(x, y, z)], dtype=self.dt)
        with open(f"/home/{DIR}/src/ffm_task/waypoints.txt", 'a') as f:
            f.write(f"{wp[0]['x']} {wp[0]['y']} {wp[0]['z']}\n")
        rospy.loginfo(f"Saved waypoints to waypoints.txt")

if __name__ == "__main__":
    aWay = appendWaypoints()
    rospy.spin()

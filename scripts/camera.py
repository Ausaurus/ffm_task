#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import *

class WebcamFilter:
    def __init__(self):
        rospy.init_node("webcam_filter")
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher("/webcam/rgb/image_raw", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("/webcam/depth/image_raw", Image, queue_size=1)  # optional placeholder
        self.service = rospy.Service("shut_depth", Trigger, self.shutdown)

        # OpenCV webcam init
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("❌ Failed to open webcam")
            exit(1)
        rospy.loginfo("✅ Webcam initialized")

    def shutdown(self, req=None):
        rospy.loginfo("🛑 Shutting down webcam node")
        self.cap.release()
        rospy.signal_shutdown("Webcam node stopped")
        return TriggerResponse(success=True, message="webcam node stopped")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("⚠️ Frame capture failed")
                continue

            # Create dummy depth map (grayscale blur)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            depth_dummy = cv2.GaussianBlur(gray, (21, 21), 0).astype(np.float32)

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                depth_msg = self.bridge.cv2_to_imgmsg(depth_dummy, "32FC1")

                self.pub_image.publish(img_msg)
                self.pub_depth.publish(depth_msg)

            except Exception as e:
                rospy.logerr(f"Conversion error: {e}")

            if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                break
            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        node = WebcamFilter()
        node.run()
    except rospy.ROSInterruptException:
        pass


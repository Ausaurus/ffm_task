#!/usr/bin/env python3

import rospy
import os
import cv2
from sensor_msgs.msg import Image
from std_srvs.srv import *
from cv_bridge import CvBridge, CvBridgeError

from config import BASE_PATH, CAMERA_TOPIC

class TakePhoto:
    def __init__(self):
        rospy.init_node('take_photo_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color", Image, self.image_callback)
        self.capture_service = rospy.Service('capture_image', Trigger, self.capture_image)
        self.latest_image = None
        self.number = 1
        rospy.loginfo("TakePhoto node is ready. Waiting for service call...")

    def image_callback(self, data):
        self.latest_image = data

    def capture_image(self, req):
        rospy.loginfo("Capture image service called")

        if self.latest_image is None:
            rospy.logwarn("No image received yet! Is the camera connected?")
            return TriggerResponse(success=False, message="No image received yet. Please check the camera connection.")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            image_path = "/tmp/person_snapshot.jpg"
            cv2.imwrite(image_path, cv_image)
            rospy.loginfo(f"Image saved to {image_path}")
            return TriggerResponse(success=True, message="Image captured and saved successfully.")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return TriggerResponse(success=False, message=f"Failed to capture image: {e}")

if __name__ == '__main__':
    try:
        take_photo = TakePhoto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

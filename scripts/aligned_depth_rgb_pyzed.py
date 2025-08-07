#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyzed.sl as sl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import *

class zed_filter:
    def __init__(self):
        rospy.init_node("zed_filter")
        self.bridge = CvBridge()
        self.win = False
        self.pub_cv2image = rospy.Publisher("/zed2/zed_node/rgb_raw/image_raw_color", Image, queue_size=0)
        self.pub_depthimage = rospy.Publisher("/zed2/zed_node/depth/depth_registered", Image, queue_size=0)
        self.service = rospy.Service("shut_depth", Trigger, self.shutdown)
        # Create a ZED camera
        self.zed = sl.Camera()

        # Create configuration parameters
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use NEURAL depth mode
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        self.selection_rect = sl.Rect()


        # Open the camera
        cam = self.zed.open(init_params)
        if (cam!=sl.ERROR_CODE.SUCCESS):
            exit(-1)
        self.runtime = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.depth = sl.Mat()

    def shutdown(self):
        rospy.sleep(1)
        rospy.signal_shutdown("shutdown so that characteristics_rs.py can open realsense")
        return TriggerResponse(success=True, message="depth closed")

    def show_window(self):
        key = ''
        while key != 113:
            grab_status = self.zed.grab(self.runtime)
            if grab_status == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
                self.cvImage_bgra = self.image.get_data()
                self.cvImage_bgr = cv2.cvtColor(self.cvImage_bgra, cv2.COLOR_BGRA2BGR)
                self.depth_np = self.depth.get_data()
            if (not self.selection_rect.is_empty() and self.selection_rect.is_contained(sl.Rect(0,0,self.cvImage_bgra.shape[1],self.cvImage_bgra.shape[0]))): #Check if selection rectangle is valid and draw it on the image
                cv2.rectangle(self.cvImage_bgra,(self.selection_rect.x,self.selection_rect.y),(self.selection_rect.width+self.selection_rect.x,self.selection_rect.height+self.selection_rect.y),(220,180,20),2)
            cv2.imshow("self.zed_window", self.cvImage_bgra)
            ros_image = self.bridge.cv2_to_imgmsg(self.cvImage_bgr, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(self.depth_np.astype(np.float32), "32FC1")
            self.pub_cv2image.publish(ros_image)
            self.pub_depthimage.publish(depth_msg)
            key = cv2.waitKey(5)
        cv2.destroyAllWindows()

    def run(self):
        if self.win:
            self.show_window()
        else:
            while not rospy.is_shutdown():
                grab_status = self.zed.grab(self.runtime)
                if grab_status == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                    self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
                    self.cvImage_bgra = self.image.get_data()
                    self.cvImage_bgr = cv2.cvtColor(self.cvImage_bgra, cv2.COLOR_BGRA2BGR)
                    self.depth_np = self.depth.get_data()
                if (not self.selection_rect.is_empty() and self.selection_rect.is_contained(sl.Rect(0,0,self.cvImage_bgra.shape[1],self.cvImage_bgra.shape[0]))): #Check if selection rectangle is valid and draw it on the image
                    cv2.rectangle(self.cvImage_bgra,(self.selection_rect.x,self.selection_rect.y),(self.selection_rect.width+self.selection_rect.x,self.selection_rect.height+self.selection_rect.y),(220,180,20),2)
                    cv2.rectangle(self.depth,(self.selection_rect.x,self.selection_rect.y),(self.selection_rect.width+self.selection_rect.x,self.selection_rect.height+self.selection_rect.y),(220,180,20),2)
                ros_image = self.bridge.cv2_to_imgmsg(self.cvImage_bgr, "bgr8")
                depth_msg = self.bridge.cv2_to_imgmsg(self.depth_np.astype(np.float32), "32FC1")
                self.pub_cv2image.publish(ros_image)
                self.pub_depthimage.publish(depth_msg)


if __name__=="__main__":
    zed = zed_filter()
    zed.run()

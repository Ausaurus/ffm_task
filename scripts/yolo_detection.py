#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from std_msgs.msg import Bool

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        rospy.loginfo("Node initialized")
        self.bridge = CvBridge()
        self.model = YOLO('/home/i_h8_ros/catkin_ws/src/ffm_pkg/yolov8n.pt')
        self.person_class_id = 0
        self.center_threshold = 20
        self.confidence_threshold = 0.7  # Confidence threshold for detections
        self.centered_pub = rospy.Publisher('/person_centered', Bool, queue_size=10)
        self.image_sub = rospy.Subscriber('/filter_rgb', Image, self.image_callback)

        # Set this to True to enable drawing and streaming
        self.enable_streaming = False

    def draw_and_stream_bounding_box(self, frame, boxes, num_people, person_at_center):
        """2
        Draw bounding boxes on the frame, display the number of detected people with confidence > 0.7,
        and display the video stream.
        """
        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Display the number of detected people on the top right corner
        cv2.putText(frame, f"People Detected: {num_people}", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"{person_at_center}", (50, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Display the video stream
        cv2.imshow('YOLO Detection', frame)
        cv2.waitKey(1)

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width // 2
        results = self.model.predict(frame, verbose=False)

        person_at_center = False
        closest_distance = float('inf')
        detected_boxes = []
        num_people_above_threshold = 0

        for result in results:
            boxes = result.boxes.xyxy
            class_ids = result.boxes.cls
            confidences = result.boxes.conf  # Get the confidence scores

            for box, class_id, confidence in zip(boxes, class_ids, confidences):
                class_id = int(class_id)
                confidence = float(confidence)
                if class_id == self.person_class_id and confidence > self.confidence_threshold:
                    detected_boxes.append(box)
                    num_people_above_threshold += 1
                    x1, y1, x2, y2 = map(int, box)  # Extract coordinates from the box
                    bbox_center_x = (x1 + x2) // 2
                    distance_to_center = abs(bbox_center_x - frame_center_x)

                    if distance_to_center < closest_distance:
                        closest_distance = distance_to_center
                        person_at_center = distance_to_center <= self.center_threshold

        self.centered_pub.publish(Bool(person_at_center))

        if self.enable_streaming:
            self.draw_and_stream_bounding_box(frame, detected_boxes, num_people_above_threshold, person_at_center)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = YoloDetector()
    detector.run()

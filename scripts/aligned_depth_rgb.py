#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialize the ROS node
rospy.init_node('depth_rgb_image_listener', anonymous=True)
rospy.loginfo("node initiliazed")
bridge = CvBridge()
depth_mask = None

# Publisher for the filtered RGB image
img_pub = rospy.Publisher("filter_rgb", Image, queue_size=2)

def display(input, title):
    normalized_input = cv2.normalize(input, None, 0, 255, cv2.NORM_MINMAX)
    input_8bit = np.uint8(normalized_input)
    cv2.imshow(title, input_8bit)

def depth_callback(msg):
    global depth_mask
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #rospy.loginfo("depth received")


        # Apply the depth filtering
        depth_mask = np.where((depth_image > 0) & (depth_image <= 1700), 1, 0).astype(np.uint8)

    except CvBridgeError as e:
        print(e)

def shift_and_fill(mask, shift, fill_value=0):
    # Create a copy of the mask to apply the shift
    shifted_mask = np.roll(mask, shift=shift, axis=(0, 1))

    # Determine the direction of the shift
    vertical_shift, horizontal_shift = shift

    # Apply the vertical shift: Fill rolled out areas with the fill value
    if vertical_shift > 0:
        shifted_mask[:vertical_shift, :] = fill_value
    elif vertical_shift < 0:
        shifted_mask[vertical_shift:, :] = fill_value

    # Apply the horizontal shift: Fill rolled out areas with the fill value
    if horizontal_shift > 0:
        shifted_mask[:, :horizontal_shift] = fill_value
    elif horizontal_shift < 0:
        shifted_mask[:, horizontal_shift:] = fill_value

    return shifted_mask

def extend_mask(mask, extension_size=20):
    # Create a kernel for dilation (square matrix)
    kernel = np.ones((extension_size, extension_size), np.uint8)

    # Dilate the mask to extend the unfiltered area
    extended_mask = cv2.dilate(mask, kernel, iterations=1)

    return extended_mask

def rgb_callback(msg):
    global depth_mask
    if depth_mask is None:
        return  # Wait until the depth image is received

    try:
        rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        #rospy.loginfo("rgb received")
        # Adjust the mask if needed
        # aligned_depth_mask = shift_and_fill(depth_mask, shift=(32, -17), fill_value=0)
        aligned_depth_mask = depth_mask


        # Extend the mask to make the unfiltered area larger
        extended_mask = extend_mask(aligned_depth_mask, extension_size=5)

        # Convert the mask to 8-bit
        extended_mask = extended_mask.astype(np.uint8)

        # Ensure the mask has the same dimensions as the RGB image
        if extended_mask.shape != rgb_image.shape[:2]:
            extended_mask = cv2.resize(extended_mask, (rgb_image.shape[1], rgb_image.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Apply the extended depth filter to the RGB image
        filtered_rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask=extended_mask)

        # Display the filtered RGB image
        # display(filtered_rgb_image, 'Filtered RGB Image')

        # Convert the filtered RGB image back to a ROS Image message and publish it
        img_msg = bridge.cv2_to_imgmsg(filtered_rgb_image, encoding="bgr8")
        img_pub.publish(img_msg)

        # Wait for a key event for 1ms, which allows OpenCV to handle the GUI events
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User requested shutdown')

    except CvBridgeError as e:
        print(e)
    except Exception as e:
        print(f"Unexpected error: {e}")

# Subscribe to the /camera/depth/image_raw topic
rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, depth_callback)

# Subscribe to the /camera/rgb/image_raw topic
rospy.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image, rgb_callback)

# Keep the program alive and processing callbacks
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

#!/home/i_h8_ros/catkin_ws/.py3.10/bin/python3

import rospy
from std_msgs.msg import String
import cv2
import os
from PIL import Image
import google.generativeai as genai
import signal
import time
from std_msgs.msg import Bool

# === Gemini AI setup ===
genai.configure(api_key=os.environ.get("GEMINI_API_KEY"))
model = genai.GenerativeModel("gemini-1.5-flash")
detection_enabled = False  # Start in disabled state

# === Timeout handler ===
def handler(signum, frame):
    raise TimeoutError("Gemini analysis took too long!")
signal.signal(signal.SIGALRM, handler)

def detection_callback(msg):
    global detection_enabled
    detection_enabled = msg.data
    rospy.loginfo(f"Detection {'enabled' if detection_enabled else 'disabled'}")

def control_callback(msg):
    global detection_enabled
    if msg.data.lower() == "stop":
        rospy.loginfo("Gemini node received stop signal")
        detection_enabled = False
        # rospy.signal_shutdown("Shutdown requested by /gemini_control")

def analyze_image(image_path):
    prompt = (
        "Analyze only the person and the chair they are sitting or standing near. "
        "Return these attributes in simple comma-separated format: \n"
        "- Gender (Male/Female)\n"
        "- Wearing Spectacles (Wearing Spectacles/Not Wearing Spectacles)\n"
        "- Shirt Color and Type\n"
        "- Skin Tone (Fair, Medium, or Dark only)\n"
        "- Face Shape (Round, Oval, Square, or Oblong only)\n"
        "- Long sleeve or short sleeve\n"
        "- Sitting or Standing\n"
        "(e.g., sitting on the couch, standing beside the lamp). \n"
        "Use only the allowed words. Do not write full sentences or extra details."
    )

    img = Image.open(image_path)
    signal.alarm(30)
    try:
        response = model.generate_content([prompt, img])
        signal.alarm(0)
        return response.text.strip()
    except TimeoutError:
        return "Gemini timeout"
    except Exception as e:
        return f"Gemini error: {e}"

def main():
    rospy.init_node("gemini_description_node", anonymous=True)
    pub = rospy.Publisher("/gemini_description", String, queue_size=10)
    rospy.Subscriber("/enable_detection", Bool, detection_callback)
    rospy.Subscriber("/gemini_control", String, control_callback)

    image_path = "/tmp/person_snapshot.jpg"  # Shared snapshot file

    rospy.loginfo("Gemini node started, waiting for snapshot...")
    rate = rospy.Rate(1)  # 1 Hz check for new snapshot

    last_mod_time = None

    while not rospy.is_shutdown():
        if detection_enabled and os.path.exists(image_path):
            mod_time = os.path.getmtime(image_path)
            if last_mod_time is None or mod_time != last_mod_time:
                rospy.loginfo("🔍 New snapshot detected, analyzing...")
                result = analyze_image(image_path)
                pub.publish(result)
                rospy.loginfo(f"📤 Gemini attributes: {result}")
                last_mod_time = mod_time
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#source ~/catkin_ws/src/gemini_ros/venv/bin/activate
#rosrun gemini_ros capture_and_analyze.py

#export GEMINI_API_KEY="your_actual_api_key"

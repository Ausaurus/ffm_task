#!/usr/bin/env python3

import cv2
import numpy as np
import mediapipe as mp
import pyzed.sl as sl
import rospy
from std_msgs.msg import String, Bool

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1)

# ----------------------------- GENDER DETECTION SETUP -----------------------------
# Load pre-trained gender detection model
modelFile = "/home/i_h8_ros/ffm_ws/src/person_description/gender_net.caffemodel"
configFile = "/home/i_h8_ros/ffm_ws/src/person_description/gender_deploy.prototxt"
net = cv2.dnn.readNet(modelFile, configFile)

# Gender labels
genders = ['Male', 'Female']
MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)

# Load Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_glasses_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye_tree_eyeglasses.xml')

# ----------------------------- COLOR NAME MATCHING UTILS -----------------------------
def closest_color_name(rgb):
    colors = {
        "black": (0, 0, 0), "white": (255, 255, 255),
        "red": (255, 0, 0), "lime": (0, 255, 0), "blue": (0, 0, 255),
        "yellow": (255, 255, 0), "cyan": (0, 255, 255), "magenta": (255, 0, 255),
        "gray": (128, 128, 128), "orange": (255, 165, 0), "brown": (165, 42, 42),
        "pink": (255, 192, 203), "purple": (128, 0, 128), "green": (0, 128, 0), "navy": (0, 0, 128)
    }
    def distance(c1, c2): return sum((a - b) ** 2 for a, b in zip(c1, c2))
    closest = min(colors.items(), key=lambda x: distance(rgb, x[1]))
    return closest[0]

def get_dominant_color(image):
    if image is None or image.size == 0:
        return (0, 0, 0)  # Return black if empty

    # Ensure the image has 3 channels (convert from BGRA or GRAY if needed)
    if len(image.shape) == 2:  # Grayscale
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif image.shape[2] == 4:  # BGRA
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    pixels = np.float32(image.reshape(-1, 3))
    _, labels, palette = cv2.kmeans(pixels, 1, None,
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.2),
                                    10, cv2.KMEANS_RANDOM_CENTERS)
    dominant_color = palette[0].astype(int)
    return tuple(dominant_color)

# ----------------------------- POSE ESTIMATION SETUP -----------------------------
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# ----------------------------- VIDEO STREAM SETUP -----------------------------
zed = sl.Camera()

init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.camera_fps = 30

if zed.open(init) != sl.ERROR_CODE.SUCCESS:
    print("ZED camera failed to open")
    exit()

runtime = sl.RuntimeParameters()
mat = sl.Mat()

# ----------------------------- ROS SETUP -----------------------------
rospy.init_node('person_characteristics_detector', anonymous=True)
pub = rospy.Publisher('/person_description', String, queue_size=10)

# Detection control
detection_enabled = True

def detection_control_callback(msg):
    global detection_enabled
    detection_enabled = msg.data
    status = "ENABLED" if detection_enabled else "DISABLED"
    rospy.loginfo(f"📡 Camera detection {status}")

def control_callback(msg):
    if msg.data == "stop":
        rospy.loginfo("📷 Camera node shutting down by request")
        zed.close()
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Camera stopped externally")

rospy.Subscriber('/enable_detection', Bool, detection_control_callback)
rospy.Subscriber("/camera_control", String, control_callback)

# Display variables for consistent text positioning
text_y_offset = 30
line_height = 25

rospy.loginfo("🎥 Camera detection node started")

while not rospy.is_shutdown():
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(mat, sl.VIEW.LEFT)
        frame = mat.get_data()
    else:
        continue

    # Initialize text position
    text_y = text_y_offset

    # Detection status display
    status_color = (0, 255, 0) if detection_enabled else (0, 0, 255)
    status_text = "DETECTION: ON" if detection_enabled else "DETECTION: OFF"
    cv2.putText(frame, status_text, (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    text_y += line_height

    if not detection_enabled:
        cv2.putText(frame, "Waiting for system...", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("Person Characteristics Detection", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            break
        continue

    # Save latest frame for Gemini
    cv2.imwrite("/tmp/person_snapshot.jpg", frame)

    # Convert frame for processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Initialize detection results
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    gender_info = "Unknown"
    glasses_info = "Unknown"
    face_shape_info = "Unknown"
    skin_tone_info = "Unknown"
    shirt_color_info = "Unknown"

    # FACE MESH PROCESSING for face shape and skin tone
    face_mesh_results = face_mesh.process(rgb)

    if face_mesh_results.multi_face_landmarks:
        for face_landmarks in face_mesh_results.multi_face_landmarks:
            ih, iw, _ = frame.shape
            landmarks = face_landmarks.landmark

            # Face shape detection
            jaw = [(int(l.x * iw), int(l.y * ih)) for l in landmarks[0:17]]
            jaw_width = np.linalg.norm(np.array(jaw[0]) - np.array(jaw[-1]))
            point_10 = (int(landmarks[10].x * iw), int(landmarks[10].y * ih))
            face_height = np.linalg.norm(np.array(jaw[8]) - np.array(point_10))

            ratio = jaw_width / face_height if face_height != 0 else 0

            if ratio > 1.6:
                face_shape_info = "Round"
            elif ratio > 1.3:
                face_shape_info = "Oval"
            elif ratio > 1.0:
                face_shape_info = "Square"
            else:
                face_shape_info = "Oblong"

            # Skin tone detection from cheek landmarks
            cheek_landmarks = [234, 454]
            colors = []

            for idx in cheek_landmarks:
                x = int(landmarks[idx].x * iw)
                y = int(landmarks[idx].y * ih)

                if 0 <= x < iw and 0 <= y < ih:
                    patch = frame[max(0, y-5):min(ih, y+5), max(0, x-5):min(iw, x+5)]
                    if patch.size > 0:
                        avg_color = np.mean(patch, axis=(0, 1))
                        colors.append(avg_color)

            if colors:
                avg_color = np.mean(colors, axis=0)
                if len(avg_color) >= 3:
                    b, g, r = avg_color[:3]
                    brightness = (r + g + b) / 3

                    if brightness > 180:
                        skin_tone_info = "Fair"
                    elif brightness > 80:
                        skin_tone_info = "Medium"
                    else:
                        skin_tone_info = "Dark"

    # GENDER AND GLASSES DETECTION
    for (x, y, w, h) in faces:
        # Gender detection
        face_img = frame[y:y + h, x:x + w].copy()
        if face_img.shape[2] == 4:
            face_img = cv2.cvtColor(face_img, cv2.COLOR_BGRA2BGR)

        blob = cv2.dnn.blobFromImage(face_img, 1.0, (227, 227), MODEL_MEAN_VALUES, swapRB=False)
        net.setInput(blob)
        gender_preds = net.forward()
        gender = genders[gender_preds[0].argmax()]
        confidence = gender_preds[0][gender_preds[0].argmax()]
        gender_info = f"{gender}"

        # Glasses detection
        roi_gray = gray[y:y + h, x:x + w]
        eyes = eye_glasses_cascade.detectMultiScale(roi_gray, scaleFactor=1.1, minNeighbors=4)

        if len(eyes) >= 2:
            glasses_info = "wearing spectacles"
        else:
            glasses_info = "not wearing spectacles"

        # Draw face rectangle
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # SHIRT COLOR DETECTION
    results = pose.process(rgb)
    if results.pose_landmarks:
        h, w, _ = frame.shape
        left_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]

        x1, y1 = int(left_shoulder.x * w), int(left_shoulder.y * h)
        x2, y2 = int(right_shoulder.x * w), int(right_shoulder.y * h)

        x_min, x_max = max(0, min(x1, x2)), min(w, max(x1, x2))
        y_start = max(0, y1)
        y_end = min(h, y_start + 50)

        shirt_region = frame[y_start:y_end, x_min:x_max]

        if shirt_region.size > 0:
            dominant_color = get_dominant_color(shirt_region)
            color_name = closest_color_name(dominant_color)
            shirt_color_info = f"{color_name}"

            # Draw shirt region rectangle
            cv2.rectangle(frame, (x_min, y_start), (x_max, y_end), (255, 0, 0), 2)

    # DISPLAY ALL CHARACTERISTICS ON SCREEN
    characteristics_detected = []

    if gender_info != "Unknown":
        cv2.putText(frame, f"Gender: {gender_info}", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        text_y += line_height
        characteristics_detected.append(f"Gender: {gender_info}")

    if glasses_info != "Unknown":
        cv2.putText(frame, f"Glasses: {glasses_info}", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        text_y += line_height
        characteristics_detected.append(f"Glasses: {glasses_info}")

    if shirt_color_info != "Unknown":
        cv2.putText(frame, f"Shirt Color: {shirt_color_info}", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        text_y += line_height
        characteristics_detected.append(f"Shirt Color: {shirt_color_info}")

    if face_shape_info != "Unknown":
        cv2.putText(frame, f"Face Shape: {face_shape_info}", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        text_y += line_height
        characteristics_detected.append(f"Face Shape: {face_shape_info}")

    if skin_tone_info != "Unknown":
        cv2.putText(frame, f"Skin Tone: {skin_tone_info}", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        text_y += line_height
        characteristics_detected.append(f"Skin Tone: {skin_tone_info}")

    # PUBLISH COMBINED MESSAGE (only if person detected and detection enabled)
    if characteristics_detected and detection_enabled:
        combined_msg = ", ".join(characteristics_detected)
        pub.publish(combined_msg)

        # Show publishing status
        cv2.putText(frame, "PUBLISHING: YES", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "PUBLISHING: NO", (10, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Show detection count
    detection_count = len(characteristics_detected)
    cv2.putText(frame, f"Characteristics Found: {detection_count}", (10, frame.shape[0] - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("Person Characteristics Detection", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
        break

zed.close()
cv2.destroyAllWindows()
rospy.loginfo("🎥 Camera detection node stopped")

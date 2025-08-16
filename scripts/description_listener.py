#!/usr/bin/env python3

import rospy
import re
import json
import os
import time
from std_msgs.msg import String, Bool
from std_srvs.srv import *
from enum import Enum

class SystemState(Enum):
    COLLECTING_NAME = 1
    CHECKING_CUSTOMER_STATUS = 2
    ANALYZING_CHARACTERISTICS = 3
    COMPARING_RESULTS = 4
    SAVING_CUSTOMER = 5
    SESSION_COMPLETE = 6

COMMON_COLORS = [
    "black", "white", "red", "green", "blue", "yellow", "purple",
    "pink", "brown", "gray", "grey", "orange", "beige", "light blue",
    "dark blue", "light green", "dark green", "light brown", "dark brown"
]

class SimplifiedCustomerRecognition:
    def __init__(self):
        rospy.init_node('simplified_customer_recognition', anonymous=True)

        # File path for customer database
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.customers_file = os.path.join(script_dir, "customers_database.json")


        # System state
        self.current_state = SystemState.COLLECTING_NAME
        self.gemini_timeout = rospy.get_param("~gemini_timeout", 20.0)

        # Current session data
        self.customer_name = None
        self.camera_description = None
        self.gemini_description = None

        # Data flags
        self.name_received = False
        self.camera_received = False
        self.gemini_received = False
        self.gemini_timeout_occurred = False

        # Load existing customer database
        self.customers_database = self.load_customers_database()

        # Timer for Gemini timeout
        self.gemini_timer = None

        # Publishers
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
        self.detection_control_pub = rospy.Publisher('/enable_detection', Bool, queue_size=10)
        self.system_status_pub = rospy.Publisher('/system_status', String, queue_size=10)
        self.customer_result_pub = rospy.Publisher('/customer_result', String, queue_size=10)
        self.camera_control_pub = rospy.Publisher('/camera_control', String, queue_size=1)
        self.gemini_control_pub = rospy.Publisher("/gemini_control", String, queue_size=1)
        self.feature_pub = rospy.Publisher('/feature_done', Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/customer_name', String, self.name_callback)
        rospy.Subscriber('/person_description', String, self.camera_callback)
        rospy.Subscriber('/gemini_description', String, self.gemini_callback)

        # Services
        # rospy.wait_for_service("shut_name")
        # rospy.loginfo("got shut_name service")
        # self.shut_name = rospy.ServiceProxy("shut_name", Trigger)

        rospy.loginfo("🤖 Simplified Customer Recognition System Started")
        rospy.loginfo(f"📊 Known customers: {len(self.customers_database)}")

        # Start the session
        self.start_session()

    def load_customers_database(self):
        if os.path.exists(self.customers_file):
            try:
                with open(self.customers_file, 'r') as f:
                    data = json.load(f)
                    # Convert list of single-key dicts to a flat dictionary
                    if isinstance(data, list):
                        database = {}
                        for entry in data:
                            name = list(entry.keys())[0]  # Get the name (key)
                            database[name] = entry[name]   # Store under name
                        return database
                    return data  # Already in dict format
            except Exception as e:
                rospy.logerr(f"Failed to load database: {e}")
        return {}  # Fallback to empty dict

    def save_customers_database(self):
        """Save customer database to JSON file"""
        try:
            # Load existing data
            if os.path.exists(self.customers_file):
                with open(self.customers_file, 'r') as f:
                    existing_data = json.load(f)
            else:
                existing_data = {}

            # Ensure existing_data is a dictionary
            if isinstance(existing_data, list):
                # Convert list format to dictionary format
                converted_data = {}
                for entry in existing_data:
                    if isinstance(entry, dict):
                        # If entry is a single customer record with nested structure
                        for name, customer_info in entry.items():
                            converted_data[name] = customer_info
                existing_data = converted_data

            # Update existing data with current database
            existing_data.update(self.customers_database)

            # Save back to file
            with open(self.customers_file, 'w') as f:
                json.dump(existing_data, f, indent=2)

            rospy.loginfo("💾 Customer database saved successfully")

        except Exception as e:
            rospy.logerr(f"❌ Failed to save customers database: {e}")
            # Fallback: save only current data
            try:
                with open(self.customers_file, 'w') as f:
                    json.dump(self.customers_database, f, indent=2)
                rospy.loginfo("💾 Customer database saved (fallback mode)")
            except Exception as fallback_error:
                rospy.logerr(f"❌ Fallback save also failed: {fallback_error}")

    def publish_system_status(self):
        """Publish current system status"""
        status = {
            'state': self.current_state.name,
            'customer_name': self.customer_name,
            'data_received': {
                'name': self.name_received,
                'camera': self.camera_received,
                'gemini': self.gemini_received
            }
        }
        self.system_status_pub.publish(json.dumps(status))

    def start_session(self):
        """Start customer recognition session"""
        rospy.loginfo("🆕 Starting customer recognition session")

        # Ask for customer name
        self.current_state = SystemState.COLLECTING_NAME

        self.publish_system_status()

    def name_callback(self, msg):
        """Handle customer name input"""
        # if self.current_state != SystemState.COLLECTING_NAME:
        #     return

        name = msg.data.strip().title()
        self.customer_name = name
        self.name_received = True
        self.tts_pub.publish(f"Hi {self.customer_name}")
        # self.shut_name()

        rospy.loginfo(f"📛 Customer name received: {name}")
        self.current_state = SystemState.CHECKING_CUSTOMER_STATUS
        self.check_customer_status()

    def check_customer_status(self):
        """Check if customer exists in database"""
        if self.customer_name in self.customers_database:
            # Customer exists - close the node
            self.handle_existing_customer()
        else:
            # New customer - start characteristic analysis
            self.tts_pub.publish("new customer")
            self.handle_new_customer()

    def handle_existing_customer(self):
        """Handle existing customer - close the node"""
        customer_info = self.customers_database[self.customer_name]
        rospy.loginfo(f"👋 Existing customer found: {self.customer_name}")

        # Greet the existing customer
        response = f"I know you {self.customer_name}"
        self.tts_pub.publish(response)

        # Publish result
        result = f"EXISTING CUSTOMER: {self.customer_name} - Already in database"
        self.customer_result_pub.publish(result)

        rospy.loginfo(f"✅ Existing customer detected. Closing node.")
        self.feature_pub.publish(True)

        # Close the node
        rospy.Timer(rospy.Duration(3.0), lambda e: self.end_session(), oneshot=True)

    def handle_new_customer(self):
        """Handle new customer - start characteristic analysis"""
        rospy.loginfo(f"🆕 New customer detected: {self.customer_name}")

        self.tts_pub.publish("Look at the camera")
        self.detection_control_pub.publish(Bool(data=True))

        # ✅ Start detection only for new customers
        self.current_state = SystemState.ANALYZING_CHARACTERISTICS
        self.start_gemini_timeout()
        self.publish_system_status()

    def start_gemini_timeout(self):
        """Start Gemini analysis timeout timer"""
        if self.gemini_timer:
            self.gemini_timer.shutdown()

        self.gemini_timer = rospy.Timer(
            rospy.Duration(self.gemini_timeout),
            self.gemini_timeout_callback,
            oneshot=True
        )
        rospy.loginfo(f"⏱️ Started Gemini timeout ({self.gemini_timeout}s)")

    def camera_callback(self, msg):
        """Handle camera description"""
        if self.current_state != SystemState.ANALYZING_CHARACTERISTICS:
            return

        self.camera_description = msg.data
        self.camera_received = True

        rospy.loginfo(f"📷 Camera description: {self.camera_description}")
        self.check_analysis_completion()

    def gemini_callback(self, msg):
        """Handle Gemini analysis result"""
        if self.current_state != SystemState.ANALYZING_CHARACTERISTICS:
            return

        self.gemini_description = msg.data
        self.gemini_received = True
        # self.detection_control_pub.publish(Bool(data=False))

        rospy.loginfo(f"🧠 Gemini description: {self.gemini_description}")

        # Cancel timeout since we received Gemini data
        if self.gemini_timer:
            self.gemini_timer.shutdown()
            self.gemini_timer = None

        self.check_analysis_completion()

    def gemini_timeout_callback(self, event):
        """Handle Gemini timeout"""
        self.gemini_timeout_occurred = True
        rospy.logwarn("⏰ Gemini analysis timed out")
        self.check_analysis_completion()

    def check_analysis_completion(self):
        """Check if we have enough data to proceed"""
        # Need camera data and either Gemini data or timeout
        if self.camera_received and (self.gemini_received or self.gemini_timeout_occurred):
            self.current_state = SystemState.COMPARING_RESULTS
            self.compare_and_save_results()

    def compare_and_save_results(self):
        """Compare camera and Gemini results and save customer"""
        rospy.loginfo("📊 Comparing camera and Gemini results...")

        # Extract characteristics from both sources
        camera_chars = self.extract_characteristics(self.camera_description, "camera")

        comparison_result = {}
        data_confidence = "low"
        location_info = None

        if self.gemini_received:
            gemini_chars = self.extract_characteristics(self.gemini_description, "gemini")
            location_info, got = self.extract_location_from_gemini(self.gemini_description)
            # Find matching characteristics
            matching_chars = self.find_matching_characteristics(camera_chars, gemini_chars)

            comparison_result = {
                'Camera_characteristics': camera_chars,
                'Gemini_characteristics': gemini_chars,
                'Matching_characteristics': matching_chars,
                'Match_count': len(matching_chars)
            }

            data_confidence = "high" if len(matching_chars) > 0 else "medium"
            rospy.loginfo(f"🔍 Found {len(matching_chars)} matching characteristics")
        else:
            # Use camera data only, treat as matching
            comparison_result = {
                'Camera_characteristics': camera_chars,
                'Gemini_characteristics': None,
                'Matching_characteristics': camera_chars,
                'Match_count': len(camera_chars),
                'Note': 'Gemini timed out'
            }
            rospy.loginfo("Using camera data only (Gemini timeout) — assuming full match")


        # Save customer to database
        customer_record = {
            'Name': self.customer_name,
            'Camera_description': self.camera_description,
            'Gemini_description': self.gemini_description if self.gemini_received else None,
            'Gemini_location': location_info,
            'Comparison_result': comparison_result,
            'Data_confidence': data_confidence,
            'Registration_date': time.strftime("%Y-%m-%d %H:%M:%S"),
            'Gemini_timeout_occurred': self.gemini_timeout_occurred
        }

        self.customers_database[self.customer_name] = customer_record
        self.save_customers_database()

        # Respond to customer
        if data_confidence == "high":
            response = f"Recorded {self.customer_name} with high confidence"
        elif data_confidence == "medium":
            response = f"Recorded {self.customer_name} with medium confidence"
        else:
            response = f"Recorded {self.customer_name} with low confidence"

        self.tts_pub.publish(response)

        # Publish result
        match_summary = f"{comparison_result['Match_count']} matches" if self.gemini_received else "camera only"
        result = f"NEW CUSTOMER SAVED: {self.customer_name} - {match_summary} (confidence: {data_confidence})"
        self.customer_result_pub.publish(result)

        rospy.loginfo(f"✅ New customer saved: {self.customer_name}")
        rospy.loginfo(f"📊 Comparison result: {comparison_result}")

        self.current_state = SystemState.SESSION_COMPLETE
        self.feature_pub.publish(True)

        # End session
        rospy.Timer(rospy.Duration(3.0), lambda e: self.end_session(), oneshot=True)

    def extract_characteristics(self, description, source):
        """Extract characteristics from description text"""
        if not description:
            return {}

        characteristics = {}
        desc_lower = description.lower()

        # Gender detection
        if re.search(r'\bmale\b', desc_lower) and not re.search(r'\bfemale\b', desc_lower):
            characteristics['gender'] = 'Male'
        elif re.search(r'\bfemale\b', desc_lower):
            characteristics['gender'] = 'Female'

        # Glasses detection
        if 'wearing spectacles' in desc_lower or 'with glasses' in desc_lower:
            characteristics['glasses'] = 'Wearing spectacles'
        elif 'not wearing spectacles' in desc_lower or 'no glasses' in desc_lower:
            characteristics['glasses'] = 'Not wearing spectacles'

        # Shirt color detection
        for color in COMMON_COLORS:
            if color in desc_lower:
                characteristics['shirt_color'] = color.title()
                break

        # Face shape detection
        shapes = ['oval', 'round', 'square', 'oblong']
        for shape in shapes:
            if shape in desc_lower:
                characteristics['face_shape'] = shape.title()
                break

        # Skin tone detection
        tones = ['fair', 'medium', 'dark']
        for tone in tones:
            if tone in desc_lower:
                characteristics['skin_tone'] = tone.title()
                break

        rospy.loginfo(f"📊 Extracted from {source}: {characteristics}")
        return characteristics

    def extract_location_from_gemini(self, description):
        """Extract relative location description from Gemini text"""
        if not description:
            return None, False

        lines = description.split(",")
        for line in lines:
            line = line.strip()
            # Heuristic: assume location is a phrase like "sitting on the couch", "standing beside the table"
            if any(prep in line for prep in ["on the", "beside the", "next to", "near the", "at the"]):
                return line, True
        return None, False

    def find_matching_characteristics(self, camera_chars, gemini_chars):
        """Find characteristics that match between camera and Gemini"""
        matching = {}

        for key, value in camera_chars.items():
            if key in gemini_chars and gemini_chars[key].lower() == value.lower():
                matching[key] = value

        return matching

    def end_session(self):
        """End the session and close the node"""
        rospy.loginfo("🏁 Session complete. Closing node.")
        self.camera_control_pub.publish("stop") #close the camera
        self.gemini_control_pub.publish("stop")

        # Disable camera detection
        self.detection_control_pub.publish(Bool(data=False))

        self.tts_pub.publish("Registration done")

        # Close the node
        # rospy.signal_shutdown("Session completed successfully")

if __name__ == '__main__':
    try:
        system = SimplifiedCustomerRecognition()
        rospy.loginfo("🚀 Simplified Customer Recognition System ready")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("👋 Customer Recognition System shutting down")
        pass

#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import Bool, String
import os

class ReporterNode:
    def __init__(self):
        rospy.init_node('reporter', anonymous=True)

        # Params
        # self.json_path = rospy.get_param('~json_path', '/path/to/people.json')
        self.json_path = "/home/i_h8_ros/ffm_ws/src/ffm_task/scripts/customers_database.json"

        # Publishers
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=1, latch=True)
        self.reported_pub = rospy.Publisher('/rep_done', Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/report', Bool, self.reach_callback)

        # Keep track of used characteristic types
        self.used_types = set()

    def reach_callback(self, msg):
        if msg.data:
            self.process_latest_person()

    def process_latest_person(self):
        # Load JSON
        rospy.loginfo("Processing latest person from JSON")
        if not os.path.exists(self.json_path):
            rospy.logwarn("JSON file not found: %s", self.json_path)
            return

        with open(self.json_path, 'r') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                rospy.logwarn("Invalid JSON file")
                return

        if not data:
            rospy.logwarn("No person data available")
            return

        # ✅ Get the last inserted person if dictionary
        if isinstance(data, dict):
            latest_person_key = list(data.keys())[-1]
        elif isinstance(data, list) and data:
            # In case it’s still a list
            latest_person_key = list(data[-1].keys())[-1]
            data = data[-1]
        else:
            rospy.logwarn("Unexpected JSON structure")
            return

        latest_person = data[latest_person_key]
        rospy.loginfo(f"Processing latest person: {latest_person_key}")
        chosen_chars = self.choose_characteristics(latest_person)

        if chosen_chars:
            # Include person's name first
            name_text = f"Name: {latest_person_key}"
            char_text = ', '.join([f"{t}: {v}" for t, v in chosen_chars])
            full_text = f"{name_text}, {char_text}"

            self.tts_pub.publish(String(data=full_text))
            rospy.loginfo(f"TTS: {full_text}")

            # Estimate speaking time (roughly 150 wpm → 2.5 words/sec)
            words = len(full_text.split())
            duration = words / 2.5  # seconds

            rospy.Timer(rospy.Duration(duration), self.publish_reported, oneshot=True)
        else:
            rospy.logwarn("No characteristics could be selected")



    def publish_reported(self, event):
        self.reported_pub.publish(Bool(data=True))
        rospy.loginfo("Reported published after TTS finished.")

    def choose_characteristics(self, person_data):
        chosen = []
        possible_sources = ['Matching_characteristics', 'Gemini_characteristics', 'Camera_characteristics']
        comparison_result = person_data.get('Comparison_result', {})

        for source in possible_sources:
            characteristics = comparison_result.get(source, {})
            rospy.loginfo(f"Processing source: {source}, characteristics: {characteristics}")

            # Skip if it's not a dictionary
            if not isinstance(characteristics, dict):
                continue

            for char_type, char_value in characteristics.items():
                if not char_type or not char_value:
                    continue  # Skip null/empty

                if char_type not in self.used_types:
                    chosen.append((char_type, char_value))
                    self.used_types.add(char_type)
                    if len(chosen) == 2:
                        print(f"Chosen characteristics: {chosen}")
                        return chosen

        return chosen if chosen else None




if __name__ == '__main__':
    try:
        ReporterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

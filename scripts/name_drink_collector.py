#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String, Bool
import socket
from difflib import SequenceMatcher  # For similarity matching

# For offline recognition
try:
    from vosk import Model, KaldiRecognizer
    import json as jsonlib
    vosk_available = True
except ImportError:
    vosk_available = False

class VoiceNameDrinkCollectorNode:
    def __init__(self):
        rospy.init_node('name_drink_collector_node')

        # Known names and drinks
        self.known_names = [
            "liam", "noah", "oliver", "laura", "james", "william",
            "benjamin", "lucas", "henry", "alexander",
            "emma", "olivia", "ava", "sophia", "isabella"
        ]

        # Publishers
        self.name_pub = rospy.Publisher('/customer_name', String, queue_size=10)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)

        # Subscriber

        self.recognizer = sr.Recognizer()
        self.collection_active = False
        self.max_attempts = 3  # Maximum number of attempts per question

        # Microphone setup
        mic_list = sr.Microphone.list_microphone_names()
        if not mic_list:
            rospy.logerr("❌ No microphone detected.")
            raise RuntimeError("No microphone found.")

        rospy.loginfo(f"🎙️ Detected microphones: {mic_list}")
        try:
            self.mic = sr.Microphone()
            rospy.loginfo(f"🎤 Using microphone index: {self.mic.device_index}")
        except OSError as e:
            rospy.logerr(f"❌ Microphone error: {e}")
            raise

        # Load Vosk model if available
        if vosk_available:
            try:
                rospy.loginfo("📦 Loading Vosk offline model...")
                self.vosk_model = Model(lang="en-us")
                rospy.loginfo("✅ Vosk model loaded successfully.")
            except Exception as e:
                rospy.logwarn(f"⚠️ Could not load Vosk model: {e}")
                self.vosk_model = None
        else:
            self.vosk_model = None

    def is_online(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=2)
            return True
        except OSError:
            return False

    def start_collection_callback(self, msg):
        if msg.data and not self.collection_active:
            self.collection_active = True
            rospy.loginfo("🚀 Starting name and drink collection")
            self.collect_name_and_drink()

    def collect_name_and_drink(self):
        name = self.ask_with_retry("Welcome. What is your name?", "name")
        if not name:
            rospy.logwarn("Failed to collect name after multiple attempts")
            self.collection_complete_pub.publish(Bool(data=False))
            self.collection_active = False
            return

        rospy.loginfo(f"🗣️ Customer name: {name}")
        self.name_pub.publish(name)

        drink = self.ask_with_retry(f"Hi, {name}! What is your favourite drink?", "drink")
        if not drink:
            rospy.logwarn("Failed to collect drink after multiple attempts")
            self.collection_complete_pub.publish(Bool(data=False))
            self.collection_active = False
            return

        rospy.loginfo(f"☕ Customer drink: {drink}")
        self.drink_pub.publish(drink)

        self.collection_complete_pub.publish(Bool(data=True))
        self.collection_active = False

    def ask_with_retry(self, question, question_type):
        attempts = 0
        while attempts < self.max_attempts and not rospy.is_shutdown():
            attempts += 1
            rospy.loginfo(f"Attempt {attempts}/{self.max_attempts} for {question_type}")

            # Only ask the full question the first time
            if attempts == 1:
                response = self.ask_question(question, question_type)
            else:
                if question_type == "name":
                    self.tts_pub.publish("Please repeat name only louder")
                    response = self.ask_question("", question_type)
                else:
                    self.tts_pub.publish("Please repeat drink only louder")
                    response = self.ask_question("", question_type)

            if response:
                return response

        # If no response after retries, use default
        default_value = "Guest" if question_type == "name" else "Water"
        rospy.logwarn(f"❗ Using default {question_type}: {default_value}")
        self.tts_pub.publish(
            f"I'll call you {default_value}." if question_type == "name"
            else f"I'll give you {default_value}."
        )
        return default_value

    def best_match(self, spoken_text, options):
        """Return the closest match from options using similarity score."""
        spoken_text = spoken_text.lower().strip()
        best_score = 0
        best_option = None
        for option in options:
            score = SequenceMatcher(None, spoken_text, option.lower()).ratio()
            if score > best_score:
                best_score = score
                best_option = option
        if best_score >= 0.4:  # Confidence threshold
            rospy.loginfo(f"🔍 Best match for '{spoken_text}': {best_option} (score: {best_score:.2f})")
            return best_option
        rospy.logwarn(f"⚠️ No confident match for '{spoken_text}', best was '{best_option}' ({best_score:.2f})")
        return None

    def ask_question(self, question, question_type):
        if question:  # Only speak if there's actually a question
            self.tts_pub.publish(question)
            rospy.loginfo(f"🤖 Asking: {question}")
            rospy.sleep(0.23 * len(question.split()))

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)
            try:
                rospy.loginfo("🎧 Listening for response...")
                audio = self.recognizer.listen(source, timeout=5)

                # Recognize speech
                if self.is_online():
                    rospy.loginfo("🌐 Using Google Speech Recognition (online)")
                    spoken_text = self.recognizer.recognize_google(audio).strip()

                elif self.vosk_model:
                    rospy.loginfo("📴 No internet. Using Vosk (offline)")
                    rec = KaldiRecognizer(self.vosk_model, source.SAMPLE_RATE)
                    rec.AcceptWaveform(audio.get_raw_data())
                    result = jsonlib.loads(rec.Result())
                    spoken_text = result.get("text", "").strip()

                else:
                    rospy.logerr("❌ No internet and no offline model available")
                    return None

                if not spoken_text:
                    return None

                # Apply fuzzy matching
                if question_type == "name":
                    return self.best_match(spoken_text, self.known_names)
                elif question_type == "drink":
                    return self.best_match(spoken_text, self.known_drinks)
                else:
                    return spoken_text

            except sr.WaitTimeoutError:
                rospy.logwarn("⏱️ Timeout: No speech detected.")
                return None
            except sr.UnknownValueError:
                rospy.logwarn("🤷 Couldn't understand audio.")
                return None
            except sr.RequestError as e:
                rospy.logerr(f"Speech recognition request error: {e}")
                return None


if __name__ == "__main__":
    try:
        node = VoiceNameDrinkCollectorNode()
        rospy.loginfo("🎤 Name and Drink Collector ready")
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError):
        pass

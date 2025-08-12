#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import signal
import os

class VoiceNameRecognizerNode:
    def __init__(self):
        rospy.init_node('voice_name_recognizer_node')
        self.name_pub = rospy.Publisher('/customer_name', String, queue_size=10)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
        self.shutdown_service = rospy.Service("shut_name", Trigger, self.name_shutdown)

        self.recognizer = sr.Recognizer()
        self.shutdown_requested = False  # Flag to track shutdown state

        # Microphone setup - improved selection
        mic_list = sr.Microphone.list_microphone_names()
        if not mic_list:
            rospy.logerr("❌ No microphone detected.")
            raise RuntimeError("No microphone found.")

        rospy.loginfo(f"🎙️ Detected microphones: {mic_list}")

        mic_index = None
        for idx, name in enumerate(mic_list):
            if "analog" in name.lower() or "input" in name.lower():
                mic_index = idx
                break
        if mic_index is None:
            mic_index = 0  # Fallback to first device
            rospy.logwarn("⚠️ No preferred microphone found, using first device")

        try:
            self.mic = sr.Microphone(device_index=mic_index)
            rospy.loginfo(f"🎤 Using microphone: {mic_list[mic_index]} (index: {mic_index})")
        except OSError as e:
            rospy.logerr(f"❌ Microphone error: {e}")
            raise

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        rospy.loginfo("Received termination signal. Shutting down...")
        self.shutdown_requested = True
        rospy.signal_shutdown("Termination signal received")

    def ask_name(self):
        question = "Hi! My name is AIROST. What is your name?"
        self.tts_pub.publish(question)
        rospy.loginfo(f"🤖 Asking: {question}")

        # Estimate TTS duration
        wait_time = 0.3 * len(question.split())
        rospy.sleep(wait_time)

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            while not self.shutdown_requested and not rospy.is_shutdown():
                try:
                    rospy.loginfo("🎧 Listening for name...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                    name = self.recognizer.recognize_google(audio)
                    rospy.loginfo(f"🗣️ Customer name: {name}")
                    self.name_pub.publish(name)
                    return  # Exit after publishing name

                except sr.WaitTimeoutError:
                    rospy.logwarn("⏱️ Timeout: No speech detected.")
                    if not self.shutdown_requested:
                        self.tts_pub.publish("I didn't hear anything. Please say your name.")

                except sr.UnknownValueError:
                    rospy.logwarn("🤷 Couldn't understand audio.")
                    if not self.shutdown_requested:
                        self.tts_pub.publish("Sorry, I didn't understand your name. Please try again.")

                except sr.RequestError as e:
                    rospy.logerr(f"🌐 Speech recognition error: {e}")
                    if not self.shutdown_requested:
                        self.tts_pub.publish("There was a problem with speech recognition.")
                    return  # Exit on network error

    def name_shutdown(self, req):
        rospy.loginfo("Service-triggered shutdown initiated")
        self.shutdown_requested = True
        rospy.signal_shutdown("Service triggered shutdown")
        return TriggerResponse(success=True)

if __name__ == "__main__":
    try:
        node = VoiceNameRecognizerNode()
        node.ask_name()
        rospy.spin()  # Keep node alive until shutdown
    except (rospy.ROSInterruptException, RuntimeError) as e:
        rospy.loginfo(f"Node terminated: {str(e)}")

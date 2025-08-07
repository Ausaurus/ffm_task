#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String
from std_srvs.srv import *

class VoiceNameRecognizerNode:
    def __init__(self):
        rospy.init_node('voice_name_recognizer_node')
        self.name_pub = rospy.Publisher('/customer_name', String, queue_size=10)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
        rospy.Service("shut_name", Trigger, self.name_shutdown)

        self.recognizer = sr.Recognizer()

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

    def ask_name(self):
        question = "Hi! My name is AIROST. What is your name?"
        self.tts_pub.publish(question)
        rospy.loginfo(f"🤖 Asking: {question}")

        # Estimate TTS duration: 0.5 seconds per word
        wait_time = 0.3 * len(question.split())
        rospy.sleep(wait_time)

        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)
            while not rospy.is_shutdown():
                try:
                    rospy.loginfo("🎧 Listening for name...")
                    audio = self.recognizer.listen(source, timeout=3)
                    name = self.recognizer.recognize_google(audio)
                    rospy.loginfo(f"🗣️ Customer name: {name}")
                    self.name_pub.publish(name)
                    break

                except sr.WaitTimeoutError:
                    rospy.logwarn("⏱️ Timeout: No speech detected.")
                    self.tts_pub.publish("I didn't hear anything. Please say your name.")

                except sr.UnknownValueError:
                    rospy.logwarn("🤷 Couldn't understand audio.")
                    self.tts_pub.publish("Sorry, I didn't understand your name. Please try again.")

                except sr.RequestError as e:
                    rospy.logerr(f"🌐 Speech recognition error: {e}")
                    self.tts_pub.publish("There was a problem with speech recognition.")
                    break

    def name_shutdown(self):
        rospy.signal_shutdown("avoid running two times")
        return TriggerResponse(success=True)

if __name__ == "__main__":
    try:
        node = VoiceNameRecognizerNode()
        node.ask_name()
    except (rospy.ROSInterruptException, RuntimeError):
        pass


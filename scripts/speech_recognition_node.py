#!/home/i_h8_ros/catkin_ws/.py3.10/bin/python3

import rospy
import socket
import os
import time
import pyaudio
import pygame
import speech_recognition as sr
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import numpy as np

from tts_utils import wait_for_tts, init_tts_subscriber, play_audio, init_mixer
from config import VOSK_MODEL_PATH, BASE_PATH

# Initialize Vosk model for offline recognition
vosk_model = None
if os.path.exists(VOSK_MODEL_PATH):
    vosk_model = Model(VOSK_MODEL_PATH)
    vosk_recognizer = KaldiRecognizer(vosk_model, 16000)

# Initialize Pygame for audio playback
pygame.mixer.init()

# # Initialize Pygame for audio playback
# try:
#     pygame.mixer.init()
# except pygame.error as e:
#     rospy.logerr(f"Failed to initialize pygame mixer: {e}")
#     # Try reinitializing after a small delay
#     rospy.sleep(1)
#     pygame.mixer.quit()
#     pygame.mixer.init()

def play_beep():
    init_mixer()  # Ensure mixer is initialized before playing
    file_path = os.path.join(BASE_PATH, 'audio/ding.wav')  # Corrected the path
    if os.path.exists(file_path):
        rospy.loginfo(f"Playing beep sound: {file_path}")
        try:
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():  # Wait for the beep sound to finish
                time.sleep(0.1)
        except pygame.error as e:
            rospy.logerr(f"Error playing beep sound: {e}")
    else:
        rospy.logwarn(f"Beep sound file {file_path} not found.")

def handle_speech_recognition(req):
    recognizer = sr.Recognizer()
    mic = None
    stream = None
    retries = 0
    max_retries = 3  # Maximum number of retries

    # Check for internet connectivity
    internet_available = check_internet_connection()

    # Initialize the microphone stream for Vosk
    if not internet_available and vosk_model:
        rospy.logwarn("No internet connection. Switching to offline Vosk model.")
        mic = pyaudio.PyAudio()
        try:
            stream = mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=16384)
            stream.start_stream()
        except OSError as e:
            rospy.logerr(f"Failed to open the microphone stream: {e}")
            pub_speak.publish("Failed to open the microphone stream. Please check your microphone settings.")
            wait_for_tts()
            return TriggerResponse(success=False, message="Failed to open the microphone stream.")

    try:
        while not rospy.is_shutdown() and retries < max_retries:
            if internet_available:
                with sr.Microphone() as source:
                    rospy.loginfo("SR Node: Adjusting for ambient noise...")
                    recognizer.adjust_for_ambient_noise(source, duration=2)
                    rospy.sleep(1)
                    # Play a beep sound to indicate the guest can start speaking
                    play_beep()
                    rospy.loginfo("SR Node: Listening...")
                    audio = recognizer.listen(source, timeout=20, phrase_time_limit=10)
                    try:
                        text = recognizer.recognize_google(audio)
                        rospy.loginfo(f"SR Node: Recognized - {text}")
                        pub.publish(text)
                        return TriggerResponse(success=True, message=text)
                    except sr.WaitTimeoutError:
                        rospy.loginfo("SR Node: Listening timed out. Please try again.")
                        play_audio("try_again.wav", BASE_PATH)
                        rospy.sleep(2)
                        retries += 1
                    except sr.UnknownValueError:
                        rospy.loginfo("SR Node: Sorry, I did not get that. Please try again.")
                        play_audio("try_again.wav", BASE_PATH)
                        rospy.sleep(2)
                        retries += 1
                    except sr.RequestError as e:
                        # audio file
                        rospy.loginfo(f"SR Node: Could not request results; {e}")
                        pub_speak.publish("There was an error with the speech recognition service. Please try again later.")
                        wait_for_tts()
                        return TriggerResponse(success=False, message=f"Request error: {e}")
            else:
                # Handle offline Vosk model here
                rospy.logwarn("SR Node: Switching to offline Vosk model.")
                if vosk_model:
                    result = handle_vosk_speech_recognition(stream)
                    if result.success:
                        return result
                    else:
                        retries += 1
                else:
                    rospy.logerr("Vosk model not available.")
                    pub_speak.publish("Sorry, I could not process your speech and the offline model is not available.")
                    wait_for_tts()
                    return TriggerResponse(success=False, message="Offline model not available.")

        # After retries exceed the limit, assign "Unknown" as the default name
        rospy.loginfo("Max retries exceeded, assigning 'Unknown' as guest name.")
        pub_speak.publish("I did not hear your name after multiple tries, skipping the name asking process.")
        wait_for_tts()
        pub.publish("Unknown")
        return TriggerResponse(success=True, message="Unknown")

    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        return TriggerResponse(success=False, message=str(e))
    finally:
        if stream:
            stream.stop_stream()
            stream.close()
        if mic:
            mic.terminate()
    # return TriggerResponse(success=False, message="Speech recognition node shutdown.")

def handle_vosk_speech_recognition(stream):
    while not rospy.is_shutdown():
        try:
            data = stream.read(2048, exception_on_overflow=False)
            rospy.loginfo(f"Raw audio data length: {len(data)}")  # Log the length of the data
        except IOError:
            rospy.loginfo("Overflow occurred, skipping this frame")
            continue

        if vosk_recognizer.AcceptWaveform(data):
            result = vosk_recognizer.Result()
            text = result[14:-3].strip().lower()  # Strip and lower the recognized text
            
            if text:  # If text is not empty
                rospy.loginfo(f"Vosk SR Node: Recognized - {text}")
                pub.publish(text)
                return TriggerResponse(success=True, message=text)
            else:
                rospy.loginfo("Vosk SR Node: No speech detected, retrying...")
                play_audio("try_again.wav", BASE_PATH)

    return TriggerResponse(success=False, message="Vosk recognition node shutdown.")

def check_internet_connection():
    try:
        # Try to connect to a common DNS server to see if the internet is available
        socket.create_connection(("8.8.8.8", 53), timeout=3)
        return True
    except OSError:
        return False

def main():
    global pub, pub_speak

    rospy.init_node('speech_recognition_node', anonymous=True)
    pub = rospy.Publisher('recognized_speech', String, queue_size=10)
    pub_speak = rospy.Publisher('speech_to_speak', String, queue_size=10)
    init_tts_subscriber()  # Initialize TTS subscriber from the utility module

    srv = rospy.Service('start_speech_recognition', Trigger, handle_speech_recognition)
    rospy.loginfo("Speech Recognition Node is ready to receive requests.")
    
    rospy.spin()

if __name__ == "__main__":
    main()

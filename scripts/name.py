#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String
import socket
import difflib
import threading

# --- OFFLINE STT (Vosk) ---
try:
    from vosk import Model, KaldiRecognizer
    import json
    VOSK_AVAILABLE = True
except ImportError:
    VOSK_AVAILABLE = False
    rospy.logwarn("⚠️ Vosk not installed. Offline mode will be disabled.")

class VoiceNameRecognizerNode:
    def __init__(self):
        rospy.init_node('voice_name_recognizer_node')
        self.name_pub = rospy.Publisher('/customer_name', String, queue_size=10)
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)

        # Subscribe to the trigger topic
        self.name_what_sub = rospy.Subscriber('/name_what', String, self.name_what_callback)

        self.recognizer = sr.Recognizer()
        self.is_listening = False
        self.listen_lock = threading.Lock()

        # Known names list (case insensitive matching)
        self.known_names = [
            "liam", "noah", "oliver", "laura", "james", "william",
            "benjamin", "lucas", "henry", "alexander",
            "emma", "olivia", "ava", "sophia", "isabella"
        ]

        # Check internet availability
        self.online_mode = self.check_internet()
        rospy.loginfo(f"🌐 Online mode: {self.online_mode}")

        # Setup Vosk offline model if available
        if VOSK_AVAILABLE:
            try:
                self.vosk_model = Model("/home/i_h8_ros/ffm_ws/src/ffm_task/scripts/vosk-model-small-en-us-0.15")
                self.vosk_recognizer = KaldiRecognizer(self.vosk_model, 16000)
                rospy.loginfo("✅ Vosk offline model loaded.")
            except Exception as e:
                rospy.logwarn(f"⚠️ Could not load Vosk model: {e}")
                self.vosk_model = None
        else:
            self.vosk_model = None

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

        rospy.loginfo("🚀 Voice Name Recognizer Node is ready. Waiting for /name_what messages...")

    def check_internet(self, host="8.8.8.8", port=53, timeout=3):
        """Check internet connection"""
        try:
            socket.setdefaulttimeout(timeout)
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
            return True
        except socket.error:
            return False

    def name_what_callback(self, msg):
        """Callback triggered when receiving a message on /name_what topic"""
        rospy.loginfo(f"📨 Received trigger message: {msg.data}")

        with self.listen_lock:
            if self.is_listening:
                rospy.loginfo("🔄 Already listening for name, ignoring new request")
                return

            self.is_listening = True

        # Start name recognition in a separate thread to avoid blocking
        threading.Thread(target=self.ask_name, daemon=True).start()

    def find_closest_name(self, recognized_text):
        """
        Find the closest matching name from the known names list.
        Returns (matched_name, confidence_score) or (None, 0) if no good match.
        """
        if not recognized_text:
            return None, 0

        recognized_text = recognized_text.lower().strip()
        words = recognized_text.split()

        best_match = None
        best_score = 0

        # Check each word in the recognized text
        for word in words:
            # First check for exact matches
            if word in self.known_names:
                return word.capitalize(), 1.0

            # Then check for close matches using difflib
            matches = difflib.get_close_matches(word, self.known_names, n=1, cutoff=0.6)
            if matches:
                score = difflib.SequenceMatcher(None, word, matches[0]).ratio()
                if score > best_score:
                    best_match = matches[0].capitalize()
                    best_score = score

        # Also check if the entire recognized text is close to any name
        matches = difflib.get_close_matches(recognized_text, self.known_names, n=1, cutoff=0.6)
        if matches:
            score = difflib.SequenceMatcher(None, recognized_text, matches[0]).ratio()
            if score > best_score:
                best_match = matches[0].capitalize()
                best_score = score

        # Only return matches with reasonable confidence
        if best_score >= 0.3:
            return best_match, best_score
        else:
            return None, 0

    def ask_name(self):
        """Ask for the customer's name and process the response"""
        try:
            question = "Hi! My name is AIROST. What is your name?"
            self.tts_pub.publish(question)
            rospy.loginfo(f"🤖 Asking: {question}")

            rospy.sleep(0.3 * len(question.split()))  # Wait for TTS

            with self.mic as source:
                self.recognizer.adjust_for_ambient_noise(source)
                max_attempts = 3
                attempt = 0

                while attempt < max_attempts and not rospy.is_shutdown():
                    try:
                        rospy.loginfo(f"🎧 Listening for name (attempt {attempt + 1}/{max_attempts})...")
                        audio = self.recognizer.listen(source, timeout=5)
                        recognized_text = None

                        # --- Try ONLINE recognition first ---
                        if self.online_mode:
                            try:
                                recognized_text = self.recognizer.recognize_google(audio)
                                rospy.loginfo(f"🗣️ Raw recognition (Google): {recognized_text}")
                            except sr.RequestError:
                                rospy.logwarn("🌐 Google STT failed, switching to offline mode.")
                                self.online_mode = False

                        # --- Offline fallback with Vosk ---
                        if not recognized_text and self.vosk_model:
                            import wave
                            wav_data = audio.get_wav_data()
                            self.vosk_recognizer.AcceptWaveform(wav_data)
                            result = json.loads(self.vosk_recognizer.Result())
                            recognized_text = result.get("text", "").strip()
                            rospy.loginfo(f"🗣️ Raw recognition (Vosk): {recognized_text}")

                        # --- Process the recognized text ---
                        if recognized_text:
                            matched_name, confidence = self.find_closest_name(recognized_text)

                            if matched_name:
                                rospy.loginfo(f"✅ Matched name: {matched_name} (confidence: {confidence:.2f})")
                                self.name_pub.publish(matched_name)

                                # Confirm the recognized name
                                confirm_msg = f"Hi, {matched_name}!"
                                self.tts_pub.publish(confirm_msg)
                                rospy.loginfo(f"🤖 Confirmed: {confirm_msg}")
                                break
                            else:
                                rospy.logwarn(f"❌ No matching name found for: '{recognized_text}'")
                                retry_msg = "Sorry, I didn't catch that. Please repeat your name loudly."
                                self.tts_pub.publish(retry_msg)
                        else:
                            rospy.logwarn("🤷 No text recognized from audio.")
                            self.tts_pub.publish("Sorry, I couldn't hear you. Please repeat loudly.")

                    except sr.WaitTimeoutError:
                        rospy.logwarn("⏱️ Timeout: No speech detected.")
                        self.tts_pub.publish("I'm still listening. Please tell me your name.")

                    except sr.UnknownValueError:
                        rospy.logwarn("🤷 Couldn't understand audio.")
                        self.tts_pub.publish("Sorry, I didn't understand. Please repeat your name clearly.")

                    attempt += 1

                    # If this wasn't the last attempt, wait a bit before trying again
                    if attempt < max_attempts:
                        rospy.sleep(1)

                # If we've exhausted all attempts
                if attempt >= max_attempts:
                    rospy.logwarn("❌ Maximum attempts reached without successful name recognition")
                    self.tts_pub.publish("Sorry, I'm having trouble hearing your name. Please try again later.")

        except Exception as e:
            rospy.logerr(f"❌ Error in ask_name: {e}")
        finally:
            # Reset the listening flag
            with self.listen_lock:
                self.is_listening = False
            rospy.loginfo("🔄 Ready to listen for next /name_what trigger")

    def run(self):
        """Main run loop - just spin and wait for messages"""
        rospy.spin()

if __name__ == "__main__":
    try:
        node = VoiceNameRecognizerNode()
        node.run()
    except (rospy.ROSInterruptException, RuntimeError) as e:
        rospy.loginfo(f"Node shutting down: {e}")

#!/usr/bin/env python3

import roslaunch
import rospy
import os
import subprocess
import json
import requests
from std_msgs.msg import String
import speech_recognition as sr
from gtts import gTTS
import time
import rospkg
# Initialize the recognizer and the microphone
recognizer = sr.Recognizer()
mic = sr.Microphone()

is_speaking = False 
# This dictionary will store timestamps: {'phone_model': last_announced_time}
last_announced = {}

# Global variable to hold latest detection
latest_detection_data = None
# Set to store phones that have already been announced
# announced_phones = set() 
last_recognized_phone = None
announcement_made = False
detection_process = None
API_URL = "https://jgftptvusufhzbk3eenu2el3ja0igvvg.lambda-url.ap-southeast-1.on.aws/ask"

# Function to speak the text using Google TTS
def speak(text, language='en'):
    try:
        tts = gTTS(text=text, lang=language)
        tts.save("/tmp/temp_audio.mp3")
        os.system("mpg321 /tmp/temp_audio.mp3")
    except Exception as e:
        rospy.logerr(f"Error in text-to-speech: {e}")

# Function to listen for and return recognized speech
def listen_and_recognize(prompt):
    with mic as source:
        rospy.loginfo(prompt)
        recognizer.adjust_for_ambient_noise(source)
        try:
            audio = recognizer.listen(source, timeout=3)
            recognized_text = recognizer.recognize_google(audio).lower()
            rospy.loginfo(f"Recognized: {recognized_text}")
            return recognized_text
        except (sr.UnknownValueError, sr.WaitTimeoutError):
            return None
        except Exception as e:
            rospy.logerr(f"Recognition error: {e}")
            return None
        
# Send query to API
def query_api(command_text):
    try:
        payload = {"query": command_text}
        headers = {'Content-Type': 'application/json'}
        response = requests.post(API_URL, json=payload, headers=headers, timeout=5)

        if response.status_code == 200:
            data = response.json()
            return data.get("answer", "Sorry, the server doesn't respond. Can you try again?")
        else:
            rospy.logwarn(f"API error: {response.status_code} - {response.text}")
            return "Sorry, I couldn't get a valid response from the server. Please try again."
    except Exception as e:
        rospy.logerr(f"API request failed: {e}")
        return "There was a problem connecting to the server. Please try again."

# Start detection nodes
def start_detection_nodes():
    global detection_process
    if detection_process is not None:
        rospy.loginfo("Detection is already running.")
        return

    try:
        rospy.loginfo("Starting usb_cam and detection nodes...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        # Correct way to find the package path
        rospack_obj = rospkg.RosPack()
        pkg_path = rospack_obj.get_path("robot_project_pkg")
        
        launch_file = os.path.join(pkg_path, 'launch', 'detection_pipeline.launch')
        
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        launch.start()
        
        detection_process = launch
        rospy.loginfo("Detection nodes started successfully.")

    except rospkg.ResourceNotFound:
        rospy.logerr("Error: 'robot_project_pkg' not found. Have you sourced your workspace?")
    except Exception as e:
        rospy.logerr(f"Error starting detection nodes: {e}")
        detection_process = None
        
# Stop detection nodes by calling external stop_detection.py
def stop_detection_nodes():
    global detection_process
    if detection_process is None:
        rospy.loginfo("Detection is not running.")
        return

    try:
        rospy.loginfo("Stopping camera and detection nodes...")
        detection_process.shutdown()
        detection_process = None
        rospy.loginfo("Detection nodes stopped successfully.")
    except Exception as e:
        rospy.logerr(f"Failed to stop detection nodes cleanly: {e}")


# Callback to capture latest detection
def detection_callback(msg):
    global is_speaking, last_announced, last_recognized_phone, announcement_made

    # 1. NEW: If an announcement has already been made, do nothing.
    if announcement_made:
        return
        
    if is_speaking:
        return

    try:
        data = json.loads(msg.data)
        if not data:
            return

        for detection in data:
            phone_model = detection.get('class_name')

            if phone_model and phone_model != "unknown":
                current_time = time.time()
                last_time_this_model_was_announced = last_announced.get(phone_model, 0)

                if (current_time - last_time_this_model_was_announced) > 5:
                    if is_speaking:
                        continue
                    
                    is_speaking = True
                    
                    try:
                        speak("I see a phone, please give me a moment.", 'en')
                        # prompt = f"The user is holding a {phone_model}. Act as a helpful product expert and give them an engaging summary of its key features and what makes it special."
                        prompt = (f"This is a brand new request. Ignore any previous context or conversation history. "
                        f"The user is showing you a '{phone_model}'. "
                        f"Act as a helpful product expert and give them an engaging summary of its key features.")
                        phone_details = query_api(prompt)
                        final_response = f"Ah, that looks like a {phone_model}. {phone_details}"
                        speak(final_response, 'en')
                        
                        last_announced[phone_model] = time.time()
                        last_recognized_phone = phone_model

                        # 2. NEW: Set the flag to True to block future announcements.
                        announcement_made = True
                        speak("What would you like to know next?", 'en')
                    finally:
                        is_speaking = False
                    
                    # Break after the first announcement
                    break
    except Exception as e:
        rospy.logerr(f"Error in detection_callback: {e}")
        is_speaking = False

# Main function

def main():
    global is_speaking, last_announced, last_recognized_phone, announcement_made
    
    rospy.init_node('speech_to_text_node', anonymous=True)
    phone_rec_pub = rospy.Publisher("/phone_recommendation", String, queue_size=10)
    rospy.Subscriber("/phone_detections", String, detection_callback)
    rate = rospy.Rate(0.5) # Increased rate for better responsiveness

    rospy.loginfo("Voice assistant ready. Say 'Hey Alex' to begin a session.")

    # --- OUTER LOOP: Waits only for 'Hey Alex' to start a session ---
    while not rospy.is_shutdown():
        
        # Check if an automatic detection is happening before trying to listen
        if is_speaking:
            rate.sleep()
            continue

        wake = listen_and_recognize("Listening for wake word 'Hey Alex'...")

        if wake and "alex" in wake:
            
            # --- SESSION STARTED ---
            rospy.loginfo("Session started by user.")
            
            # Give a short, one-time greeting
            is_speaking = True
            speak("Hi! How can I help you? I'm here to help you with phone recommendation or even phone model recognition. " \
                    "For phone recommendation, just tell me your requirement and I will advice you. For phone model recognition, I will start scanning" \
                    "after you trigger me with words \"start detection\", to stop it, end it with words \"stop detection\"," \
                    "after your first phone model recognition, trigger me with \"next detection\" ", 'en')
           
            is_speaking = False

            while not rospy.is_shutdown():
                
                # If an automatic detection starts speaking, wait for it to finish
                if is_speaking:
                    rate.sleep()
                    continue

                command = listen_and_recognize("Listening for command...")

                if command:
                    # First, check for the command to end the session
                    if "bye alex" in command:
                        rospy.loginfo("Session ended by user.")
                        is_speaking = True
                        speak("Goodbye!", 'en')
                        is_speaking = False
                        break

                    # --- Process all other commands within the session ---
                    is_speaking = True
                    try:
                        if "start detection" in command:
                            start_detection_nodes()
                            speak("Starting detection...", 'en')

                        elif "stop detection" in command:
                            stop_detection_nodes()
                            speak("Stopping detection...", 'en')

                        elif "next detection" in command or "forget phone" in command:
                            last_announced.clear()
                            last_recognized_phone = None
                            announcement_made = False
                            speak("Okay, I'm ready to identify the next new phone you show me.", 'en')
                        
                        else: # Handle all other verbal questions
                            phone_rec_msg = String()
                            if last_recognized_phone is None:
                                phone_rec_msg.data = "&&&" + command
                            else:
                                phone_rec_msg.data = last_recognized_phone + "&&&" + command
                            phone_rec_pub.publish(phone_rec_msg)
                            #prompt = ""
                            #if last_recognized_phone:
                            #    prompt = (f"The user's last point of interest was the '{last_recognized_phone}'. "
                            #              f"They have now asked: '{command}'. "
                            #              f"If this new question is a follow-up about the phone, answer it in that context. "
                            #              f"If it is a new, unrelated question, ignore the previous phone and answer the question directly.")
                            #else:
                            #    prompt = command
                            
                            #speak("Okay, give me a moment to look that up.", 'en')
                            #response = query_api(prompt)
                            #speak(response, 'en')
                    finally:
                        is_speaking = False # Release the lock after the command is done
            
            # After breaking from the inner loop, the outer loop will resume,
            # waiting for "Hey Alex" to start a new session.
            rospy.loginfo("Session ended. Listening for wake word 'Hey Alex'...")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

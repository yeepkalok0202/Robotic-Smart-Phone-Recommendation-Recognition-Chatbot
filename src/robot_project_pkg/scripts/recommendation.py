#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import requests
from gtts import gTTS
import os

API_URL = "https://jgftptvusufhzbk3eenu2el3ja0igvvg.lambda-url.ap-southeast-1.on.aws/ask"

# Function to speak the text using Google TTS
def speak(text, language='en'):
    try:
        tts = gTTS(text=text, lang=language)
        tts.save("/tmp/temp_audio.mp3")
        os.system("mpg321 /tmp/temp_audio.mp3")
    except Exception as e:
        rospy.logerr(f"Error in text-to-speech: {e}")

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

def callback_receive_phone_recommendation_command(msg):
    params = msg.data.split("&&&")
    last_recognized_phone = params[0]
    command = params[1]
    prompt = ""
    if last_recognized_phone != "":
        prompt = (f"The user's last point of interest was the '{last_recognized_phone}'. "
            f"They have now asked: '{command}'. "
            f"If this new question is a follow-up about the phone, answer it in that context. "
            f"If it is a new, unrelated question, ignore the previous phone and answer the question directly.")
    else:
        prompt = command
    
    speak("Okay, give me a moment to look that up.", 'en')
    response = query_api(prompt)
    speak(response, 'en')

def main():
    rospy.loginfo("Started phone recommendation node...")
    rospy.init_node('phone_recommendation_node')
    sub = rospy.Subscriber("/phone_recommendation", String, callback_receive_phone_recommendation_command)

    rospy.spin()

if __name__ == '__main__':
    main()

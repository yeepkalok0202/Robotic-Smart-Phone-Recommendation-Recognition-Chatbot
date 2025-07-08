# 🤖 Jupiter Robot – Smart Phone Recognition and Recommendation Assistant

A ROS-based Smart Phone Recognition and Recommendation Assistant that combines speech-to-text (STT) with RAG implementation, wake-word activation, and camera-based phone detection into a single intelligent agent. Triggered by the wake word “Hey Alex,” it supports voice-based phone model recognition and personalized recommendations. The system uses YOLOv8 for phone detection, ResNet34 for classification, and the Google Gemini AI API to generate feature explanations and tailored recommendation.  

---

## System Overview
The Smart Phone Recognition and Recommendation Assistant is an integrated ROS-based system designed to provide interactive assistance through natural voice commands. The system combines real-time object detection, deep learning classification, and AI-powered dialogue generation to recognize smartphone models and offer tailored recommendations or feature descriptions. The interaction begins with a wake-word activation (“Hey Alex”), initiating a session where users can either show a phone to the camera for recognition or ask for phone suggestions based on their needs. The system seamlessly coordinates between speech processing, visual detection, classification, and API-based response generation to deliver a smooth and intelligent user experience.

**Key Features**
- **Wake Word Activation**: Starts the session when “Hey Alex” is spoken.
- **Voice Command Interface**: Converts user speech to actionable commands.
- **Real-Time Phone Detection**: Uses **YOLOv8** to detect phones from the camera feed.
- **Model Classification**: Employs **ResNet34** to classify the detected phone model.
- **AI-Powered Explanation**: Sends user commands to **Google Gemini AI API** to generate phone summaries or recommendations.
- **Modular ROS Integration**: Ensures synchronized communication between speech, vision, and AI modules.
- **Visual Feedback**: Publishes annotated images with detection results for user confirmation.


## Development Environment
- **Operating System**: Ubuntu 20.04 LTS  
- **Robot Operating System (ROS)**: ROS Noetic 
- **Programming Language**: Python ≥ 3.8.0  
- **Environment**: Virtual environment using `venv`  
- **IDE**: Visual Studio Code  

## 🛠️ Environment Setup 
**1. Create a New ROS Workspace**
```bash
mkdir -p ~/my_project_ws/src
cd ~/my_project_ws/src
catkin_init_workspace
```

**2. Build the empty workspace once**
```bash
cd ~/my_project_ws
catkin_make
```

**3. Create a new ROS package**

Navigate to src/ and create your package:
```bash
cd ~/my_project_ws/src
catkin_create_pkg my_robot_pkg rospy std_msgs
```

**4. Build the workspace again**
```bash
cd ~/my_project_ws
catkin_make
```

**5. Source the workspace setup file**
```bash
source ~/my_project_ws/devel/setup.bash
```

## 🛠️ Project Setup
**1.Clone the repository**
```bash
cd ~
git clone https://github.com/yeepkalok0202/Robotic-Smart-Phone-Recommendation-Recognition-Chatbot.git robot_project
cd robot_project
```

**2. Build the workspace**
```bash
rm -rf build/ devel/
catkin_make
```

**3. Set up the python environment**
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
deactivate
```

**4. Source and run**
```bash
source devel/setup.bash
```

## ⚙️ Running the Complete Agent ##

```bash
roslaunch robot_project_pkg phone_agent.launch
```

## 📌 Usage

1. Start the session by saying the wake word: **"Hey Alex"**.

2. Provide voice commands such as:
   - "Start detection"
   - "Stop detection"
   - "Next detection"
   - "Recommend a Oppo phone with budget 3000 ringgit."
   - "Recommend a Oppo with good battery life."

3. For phone recognition:
   - Display the phone image in front of the camera.
   - Wait for the assistant to detect and respond with the phone model and its key features.

4. For phone recommendation:
   - Verbally describe your requirements (e.g., budget, battery).
   - The assistant will respond with a suitable phone recommendation based on the criteria.


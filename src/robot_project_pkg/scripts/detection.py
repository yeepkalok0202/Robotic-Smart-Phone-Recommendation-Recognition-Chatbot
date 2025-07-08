#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO
import json
from classification import predict, load_model
from PIL import Image as PILImage
# ROS Message Imports
from sensor_msgs.msg import Image
from std_msgs.msg import String

# To convert ROS Images to OpenCV
from cv_bridge import CvBridge, CvBridgeError

WORD_DIR = "/home/mustar/robot_project/src/robot_project_pkg/data"

class PhoneDetector:
    def __init__(self):
        # --- Class Members ---
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Load the YOLOv8 model
        self.classifier = load_model(f"{WORD_DIR}/model/best_model_v2.pt") # Load the classifier model

        # ROS topic names
        image_topic = "/usb_cam/image_raw"
        self.detection_pub = rospy.Publisher("/phone_detections", String, queue_size=10)
        self.annotated_image_pub = rospy.Publisher("/phone_detections/image_annotated", Image, queue_size=10)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)

        rospy.loginfo("Phone Detector node started. Listening on: %s", image_topic)

   
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return
        
        annotated_frame = cv_image.copy()
        results = self.model(cv_image, classes=[67], verbose=False)
        detections = []


        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            crop = cv_image[y1:y2, x1:x2]

            if crop.size == 0:
                continue

            try:
                phone_crop_pil = PILImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                phone_model_name = predict(phone_crop_pil, self.classifier)
            except Exception as e:
                rospy.logwarn(f"Classification failed: {e}")
                phone_model_name= "unknown"


            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (36, 255, 12), 2) 
            cv2.putText(annotated_frame, phone_model_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,0.6,(36, 255, 12),2)


            # Add detection
            detections.append({
                'class_name': phone_model_name
            })


        # Publish annotated image
        try:
            annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.annotated_image_pub.publish(annotated_image_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

        # Publish detection results
        if detections:
            json_output = json.dumps(detections)
            self.detection_pub.publish(json_output)

def main():
    rospy.init_node('phone_detector_node', anonymous=True)
    detector = PhoneDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Phone Detector node.")

if __name__ == '__main__':
    main()

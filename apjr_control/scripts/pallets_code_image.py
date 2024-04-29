#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from roboflow import Roboflow
import supervision as sv
import cv2
import numpy as np
from std_msgs.msg import String

def main():
    rospy.init_node('pallet_detection_publisher')

    # Initialize CvBridge
    bridge = CvBridge()

    # Publisher for annotated image
    image_pub = rospy.Publisher('/pallet_detection/image', Image, queue_size=10)

    # Publisher for decision
    decision_pub = rospy.Publisher('/final_decision', String, queue_size=10)  # Adjust topic and message type as needed

    # Initialize the decision message
    decision_msg = String()

    # Initialize Roboflow
    rf = Roboflow(api_key="LlIKXYgtTmJZ2CUwv4Hl")
    project = rf.workspace().project("pallet-5esjz")
    model = project.version(1).model

    # create instance of BoxAnnotator
    box_annotator = sv.BoxAnnotator(thickness=4, text_thickness=4, text_scale=2)

    # Read the image
    frame = cv2.imread("/home/ali/Pallets_Grad_Data/pallet_image1.jpg")  # Path to your image

    # Predict on the image
    results = model.predict(frame).json()
    detections = sv.Detections.from_roboflow(results)

    # Find indices of "front" and "pallet" classes
    front_indices = np.where(detections.data['class_name'] == 'front')[0]
    pallet_indices = np.where(detections.data['class_name'] == 'pallet')[0]

    # Find the index with the highest confidence for each class
    highest_front_index = np.argmax(detections.confidence[front_indices])
    highest_pallet_index = np.argmax(detections.confidence[pallet_indices])

    # Get the detection data for the highest confidence detections
    if front_indices.size > 0:
        front_confidence = detections.confidence[front_indices[highest_front_index]]
        front_bbox = detections.xyxy[front_indices[highest_front_index]]
        front_label = "front"
        front_width = front_bbox[2] - front_bbox[0]
        front_height = front_bbox[3] - front_bbox[1]
    else:
        front_confidence = 0
        front_bbox = None
        front_label = None
        front_width = 0
        front_height = 0

    if pallet_indices.size > 0:
        pallet_confidence = detections.confidence[pallet_indices[highest_pallet_index]]
        pallet_bbox = detections.xyxy[pallet_indices[highest_pallet_index]]
        pallet_label = "pallet"
        pallet_width = pallet_bbox[2] - pallet_bbox[0]
        pallet_height = pallet_bbox[3] - pallet_bbox[1]
    else:
        pallet_confidence = 0
        pallet_bbox = None
        pallet_label = None
        pallet_width = 0
        pallet_height = 0

    # Draw bounding boxes on the image for the highest confidence detections
    if front_bbox is not None:
        cv2.rectangle(frame, (int(front_bbox[0]), int(front_bbox[1])), (int(front_bbox[2]), int(front_bbox[3])), (0, 255, 0), 2)
        cv2.putText(frame, f'{front_label} {front_confidence:.2f}', (int(front_bbox[0]), int(front_bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if pallet_bbox is not None:
        cv2.rectangle(frame, (int(pallet_bbox[0]), int(pallet_bbox[1])), (int(pallet_bbox[2]), int(pallet_bbox[3])), (0, 0, 255), 2)
        cv2.putText(frame, f'{pallet_label} {pallet_confidence:.2f}', (int(pallet_bbox[0]), int(pallet_bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Convert the frame to ROS Image message
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image_msg = bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")

    # Publish the annotated image
    image_pub.publish(image_msg)

    frame_height, frame_width, _ = frame.shape

    # Calculate left and right widths for centering bounding box
    left_width = front_bbox[0]
    right_width = frame_width - front_bbox[2]

    # Calculate top and bottom heights for centering bounding box
    top_height = pallet_bbox[1]
    bottom_height = frame_height - front_bbox[3]

    # Calculate the difference between left and right widths
    width_difference = abs(left_width - right_width)

    # Define a threshold for considering the pallet centered horizontally
    threshold = 10  # Adjust as needed

    # Determine if the pallet is centered horizontally
    if width_difference <= threshold:
        horizontal_decision = "Centered"
    elif left_width < right_width:
        horizontal_decision = "Go Left"
    else:
        horizontal_decision = "Go Right"

    # Calculate the difference between top and bottom heights
    height_difference = abs(top_height - bottom_height)

    # Define a threshold for considering the pallet centered vertically
    height_threshold = 10  # Adjust as needed

    # Determine if the pallet is centered vertically
    if height_difference <= height_threshold:
        vertical_decision = "Centered"
    elif top_height < bottom_height:
        vertical_decision = "Go Up"
    else:
        vertical_decision = "Go Down"

    # Integrate both width and height decisions
    if horizontal_decision == "Centered" and vertical_decision == "Centered":
        final_decision = "Centered"
    else:
        final_decision = f"{horizontal_decision}, {vertical_decision}"

    # Publish the final decision on the "decision" topic
    decision_pub.publish(final_decision)

    # Print left and right widths and horizontal decision
    print("Left Width:", left_width)
    print("Right Width:", right_width)
    print("Horizontal Decision:", horizontal_decision)

    # Print top and bottom heights and vertical decision
    print("Top Height:", top_height)
    print("Bottom Height:", bottom_height)
    print("Vertical Decision:", vertical_decision)

    # Print final decision
    print("Final Decision:", final_decision)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

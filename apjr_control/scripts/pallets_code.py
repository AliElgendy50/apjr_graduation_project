#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from roboflow import Roboflow
import supervision as sv
import cv2
import numpy as np

def main():
    rospy.init_node('pallet_detection_publisher')

    # Initialize CvBridge
    bridge = CvBridge()

    # Publisher for annotated image
    image_pub = rospy.Publisher('/pallet_detection/image', Image, queue_size=10)

    # Initialize Roboflow
    rf = Roboflow(api_key="LlIKXYgtTmJZ2CUwv4Hl")
    project = rf.workspace().project("pallet-5esjz")
    model = project.version(1).model

    # create instance of BoxAnnotator
    box_annotator = sv.BoxAnnotator(thickness=4, text_thickness=4, text_scale=2)

    # Open a video file or stream
    video_capture = cv2.VideoCapture("/home/ali/Pallets_Grad_Data/pallets3.mp4")  # Corrected file path

    # Check if the video capture is open
    if not video_capture.isOpened():
        print("Error: Unable to open video source.")
        return

    # Loop for processing frames
    rate = rospy.Rate(30)  # Adjust as needed
    while not rospy.is_shutdown():
        # Read a frame from the video
        ret, frame = video_capture.read()
        if not ret:
            print("Error: Unable to read frame.")
            break

        # Predict on the frame
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


        # Print data for the highest confidence detections
        print("Front Detection:")
        if front_bbox is not None:
            print(f"  Confidence: {front_confidence:.2f}")
            print(f"  Bounding Box:")
            print(f"    - Top-Left:     ({front_bbox[0]:.2f}, {front_bbox[1]:.2f})")
            print(f"    - Bottom-Right: ({front_bbox[2]:.2f}, {front_bbox[3]:.2f})")
            print(f"  Width:  {front_width:.2f}")
            print(f"  Height: {front_height:.2f}")
        else:
            print("  No front detection")

        print("Pallet Detection:")
        if pallet_bbox is not None:
            print(f"  Confidence: {pallet_confidence:.2f}")
            print(f"  Bounding Box:")
            print(f"    - Top-Left:     ({pallet_bbox[0]:.2f}, {pallet_bbox[1]:.2f})")
            print(f"    - Bottom-Right: ({pallet_bbox[2]:.2f}, {pallet_bbox[3]:.2f})")
            print(f"  Width:  {pallet_width:.2f}")
            print(f"  Height: {pallet_height:.2f}")
        else:
            print("  No pallet detection")

        # Draw bounding boxes on the frame for the highest confidence detections
        if front_bbox is not None:
            cv2.rectangle(frame, (int(front_bbox[0]), int(front_bbox[1])), (int(front_bbox[2]), int(front_bbox[3])), (0, 255, 0), 2)
            cv2.putText(frame, f'{front_label} {front_confidence:.2f}', (int(front_bbox[0]), int(front_bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if pallet_bbox is not None:
            cv2.rectangle(frame, (int(pallet_bbox[0]), int(pallet_bbox[1])), (int(pallet_bbox[2]), int(pallet_bbox[3])), (0, 0, 255), 2)
            cv2.putText(frame, f'{pallet_label} {pallet_confidence:.2f}', (int(pallet_bbox[0]), int(pallet_bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Convert the frame to ROS Image message
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_msg = bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")

        # Publish the annotated frame
        image_pub.publish(image_msg)

        frame_height, frame_width, _ = frame.shape
        total_frame_width_height = (frame_width, frame_height)
        print("Total Frame Width and Height:", total_frame_width_height)


        # Calculate left and right widths for centering bounding box
        left_width = front_bbox[0]
        right_width = frame_width - front_bbox[2]

        # Print left and right widths
        print("Left Width:", left_width)
        print("Right Width:", right_width)

        

        rate.sleep()

    # Release the video capture
    video_capture.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

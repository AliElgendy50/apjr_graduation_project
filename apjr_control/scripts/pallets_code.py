#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from roboflow import Roboflow
import supervision as sv
import cv2

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

    # Open a video file or stream
    video_capture = cv2.VideoCapture("/home/ali/Pallets_Grad/pallets4.mp4")  # Corrected file path

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
        result = model.predict(frame, confidence=40, overlap=30).json()
        detections = sv.Detections.from_inference(result)
        labels = [item["class"] for item in result["predictions"]]

        # Annotate the frame
        label_annotator = sv.LabelAnnotator()
        bounding_box_annotator = sv.BoundingBoxAnnotator()
        annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections, labels=labels)

        # Convert the annotated frame from BGR to RGB format
        annotated_frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)

        # Convert the annotated frame to ROS Image message
        image_msg = bridge.cv2_to_imgmsg(annotated_frame_rgb, encoding="rgb8")

        # Publish the annotated frame
        image_pub.publish(image_msg)

        rate.sleep()

    # Release the video capture
    video_capture.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
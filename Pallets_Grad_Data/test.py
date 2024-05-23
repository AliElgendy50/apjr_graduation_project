#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('test_video_publisher')

    # Initialize CvBridge
    bridge = CvBridge()

    # Publisher for test video frames
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)

    # Create a video stream with moving rectangles
    video_file_path = '/Pallets_Grad'  # Update with your video file path
    video_capture = cv2.VideoCapture(video_file_path)
    if not video_capture.isOpened():
        print(f"Error: Unable to open video file at {video_file_path}")
        return

    rate = rospy.Rate(30)  # Publish at 30 Hz
    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        if not ret:
            print("Error: Unable to read frame from video file")
            break

        # Draw a moving rectangle on the frame
        cv2.rectangle(frame, (50, 50), (200, 200), (0, 255, 0), 2)  # Draw a green rectangle

        # Convert the frame to ROS Image message
        frame_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the frame
        image_pub.publish(frame_msg)

        rate.sleep()

    video_capture.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
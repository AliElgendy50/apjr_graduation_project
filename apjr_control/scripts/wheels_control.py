#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose,PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from apjr_control.msg import CustomMsg
import math
import time

##########################################################################################
#                                 Euler From Quaternion                                  #
##########################################################################################

# def euler_from_quaternion(x, y, z, w):
#     orientation = quaternion.quaternion(w, x, y, z)
#     roll_x = math.degrees(euler[0])
#     pitch_y = math.degrees(euler[1])
#     yaw_z = math.degrees(euler[2])
#     return roll_x, pitch_y, yaw_z # in radians

##########################################################################################
#                                 Odom Callback function                                 #
##########################################################################################
def odom_callback(msg):
    posX = msg.pose.pose.position.x
    posY = msg.pose.pose.position.y
    orinetationX = msg.pose.pose.orientation.x
    orinetationY = msg.pose.pose.orientation.y    
    orinetationZ = msg.pose.pose.orientation.z
    orinetationW = msg.pose.pose.orientation.w

    orientation_list = [orinetationX,orinetationY,orinetationZ,orinetationW]
    EulerOrientation.roll,EulerOrientation.pitch,EulerOrientation.yaw = euler_from_quaternion(orientation_list)
    EulerOrientation = CustomMsg()
    
    pub.publish(EulerOrientation)

    
##########################################################################################
#                                 Cmd_Vel callback function                              #
##########################################################################################
def cmd_vel_callback(msg):
     linearVx = msg.linear.x
     linearVy = msg.linear.y
     angularVz = msg.angular.z


##########################################################################################
#                                         Main                                           #
##########################################################################################
if __name__=="__main__":
    rospy.init_node('wheels_control')
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,odom_callback)
    rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)

    pub = rospy.Publisher('desired_velocities',CustomMsg,queue_size=10)
    
    rospy.spin()

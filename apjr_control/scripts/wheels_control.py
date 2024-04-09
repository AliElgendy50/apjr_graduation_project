#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose,PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from apjr_control.msg import CustomMsg, VelocitiesMsg
import math
import time


##########################################################################################
#                                 Global Variables                                       #
##########################################################################################
EulerOrientation = CustomMsg()
VelocitiesData = VelocitiesMsg()
##########################################################################################
#                                 Amcl Callback function                                 #
##########################################################################################
def amcl_callback(msg):
    posX = msg.pose.pose.position.x
    posY = msg.pose.pose.position.y
    orinetationX = msg.pose.pose.orientation.x
    orinetationY = msg.pose.pose.orientation.y    
    orinetationZ = msg.pose.pose.orientation.z
    orinetationW = msg.pose.pose.orientation.w

    orientation_list = [orinetationX,orinetationY,orinetationZ,orinetationW]
    EulerOrientation.roll,EulerOrientation.pitch,EulerOrientation.yaw = euler_from_quaternion(orientation_list)
    pub_euler.publish(EulerOrientation)

    
##########################################################################################
#                                 Cmd_Vel callback function                              #
##########################################################################################
def cmd_vel_callback(msg):
    linearVx = msg.linear.x
    linearVy = msg.linear.y
    angularVz = msg.angular.z

    VelocitiesData.linearVx = linearVx
    VelocitiesData.linearVy = linearVy
    VelocitiesData.angularVz = angularVz

    pub_vel.publish(VelocitiesData)
    
     


##########################################################################################
#                                         Main                                           #
##########################################################################################
if __name__=="__main__":
    rospy.init_node('wheels_control')
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,amcl_callback)
    rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)

    pub_euler = rospy.Publisher('euler_orientation',CustomMsg,queue_size=10)
    pub_vel = rospy.Publisher('desired_velocities',VelocitiesMsg,queue_size=10)
    rospy.spin()
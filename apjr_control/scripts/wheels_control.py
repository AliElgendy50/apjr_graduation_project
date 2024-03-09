#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

##########################################################################################
#                                 Euler From Quaternion                                  #
##########################################################################################

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

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

    roll,pitch,yaw = euler_from_quaternion(orinetationX,orinetationY,orinetationZ,orinetationW)
    return roll,pitch,yaw
##########################################################################################
#                                 Cmd_Vel callback function                              #
##########################################################################################
def cmd_vel_callback(msg):
     linearVx = msg.linear.x
     linearVy = msg.linear.y
     angularVz = msg.angular.z

     return linearVx, linearVy, angularVz

##########################################################################################
#                                         Main                                           #
##########################################################################################
if __name__=="__main__":
    rospy.init_node('wheels_control')
    odom_sub = rospy.Subscriber('/odom',Odometry,odom_callback)
    odom_sub = rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)
    rospy.spin()

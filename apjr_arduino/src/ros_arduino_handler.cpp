#include "ros_arduino_handler.h"

ros_arduino_handler::ros_arduino_handler(/* args */)
{
    
}


void ros_arduino_handler::publish_data_to_ros(VectorInt16 linearVel,VectorInt16 angularVel,Quaternion quant)
{
    imu_message.linear_acceleration.x = linearVel.x;
    imu_message.linear_acceleration.y = linearVel.y;
    imu_message.linear_acceleration.z = linearVel.z;

    imu_message.angular_velocity.x = angularVel.x * (25.0 / 131.0);
    imu_message.angular_velocity.y = angularVel.y * (25.0 / 131.0);
    imu_message.angular_velocity.z = angularVel.z * (25.0 / 131.0);

    imu_message.orientation.x = quant.x;
    imu_message.orientation.y = quant.y;
    imu_message.orientation.z = quant.z;
    imu_message.orientation.w = quant.w;

    imu_message.header.frame_id = IMU_FRAME_ID;
    imu_message.header.stamp = nh.now();

}

ros_arduino_handler::~ros_arduino_handler()
{
}
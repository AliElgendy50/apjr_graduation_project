// #ifndef ros_arduino_handler_h
// #define ros_arduino_handler_h

// #include <ros.h>
// #include <sensor_msgs/Imu.h>
// #include "imu.h"
// #define IMU_FRAME_ID "imu_link" // Frame ID for the IMU data

// class ros_arduino_handler
// {
//     private:
//         /* data */
//         // ros::Time last_published_time;
//         // ros::Publisher publisher;
        
//     public:
        
//         ros_arduino_handler(/* args */);
//         ~ros_arduino_handler();
//         void publish_data_to_ros(VectorInt16 linearVel,VectorInt16 angularVel,Quaternion quant );
//         sensor_msgs::Imu imu_message;
//         ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh;
        
// };


// // ros_communication::ros_communication(/* args */)
// // {
// //     ros::NodeHandle  nh;
// // }

// // ros_communication::~ros_communication()
// // {
// // }


// #endif
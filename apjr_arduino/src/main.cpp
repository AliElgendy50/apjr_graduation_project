// #include <Arduino.h>
// #include "imu.h"
// #include <ros.h>
// #include "ros_arduino_handler.h"
// imu imu_var;
// ros_arduino_handler ros_arduino_handle;

// ros::Publisher publisher("imu",&ros_arduino_handle.imu_message);

// void setup()
// {
//     Serial.begin(9600);
//     ros_arduino_handle.nh.advertise(publisher);
// }

// void loop()
// {
//     imu_var.imu_main();
//     ros_arduino_handle.publish_data_to_ros(imu_var.aaReal,imu_var.gyro,imu_var.q);
//     publisher.publish(&ros_arduino_handle.imu_message);
    
//     ros_arduino_handle.nh.spinOnce();
//     delay(1);
    

// }
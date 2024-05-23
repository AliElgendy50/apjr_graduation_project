// /*
//  * 1) Upload the code to Arduino
//  * 2) Compile project in the ROS workspace (catkin_make)
//  * 3) $ roslaunch arduino_peripherals run_arduino_node.launch
//  * 
//  * Expected output:
//  * 
//  * // TODO: Fill with rosnode info ...
//  */

// // Include libraries
// #include "ATmega2560-HW.h"
// #include "DCMotor.h"
// #include "MagneticEncoder.h"
// #include "DifferentialDriveRobot.h"

// DifferentialDriveRobot* apjr_left;
// DifferentialDriveRobot* apjr_right;
// void setup()
// {

//     apjr_left = new DifferentialDriveRobot(LEFT);
//     apjr_right = new DifferentialDriveRobot(RIGHT);

//     nh.initNode();
    
//     last_time = millis();
// }

// void loop() 
// {

//     const double current_time = millis() / 1E3;
//     const double delta_time = (current_time - last_time);
//     // if(delta_time >= 1.0/rate)
//     // {
//     //     // Publish angles
//     //     motor_left->PublishAngle();
//     //     motor_right->PublishAngle();
//     //     // Update last time
//     //     last_time = current_time;
//     // }
//     nh.spinOnce();
//     delay(1);
// }

#pragma once

// #include <AS5048A.h>
// #include <spline.h>
#include "ATmega2560-HW.h"
#include <std_msgs/Float32.h>
#include <ros.h>
#include <InterpolationLib.h>

class MagneticEncoder
{
    double GetAngle();
    static void HandleEncoderLeft();
    double getRotationInRadians();
    // LEFT or RIGHT
	bool position;
    // Spline for linearizing the encoder output
    // Spline<double> spline;
    // ROS message
    std_msgs::Float32 msg;
    ros::Publisher pub;
public:
	// Constructor
    MagneticEncoder(uint8_t, boolean);
    // Public method
    void PublishAngle();
};

MagneticEncoder::MagneticEncoder(uint8_t digitalPin, boolean position):pub(position == LEFT ? "apjr/left_wheel_angle" : "apjr/right_wheel_angle", &msg)    
{
    pinMode(ChA, INPUT);
    pinMode(ChB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ChA),HandleEncoderLeft, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ChB), handleEncoder_A, CHANGE);
    nh.advertise(pub);
}

void MagneticEncoder::HandleEncoderLeft(){

    int currentState_A = digitalRead(ChA);
    int currentState_B = digitalRead(ChB);
    if ((lastEncoderState_A == LOW) && (currentState_A == HIGH)) 
    {
        if (currentState_B == LOW)
        {
        encoderCount_A++;
        } 
        else 
        {
        encoderCount_A--;
        }
    }
    lastEncoderState_A = currentState_A;
}

double MagneticEncoder::getRotationInRadians(){

     // Calculate the angle of rotation in radians
    double rotation = (2 * M_PI * encoderCount_A) / encoder_cpr;

    return rotation;
}

double MagneticEncoder::GetAngle() {
    // Read device output
	const double current_angle = getRotationInRadians();
    // Normalize encoder output (Change sign of the sensor reading)
	// return (this->position == LEFT? 1 : -1) * spline.value(current_angle);
    return (this->position == LEFT? 1 : -1) * map(current_angle,0,encoder_cpr,0,360);
}

void MagneticEncoder::PublishAngle() {
    msg.data = GetAngle();
    pub.publish(&msg);
}

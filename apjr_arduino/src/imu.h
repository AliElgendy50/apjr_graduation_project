#ifndef imu_h
#define imu_h

/****************************************************/
/*                     Includes                     */
/****************************************************/
#include <Arduino.h>
//#include <MPU6050.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/****************************************************/
/*                  Declaration                     */
/****************************************************/
#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno 

class imu
{
public:
    imu();
    void imu_main();
    // void publish_data_to_ros();
    Quaternion q;
    float ypr[3];
    VectorInt16 aa;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    VectorInt16 gyro;
    //MPU6050 mpu;

private:
    MPU6050 mpu;
    volatile bool mpuInterrupt;
    uint8_t fifoBuffer[64];
    VectorFloat gravity;
    uint16_t packetSize;
    bool dmpReady;
    uint8_t mpuIntStatus;
    uint16_t devStatus;

};

#endif

// class imu
// {
//     private:
//     // orientation/motion vars
//         Quaternion q;           // [w, x, y, z]         quaternion container
//         VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//         VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//         //VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//         VectorFloat gravity;    // [x, y, z]            gravity vector
//         //float euler[3];         // [psi, theta, phi]    Euler angle container
//         float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//     public:
//         imu();
//         void imu_main();    /*This function gets the readings from the imu and sets it to variables*/
//         void send_data_to_ros();
//         ~imu();

//         // MPU control/status vars
//         bool dmpReady ;  // set true if DMP init was successful
//         uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//         uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//         uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//         uint16_t fifoCount;     // count of all bytes currently in FIFO
//         uint8_t fifoBuffer[64]; // FIFO storage buffer
// };

// imu::imu(/* args */)
// {
// }

// imu::~imu()
// {
// }

//void Publish_IMU();


//#endif

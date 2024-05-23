// #include <Arduino.h>
// #include <Wire.h>
// #include <MPU6050.h>
// #include <ros.h>
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/String.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Quaternion.h>
// #include <tf/tf.h>
// #include <tf2_msgs/LookupTransformResult.h>
// #include <ros/time.h>
// #include <geometry_msgs/Quaternion.h>

// #define IMU_FRAME_ID "imu_link" // Frame ID for the IMU data

// // #include <imu_filter_madgwick/include/imu_filter_madgwick/imu_filter.h>
// ros::NodeHandle  n;

// MPU6050 mpu;
// sensor_msgs::Imu imu_message;



// ros::Publisher imu_pub("imu",&imu_message);
// geometry_msgs::Quaternion q;
// // geometry_msgs::TransformStamped t;

// // void tf_broadcast(sensor_msgs::Imu imu_message){
// //   t.header.stamp = n.now();
// //   t.header.frame_id = 




// // }

// void imu_publisher(double &accelX, double &accelY, double &accelZ, double &gyroX,double &gyroY,double &gyroZ, geometry_msgs::Quaternion quant){
//   imu_message.linear_acceleration.x = accelX *9.80665;
//   imu_message.linear_acceleration.y = accelY *9.80665;
//   imu_message.linear_acceleration.z = accelZ *9.80665;

//   imu_message.angular_velocity.x = gyroX *0.0174;
//   imu_message.angular_velocity.y = gyroY *0.0174;
//   imu_message.angular_velocity.z = gyroZ *0.0174;

//   imu_message.orientation.x = quant.x;
//   imu_message.orientation.y = quant.y;
//   imu_message.orientation.z = quant.z;
//   imu_message.orientation.w = quant.w;

//   imu_message.header.frame_id = IMU_FRAME_ID;
//   imu_message.header.stamp = n.now();

//   imu_pub.publish(&imu_message);

// }



// void setup() {
//   Serial.begin(9600);

//   Wire.begin();
  
//   mpu.initialize(); // Initialize MPU6050
//   n.advertise(imu_pub);
//   // Verify MPU6050 connection
//   // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
// }

// void loop() {
  
//   // Read raw accelerometer and gyro data
//   int16_t ax, ay, az, gx, gy, gz;
//   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//   // Convert raw data to sensible values
//   double accel_x = (float)ax / 16384.0 ; // Sensitivity Scale Factor for +/-2g range
//   double accel_y = (float)ay / 16384.0 ;
//   double accel_z = (float)az / 16384.0 ;
//   double gyro_x = (float)gx / 131.0; // Sensitivity Scale Factor for +/-250 degrees/s range
//   double gyro_y = (float)gy / 131.0;
//   double gyro_z = (float)gz / 131.0;
//   mpu.CalibrateAccel(2);
//   mpu.CalibrateGyro(2);
//   q = tf::createQuaternionFromYaw(gyro_z);
//   imu_publisher(accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,q);
  
//   n.spinOnce();


//   // Wait for a short duration before printing again
//   delay(1); // Adjust delay as needed
// }
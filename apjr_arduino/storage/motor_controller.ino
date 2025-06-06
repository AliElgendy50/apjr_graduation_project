
// // #include <Adafruit_MotorShield.h>
// // #include <Wire.h>
// #include <PID_v1.h>
// #include <ros.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Vector3Stamped.h>
// #include <geometry_msgs/Twist.h>
// #include <ros/time.h>
// #include "DCMotor.h"
// #include <std_msgs/Int16.h>

// #define Cytron
// //initializing all the variables
// #define LOOPTIME                      100     //Looptime in millisecond
// #define ENA 5
// #define ENB 9
// // DC Motor signals

// #define IN1	7
// #define IN2 8
// #define IN3 9
// #define IN4	10  

// #define Dir_1 7
// #define Dir_2 8

// volatile int lastEncoderState_A=0;

// template <typename T> int sgn(T val) {
//     return (T(0) < val) - (val < T(0));
// }
// const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
// unsigned int noCommLoops = 0;                 //main loop without communication counter

// double speed_cmd_left2 = 0;      

// const int PIN_ENCOD_A_MOTOR_LEFT = 3;               //A channel for encoder of left motor                    
// const int PIN_ENCOD_B_MOTOR_LEFT = 12;               //B channel for encoder of left motor

// const int PIN_ENCOD_A_MOTOR_RIGHT = 2;              //A channel for encoder of right motor         
// const int PIN_ENCOD_B_MOTOR_RIGHT = 6;              //B channel for encoder of right motor 

// // const int PIN_SIDE_LIGHT_LED = 13;                  //Side light blinking led pin

// unsigned long lastMilli = 0;

// //--- Robot-specific constants ---
// const double radius = 0.1;                   //Wheel radius, in m
// const double wheelbase = 0.475;               //Wheelbase, in m
// const double encoder_cpr = 7;               //Encoder ticks or counts per rotation
// const double speed_to_pwm_ratio = 0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
// const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

// double speed_req = 0;                         //Desired linear speed for the robot, in m/s
// double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

// double speed_req_left = 0;                    //Desired speed for left wheel in m/s
// double speed_act_left = 0;                    //Actual speed for left wheel in m/s
// double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

// double speed_req_right = 0;                   //Desired speed for right wheel in m/s
// double speed_act_right = 0;                   //Actual speed for right wheel in m/s
// double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                        
// const double max_speed = 10;                 //Max speed in m/s (was 0.4)

// int PWM_leftMotor = 0;                     //PWM command for left motor
// int PWM_rightMotor = 0;                    //PWM command for right motor 
                                                      
// // PID Parameters
// const double PID_left_param[] = { 0.3, 0.5, 0.001 }; //Respectively Kp, Ki and Kd for left motor PID
// const double PID_right_param[] = { 0.3, 0.5, 0.001 }; //Respectively Kp, Ki and Kd for right motor PID

// volatile float pos_left = 0;       //Left motor encoder position
// volatile float pos_right = 0;      //Right motor encoder position

// PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
// PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

// #ifndef Cytron
//   DCMotor *leftMotor = new DCMotor(IN1,IN2,ENA);
//   DCMotor *rightMotor = new DCMotor(IN3,IN4,ENB);
// #else
//   DCMotor *leftMotor = new DCMotor(Dir_2,ENB);
//   DCMotor *rightMotor = new DCMotor(Dir_1,ENA);
// #endif

// // Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create the motor shield object with the default I2C address
// // Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);      //Create left motor object
// // Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);     //Create right motor object
  
// ros::NodeHandle nh;

// //function that will be called when receiving command from host
// void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
//   noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
//   speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

//   angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
//   speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
//   speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
// }

// ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
// geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
// geometry_msgs::Vector3Stamped pwm_msg;  
// geometry_msgs::Vector3Stamped encoder_msg;  

// ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
// ros::Publisher pwm_pub("pwm", &pwm_msg);     
// ros::Publisher encoder_pub("encoder", &encoder_msg);   

// const int lightIncNumber = 30;                                                                                                                                       //Number of lightIncrements for side light blinking
// int lightInc = 0;                                                                                                                                                    //Init increment for side light blinking
// int lightValue [lightIncNumber]= { 10, 40, 80, 160, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 160, 80, 40, 10, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
// int lightValueNoComm [25]= { 255, 0, 255, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
// int lightT = 0; //init light period

// //__________________________________________________________________________

// void setup() {

  
// //   pinMode(PIN_SIDE_LIGHT_LED, OUTPUT);      //set pin for side light leds as output
// //   analogWrite(PIN_SIDE_LIGHT_LED, 255);     //light up side lights
  
//   nh.initNode();                            //init ROS node
//   nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
//   nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
//   nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
//   // nh.advertise(pwm_pub);
//   // nh.advertise(encoder_pub);

//   //setting motor speeds to zero
//   leftMotor->setSpeed(PWM_leftMotor);
// //   leftMotor->run(BRAKE);
//   rightMotor->setSpeed(0);
//   //rightMotor->run(BRAKE);
 
//   //setting PID parameters
//   PID_leftMotor.SetSampleTime(95);
//   PID_rightMotor.SetSampleTime(95);
//   PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
//   PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
//   PID_leftMotor.SetMode(AUTOMATIC);
//   PID_rightMotor.SetMode(AUTOMATIC);
    
//   // Define the rotary encoder for left motor
//   pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
//   pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
//   // digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
//   // digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
//   attachInterrupt(0, encoderLeftMotor, RISING);

//   // Define the rotary encoder for right motor
//   pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
//   pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
//   // digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
//   // digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
//   attachInterrupt(1, encoderRightMotor, RISING);
// }

// //_________________________________________________________________________

// void loop() {
//   nh.spinOnce();
//   if((millis()-lastMilli) >= LOOPTIME)   
//   {                                                                           // enter timed loop
//     lastMilli = millis();
//     pinMode(LED_BUILTIN,OUTPUT);
//     if (!nh.connected()){
//       analogWrite(LED_BUILTIN, lightValueNoComm[lightInc]);
//       lightInc=lightInc+1;
//       if (lightInc >= 25){
//         lightInc=0;
//       }
//     }
//     else{
//       analogWrite(LED_BUILTIN, lightValue [lightInc]);
//       lightT = 3000 - ((2625/max_speed)*((abs(speed_req_left)+abs(speed_req_right))/2));
//       lightInc=lightInc+(30/(lightT/LOOPTIME));
//       if (lightInc >= lightIncNumber){
//         lightInc=0;
//       }
//     }
    
    
//     if (abs(pos_left) < 1){                                                   //Avoid taking in account small disturbances
//       speed_act_left = 0;
//     }
//     else {
//       speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
//     }
    
//     if (abs(pos_right) < 1){                                                  //Avoid taking in account small disturbances
//       speed_act_right = 0;
//     }
//     else {
//     speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
//     }

//     encoder_msg.vector.x = speed_act_right;
//     encoder_msg.vector.y = speed_req_right;  

//     encoder_pub.publish(&encoder_msg);
     
//     pos_left = 0;
//     pos_right = 0;

//     speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
//     PID_leftMotor.Compute();                                               //Uncomment later !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                             
//     // compute PWM value for left motor. Check constant definition comments for more information.
//     PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
//     if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
//       // leftMotor->setSpeed(0);
//       analogWrite(ENB,0);
//     //   leftMotor->run(BRAKE);
//     }
//     else if (speed_req_left == 0){                        //Stopping
//       // leftMotor->setSpeed(0);
//       analogWrite(ENB,0);
//     //   leftMotor->run(BRAKE);
//     }
//     else{                          //Going forward
//       // leftMotor->setSpeed(PWM_leftMotor);
//       if (PWM_leftMotor > 0) {
//         digitalWrite (Dir_2, LOW);  
//         analogWrite(ENB,PWM_leftMotor);
//       } else {
//         digitalWrite (Dir_2, HIGH);  
//         analogWrite(ENB,PWM_leftMotor);
//       }
//     //   leftMotor->run(BACKWARD);
//     }
    
//     speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
//     PID_rightMotor.Compute();                                 //Uncomment later !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                                                 
//     // compute PWM value for right motor. Check constant definition comments for more information.
//     PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 

//     if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
//       // rightMotor->setSpeed(0);
//       analogWrite(ENA,0);
//     //   rightMotor->run(BRAKE);
//     }
//     else if (speed_req_right == 0){                       //Stopping
//       analogWrite(ENA,0);
//       // rightMotor->setSpeed(0);
//     //   rightMotor->run(BRAKE);
//     }
//     else{                         //Going forward
//       // rightMotor->setSpeed(PWM_rightMotor);
//       if (PWM_rightMotor > 0) {
//         digitalWrite (Dir_1, LOW);  
//         analogWrite(ENA,PWM_rightMotor);
//       } else {
//         digitalWrite (Dir_1, HIGH);  
//         analogWrite(ENA,PWM_rightMotor);
//       }
//       pwm_msg.vector.x = PWM_rightMotor;
//       pwm_msg.vector.y = PWM_leftMotor;
//       pwm_pub.publish(&pwm_msg);
//     //   rightMotor->run(FORWARD);
//     }


//     if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
//       Serial.println(" TOO LONG ");
//     }

//     noCommLoops++;
//     if (noCommLoops == 65535){
//       noCommLoops = noCommLoopMax;
//     }
    
//     publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
//   }
//  }

// //Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
// void publishSpeed(double time) {
//   speed_msg.header.stamp = nh.now();      //timestamp for odometry data
//   speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
//   speed_msg.vector.y = speed_act_right;    //left wheel speed (in m/s)
//   // speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
//   speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
//   speed_pub.publish(&speed_msg);
//   nh.spinOnce();
//   nh.loginfo("Publishing odometry");
// }

// //Left motor encoder counter
// void encoderLeftMotor() {
//   // int currentState_B = digitalRead(PIN_ENCOD_B_MOTOR_LEFT);
//   // if (currentState_B > 0)
//   // {
//   //   pos_left++;
//   // } 
//   // else 
//   // {
//   //   pos_left--;
//   // }
//   // lastEncoderState_A = currentState_A;
//   if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
//   else pos_left--;
// }

// // Right motor encoder counter
// void encoderRightMotor() {
//   if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
//   else pos_right++;
// }


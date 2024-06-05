
// #include <Adafruit_MotorShield.h>
// #include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "DCMotor.h"
#include <std_msgs/Int16.h>

#define Cytron
//initializing all the variables
#define LOOPTIME                      1000     //Looptime in millisecond
#define ENA 5
#define ENB 9
// DC Motor signals

// #define IN1	7
// #define IN2 8
// #define IN3 9
// #define IN4	10  

#define Dir_1 7
#define Dir_2 8
/* PID Variables */
// float error_r = 0;
// float error_1_r = 0;
// float error_2_r = 0;
// float u_Prev_r = 0;

// double kp_r = 50;
// double kd_r = 0.1;
// double ki_r = 10;
// float Ts = 0.1;

// float error_l = 0;
// float error_1_l = 0;
// float error_2_l = 0;
// float u_Prev_l = 0;

// double kp_l = 53;
// double kd_l = 0.1;
// double ki_l = 10;

int dir_l = 0;
int dir_r = 0;
///////////////////////////////////
double Pk1 = 1;
double Ik1 = 0;
double Dk1 = 0.03;

double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);

double Pk2 = 1;
double Ik2 = 0;
double Dk2 = 0.03;

double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

// float demand1;
// float demand2;

// float demandx;
// float demandz;

unsigned long currentMillis;
unsigned long previousMillis;

volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;
//////////////////////////////////////

volatile int lastEncoderState_A=0;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;      

const int PIN_ENCOD_A_MOTOR_LEFT = 3;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 12;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 2;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 6;              //B channel for encoder of right motor 

// const int PIN_SIDE_LIGHT_LED = 13;                  //Side light blinking led pin

unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.05;                   //Wheel radius, in m
const double wheelbase = 0.475;               //Wheelbase, in m
const double encoder_cpr = 7;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
//double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
//double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                        
const double max_speed = 0.628;                 //Max speed in m/s (was 0.4)

double PWM_leftMotor = 0;                     //PWM command for left motor
double PWM_rightMotor = 0;                    //PWM command for right motor 
                                                      
// PID Parameters
//AutoPID myPIDLeft(&speed_act_left,&speed_req_left,&PWM_leftMotor,0,255,0,0,0);
//AutoPID myPIDRight(&speed_act_right,&speed_req_right,&PWM_rightMotor,0,255,0,0,0);
// const double PID_left_param[] = { 0.3, 0.5, 0.001 }; //Respectively Kp, Ki and Kd for left motor PID
// const double PID_right_param[] = { 0.3, 0.5, 0.001 }; //Respectively Kp, Ki and Kd for right motor PID

//volatile float pos_left = 0;       //Left motor encoder position
//volatile float pos_right = 0;      //Right motor encoder position

//PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
//PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

#ifndef Cytron
  DCMotor *leftMotor = new DCMotor(IN1,IN2,ENA);
  DCMotor *rightMotor = new DCMotor(IN3,IN4,ENB);
#else
  DCMotor *leftMotor = new DCMotor(Dir_2,ENB);
  DCMotor *rightMotor = new DCMotor(Dir_1,ENA);
#endif

// Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create the motor shield object with the default I2C address
// Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);      //Create left motor object
// Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);     //Create right motor object
  
ros::NodeHandle nh;
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

  
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
//geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
// geometry_msgs::Vector3Stamped pwm_msg;  
geometry_msgs::Vector3Stamped encoder_msg;  

//ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
// ros::Publisher pwm_pub("pwm", &pwm_msg);     
ros::Publisher encoder_pub("encoder", &encoder_msg);   

// const int lightIncNumber = 30;                                                                                                                                       //Number of lightIncrements for side light blinking
// int lightInc = 0;                                                                                                                                                    //Init increment for side light blinking
// int lightValue [lightIncNumber]= { 10, 40, 80, 160, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 160, 80, 40, 10, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
// int lightValueNoComm [25]= { 255, 0, 255, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
// int lightT = 0; //init light period

//__________________________________________________________________________

void setup() {



//   pinMode(PIN_SIDE_LIGHT_LED, OUTPUT);      //set pin for side light leds as output
//   analogWrite(PIN_SIDE_LIGHT_LED, 255);     //light up side lights
  
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  //nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  //nh.advertise(pwm_pub);
  nh.advertise(encoder_pub);

  //setting motor speeds to zero
  leftMotor->setSpeed(PWM_leftMotor);
//   leftMotor->run(BRAKE);
  rightMotor->setSpeed(0);
  //rightMotor->run(BRAKE);
 
  //setting PID parameters
  // myPIDLeft.setOutputRange(0,255);
  // myPIDRight.setOutputRange(0,255);
  // PID_leftMotor.SetSampleTime(95);
  // PID_rightMotor.SetSampleTime(95);
  // PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  // PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  // PID_leftMotor.SetMode(AUTOMATIC);
  // PID_rightMotor.SetMode(AUTOMATIC);
    
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  // digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  // digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  // digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  // digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255,255);
  PID1.SetSampleTime(500);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-255,255);
  PID2.SetSampleTime(500);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();

    // pinMode(LED_BUILTIN,OUTPUT);
    // if (!nh.connected()){
    //   analogWrite(LED_BUILTIN, lightValueNoComm[lightInc]);
    //   lightInc=lightInc+1;
    //   if (lightInc >= 25){
    //     lightInc=0;
    //   }
    // }
    // else{
    //   analogWrite(LED_BUILTIN, lightValue [lightInc]);
    //   lightT = 3000 - ((2625/max_speed)*((abs(speed_req_left)+abs(speed_req_right))/2));
    //   lightInc=lightInc+(30/(lightT/LOOPTIME));
    //   if (lightInc >= lightIncNumber){
    //     lightInc=0;
    //   }
    // }

    
    
    if (abs(encoder0Pos) < 1){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      // speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
      // speed_act_left=((pos_left/encoder_cpr)*2*PI)*radius; 
      speed_act_left = (encoder0Pos/7.0) * 2.0 * 3.14 * 0.1;
    }
    
    if (abs(encoder1Pos) < 1){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    // speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
      speed_act_right = (encoder1Pos/7.0) * 2.0 * 3.14 * 0.1;
    }
  
    // pos_left = 0;
    // pos_right = 0;
    // PWM_calculate();
    // error = float(speed_req_right) - float(speed_act_right);

    // cv = cv1 + (kp+kd/Ts)*error + (-kp + ki*Ts - 2*kd/Ts) *error1 + (kd/Ts)* error2;
    // cv1 = cv;
    // error2 = error1;
    // error1 = error;

    // char dir_right = 0;

    // if(speed_req_right<0){
    //   dir_right = -1;
    // }else{
    //   dir_right = 1;
    // }

    // if(cv>500){
    // cv = 500;
    // }else if (cv<30){
    //   cv = 30;
    // }

    //in one meter the encoder produces roughly 22 ticks, so in the 500 ms, it would produce 11 ticks to have a speed of 1 m/s

    
    



    //speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    speed_req_left = constrain(speed_req_left, -max_speed, max_speed);
    encoder0Diff = encoder0Pos - encoder0Prev;
    encoder0Error = (speed_req_left*11) - encoder0Diff;
    encoder0Prev = encoder0Pos;
    Setpoint1 = (speed_req_left * 500/(2 * M_PI * 0.0508)) *7/50 ;
    Input1 = encoder0Diff;
    PID1.Compute();

    if(Output1>0){
      PWM_leftMotor = abs(Output1);
      dir_l = -1;
    }else if(Output1<0){
      PWM_leftMotor = abs(Output1);
      dir_l = 1;
    }else{
      dir_l = 0;
    }

    if(Output2>0){
      PWM_rightMotor = abs(Output2);
      dir_r = 1;
    }else if(Output2<0){
      PWM_rightMotor = abs(Output2);
      dir_r = -1;
    }else{
      dir_r = 0;
    }
    // myPIDLeft.run();
  
    // PID_leftMotor.Compute();                                               //Uncomment later !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                             
    // compute PWM value for left motor. Check constant definition comments for more information.
    
    //PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    //PWM_leftMotor = dir_left * cv*255/500;


    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      // leftMotor->setSpeed(0);
      analogWrite(ENB,0);
    //   leftMotor->run(BRAKE);
    }
    else if (speed_req_left == 0){                        //Stopping
      // leftMotor->setSpeed(0);
      analogWrite(ENB,0);
    //   leftMotor->run(BRAKE);
    }
    else{                          //Going forward
      // leftMotor->setSpeed(PWM_leftMotor);
      setMotor(dir_l,PWM_leftMotor,ENB,Dir_2);
      // if (PWM_leftMotor > 0) {
      //   digitalWrite (Dir_2, LOW);      
      //   analogWrite(ENB,PWM_leftMotor);
      // } else {
      //   digitalWrite (Dir_2, HIGH);  
      //   analogWrite(ENB,PWM_leftMotor);
      // }
    //   leftMotor->run(BACKWARD);
    }
    
    //speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);   
    speed_req_right = constrain(speed_req_right, -max_speed, max_speed);  
    encoder1Diff = encoder1Pos - encoder1Prev;
    encoder1Error = (speed_req_right*11) - encoder1Diff;
    encoder1Prev = encoder1Pos;
    Setpoint2 = (speed_req_right * 500/(2 * M_PI * 0.0508)) *7/50 ;
    Input2 = encoder1Diff;
    PID2.Compute();  

    // myPIDRight.run(); 
    // PID_rightMotor.Compute();                                 //Uncomment later !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    //PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 
    //PWM_rightMotor = dir_right * cv*255/500;
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      // rightMotor->setSpeed(0);
      analogWrite(ENA,0);
    //   rightMotor->run(BRAKE);
    }
    else if (speed_req_right == 0){                       //Stopping
      analogWrite(ENA,0);
      // rightMotor->setSpeed(0);
    //   rightMotor->run(BRAKE);
    }
    else{                         //Going forward
      // rightMotor->setSpeed(PWM_rightMotor);
      setMotor(dir_r,PWM_rightMotor,ENA,Dir_1);
      // if (PWM_rightMotor > 0) {
      //   digitalWrite (Dir_1, LOW);  
      //   analogWrite(ENA,PWM_rightMotor);
      // } else {
      //   digitalWrite (Dir_1, HIGH);  
      //   analogWrite(ENA,PWM_rightMotor);
      // }
      // pwm_msg.vector.x = PWM_rightMotor;
      // pwm_msg.vector.y = PWM_leftMotor;
      // pwm_pub.publish(&pwm_msg);
    //   rightMotor->run(FORWARD);
    }


    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  // speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  // speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  // speed_msg.vector.y = speed_act_right;    //left wheel speed (in m/s)
  // // speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  // speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  // speed_pub.publish(&speed_msg);
  encoder_msg.vector.x = speed_act_right;
  encoder_msg.vector.y = speed_act_left;  
  encoder_msg.vector.z = PWM_rightMotor;

  encoder_pub.publish(&encoder_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Left motor encoder counter
void encoderLeftMotor() {
  // int currentState_B = digitalRead(PIN_ENCOD_B_MOTOR_LEFT);
  // if (currentState_B > 0)
  // {
  //   pos_left++;
  // } 
  // else 
  // {
  //   pos_left--;
  // }
  // lastEncoderState_A = currentState_A;
  // if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  // else pos_left--;
  if (digitalRead(PIN_ENCOD_B_MOTOR_LEFT)>0){
    encoder0Pos --;
  }else{
    encoder0Pos ++;
  }
}

// Right motor encoder counter
void encoderRightMotor() {
  // if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  // else pos_right++;
  if (digitalRead(PIN_ENCOD_B_MOTOR_RIGHT) > 0) encoder1Pos --;
  else encoder1Pos ++;
}

// void PWM_calculate(){

//   error_r = speed_req_right - speed_act_right;
//   error_l = speed_req_left - speed_act_left;
//   float u_r= u_Prev_r + (kp_r+kd_r/Ts)*error_r + (-kp_r + ki_r*Ts - 2*kd_r/Ts) *error_1_r + (kd_r/Ts)* error_2_r;
//   u_Prev_r = u_r;

//   float u_l= u_Prev_l + (kp_l+kd_l/Ts)*error_l + (-kp_l + ki_l*Ts - 2*kd_l/Ts) *error_1_l + (kd_l/Ts)* error_2_l;
//   u_Prev_l = u_l;

//   PWM_rightMotor = (int) fabs(u_r);
//   if(PWM_rightMotor>255){
//     PWM_rightMotor = 255;
//   }else if (PWM_rightMotor<20){
//     PWM_rightMotor = 30;
//   }

//   PWM_leftMotor = (int) fabs(u_l);
//   if(PWM_leftMotor>255){
//     PWM_leftMotor = 255;
//   }else if (PWM_leftMotor<20){
//     PWM_leftMotor = 30;
//   }

//   dir_r = 1;
//   if (u_r<0){
//     dir_r = -1;
//   }

//   dir_l = 1;
//   if (u_l<0){
//     dir_l = -1;
//   }


// }

//////////////////////////////////////////////////////////////////////////////////

void setMotor(int dir, int pwmVal, int pwm, int Dir){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(Dir,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(Dir,HIGH);
  }
  else{
    // Or dont turn
    analogWrite(pwm,0); // Motor speed
  }
}

#pragma once
#include "ATmega2560-HW.h"

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define MAX_VALUE 0xFFFF
  #define INT_PWM   uint16_t
#else
  #define MAX_VALUE 0xFF
  #define INT_PWM   uint8_t
#endif

class DCMotor {
public:
  DCMotor();
  DCMotor(int,int,int);
  void CW(INT_PWM);
  void CCW(INT_PWM);
  void Stop();
private:  
  int INL; 
  int INH;
  int EN;
  void initPins();
  INT_PWM protectOutput(INT_PWM);
};

DCMotor::DCMotor()
  : DCMotor::DCMotor(IN1, IN2, ENA) {
  }

DCMotor::DCMotor(int INL, int INH, int EN) {
  this->INL = INL;
  this->INH = INH;
  this->EN = EN;
  DCMotor::initPins();
}

void DCMotor::initPins() {
  pinMode(this->INL, OUTPUT);
  pinMode(this->INH, OUTPUT);
  pinMode(this->EN, OUTPUT);
  DCMotor::Stop();
}

void DCMotor::Stop() {
  // Motor no gira
  digitalWrite (INL, LOW); 
  digitalWrite (INH, LOW); 
  analogWrite(EN,0);
}

void DCMotor::CW(INT_PWM val) {
  
  // Motor turns forward or CW
  digitalWrite (INL, LOW); 
  digitalWrite (INH, HIGH); 
  analogWrite(EN, protectOutput(val));
}

void DCMotor::CCW(INT_PWM val) {

  // Motor turns in the inverse direction or CCW
  digitalWrite (INL, HIGH); 
  digitalWrite (INH, LOW); 
  analogWrite(EN, protectOutput(val));
}

INT_PWM DCMotor::protectOutput(INT_PWM val) {

  // For security reasons
  val > MAX_VALUE? val = MAX_VALUE : val;

  return val;
}
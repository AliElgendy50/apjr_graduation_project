#pragma once
// #include "ATmega2560-HW.h"
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
//#define _16BIT_PWM_
#define Cytron

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
  #ifndef  Cytron
    DCMotor(int,int,int);
  #else
    DCMotor(int,int);
  #endif
  void CW(INT_PWM);
  void CCW(INT_PWM);
  void Stop();
  void setSpeed(int);
private:
  #ifndef Cytron 
    int INL; 
    int INH;
  #else
    int Dir;
  #endif
  int EN;
  void initPins();
  INT_PWM protectOutput(INT_PWM);
};
#ifndef Cytron 
  DCMotor::DCMotor()
    : DCMotor::DCMotor(INL, INH, EN) {
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
#else
  DCMotor::DCMotor()
    : DCMotor::DCMotor(Dir, EN) {
    }
  DCMotor::DCMotor(int DIR, int EN) {
    this->Dir = DIR;
    this->EN = EN;
    DCMotor::initPins();
  }
  void DCMotor::initPins() {
    pinMode(this->Dir, OUTPUT);
    pinMode(this->EN, OUTPUT);
    DCMotor::Stop();
  }
  void DCMotor::Stop() {
    // Motor no gira
    analogWrite(EN,0);
  }
  void DCMotor::CW(INT_PWM val) {
  
    // Motor turns forward or CW
    digitalWrite (Dir, LOW);  
    analogWrite(EN, protectOutput(val));
  }

  void DCMotor::CCW(INT_PWM val) {

    // Motor turns in the inverse direction or CCW
    digitalWrite (Dir, HIGH); 
    analogWrite(EN, protectOutput(val));
  }
#endif


void DCMotor::setSpeed(int PWM) {
  if (PWM > 0) {
    DCMotor::CW(PWM);
  } else {
    DCMotor::CCW(PWM);  // Pass the absolute value of msg.data as speed
  }
}

INT_PWM DCMotor::protectOutput(INT_PWM val) {

  // For security reasons
  val > MAX_VALUE? val = MAX_VALUE : val;

  return val;
}
// Driver 1 PINOUT (Right side)
// L_EN -> 53
// R_EN -> 53
// L_PWM -> 6
// R_PWM -> 5

// Driver 2 PINOUT (Left side)
// L_EN -> 52
// R_EN -> 52
// L_PWM -> 8
// R_PWM -> 7

#include "BTS7960.h"

const uint8_t EN1 = 53;
const uint8_t L_PWM1 = 6;
const uint8_t R_PWM1 = 5;
const uint8_t EN2 = 52;
const uint8_t L_PWM2 = 8;
const uint8_t R_PWM2 = 7;

BTS7960 motorController1(EN1, L_PWM1, R_PWM1);
BTS7960 motorController2(EN2, L_PWM2, R_PWM2);

void forward(int speed) {
  motorController1.TurnRight(speed);
  motorController2.TurnLeft(speed);
}

void backward(int speed) {
  motorController1.TurnLeft(speed);
  motorController2.TurnRight(speed);
}

void disable() {
  motorController1.Disable();
  motorController2.Disable();
}

void enable() {
  motorController1.Enable();
  motorController2.Enable();
}

void stop() {
  motorController1.Stop();
  motorController2.Stop();
}

void setup() 
{
}

void loop()
{
  enable();

  for(int speed = 0 ; speed < 255; speed+=10)
  {
    backward(speed);
    delay(100);
  }  

  stop();
  
  for(int speed = 255 ; speed > 0; speed-=10)
  {
    backward(speed);
    delay(100);
  }  
  
  stop();

  disable();
  
  delay(5000);
}


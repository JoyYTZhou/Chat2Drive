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

// Directions
enum directions {straight=0, left=1, right=2, back=3} ;
directions turnDirection = straight;

void move(int speed, int turndirection) {
  switch(turndirection) {
    case straight:
      motorController1.TurnRight(speed);
      motorController2.TurnLeft(speed);
      break;
    case back:
      motorController1.TurnLeft(speed);
      motorController2.TurnRight(speed); 
      break;
    case left:
      stop();
      motorController1.TurnRight(speed);
      break;
    case right:
      stop();
      motorController2.TurnLeft(speed);
      break;
  }
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
  move(100, back);
  // motorController2.TurnLeft(100); // Left wheel forward
  // motorController2.TurnRight(100); // Left wheel backward
  delay(500);
  move(100, back);
  delay(500);
  stop();
  disable();
  
  delay(5000);
}


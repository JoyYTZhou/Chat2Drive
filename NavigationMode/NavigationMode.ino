/* Instructions: Upload this to Mega, and senderCode.ino to ESP 8266 Wifi Board. */

// Compass navigation
#include <Wire.h>
#include <QMC5883LCompass.h>
QMC5883LCompass compass; // Default init status for QMC5883 compass
int targetHeading;              // the direction of intended angle, course in degrees (North=0, West=270) from position 1 to position 2,
int currentHeading;             // current angular direction
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 15    // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

// Direction received by Serial communication
char recvDirection;       // User-input direction
bool newData=false; 


// Motor Driver
#include "BTS7960.h"
const uint8_t EN1 = 53;
const uint8_t L_PWM1 = 6;
const uint8_t R_PWM1 = 5;
const uint8_t EN2 = 52;
const uint8_t L_PWM2 = 8;
const uint8_t R_PWM2 = 7;
BTS7960 motorController1(EN1, L_PWM1, R_PWM1); 
BTS7960 motorController2(EN2, L_PWM2, R_PWM2);

#include <NewPing.h> // Ultrasonic sensor library

// Ultrasonic ping sensor
#define TRIGGER_PIN  11      
#define ECHO_PIN  22
#define TRIGGER_PIN_UP 10
#define ECHO_PIN_UP 23 
#define TRIGGER_PIN_RIGHT 9
#define ECHO_PIN_RIGHT 25       
#define MAX_DISTANCE_CM 250                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches
int sonarDistance, sonarDistance_left;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);     // NewPing setup of pins and maximum distance.
NewPing sonar_left(TRIGGER_PIN_UP, ECHO_PIN_UP, MAX_DISTANCE_CM);

// Object avoidance distances (in inches)
#define SAFE_DISTANCE 50
#define TURN_DISTANCE 20
#define STOP_DISTANCE 10

// Directions
enum directions {straight=0, left=1, right=2, back=3, stop=4};
directions UserDirection = stop;
directions turnDirection = stop;

// Speeds (range: 0 - 255)
#define FAST_SPEED 230
#define NORMAL_SPEED 200
#define TURN_SPEED 170
#define SLOW_SPEED 130
int speed = NORMAL_SPEED;


void setup()
{
  Serial.begin(9600);         //Debug
  Serial2.begin(115200);        //Telegram input

  delay(1000);

  compass.init();
}

void loop()
{
  enable();

  // Know my current heading so we can still remember our target after object avoidance
  currentHeading = readCompass();

  // Read user input if available and convert that to target heading
  recvOneChar();
  newDirection();

  control();
}

void control() {
  checkSonar1();
  checkSonar2();
  // obstacles very close both ahead and on the left
  if (sonarDistance_left <= STOP_DISTANCE && sonarDistance <= STOP_DISTANCE) {
    speed=FAST_SPEED;
    turnDirection=back;
  }
  // obstacles very close on the left but not very close ahead
  else if (sonarDistance_left <= STOP_DISTANCE && sonarDistance >= STOP_DISTANCE && sonarDistance <= TURN_DISTANCE) {
    turnDirection=right;
  }
  // obstacles very close ahead but none on the left
  else if (sonarDistance_left >= TURN_DISTANCE && sonarDistance <= STOP_DISTANCE) {
    turnDirection=left;
  }
  
  // Move car based on either the user input or sonar distance sensing
  moveCar(speed, turnDirection);
  // Change turnDirection based on targetHeading
  calcDesiredTurn();
}

// currentHeading: Get the current Heading Degree
int readCompass()
{
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  // For Surabaya declination angle is 0'54E (positive)

  /* Query result for La Jolla, San Diego 
  Magnetic Declination: +11° 7'
  Declination is POSITIVE (EAST)
  Inclination: 58° 0'
  Magnetic field strength: 45810.8 nT
  Declination angle is +11 deg 7 min East (positive)
  */
  int a;

  // Read compass values
  compass.setCalibration(-1218, 1492, -1297, 1567, -800, 1747);
  compass.read();
  // Return Azimuth reading (La Jolla declination added into the method)
  a = compass.getAzimuth();
  a = (int) (a+20);

  // Correct for when signs are reversed.
  if(a < 0)
    a += 2*180;
    
  // Check for wrap due to addition of declination.
  if(a > 2*180)
    a -= 2*180;
  
  return a;
}

void calcDesiredTurn(void)
{
  // calculate where we need to turn to head to destination
  headingError = targetHeading - currentHeading;
  
  // adjust for compass wrap
  if (headingError < -180)      
    headingError += 360;
  if (headingError > 180)
    headingError -= 360;

  // calculate which way to turn to intercept the targetHeading
  if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
    turnDirection = straight;  
  else if (headingError < 0)
    turnDirection = left;
  else if (headingError > 0)
    turnDirection = right;
  else
    turnDirection = straight;
} 


// Functions for motor driving
/* ================================================================================================================================================
================================================================================================================================================ */
// motorController2.TurnLeft(100); // Left wheel forward
// motorController2.TurnRight(100); // Left wheel backward
// motorController1.TurnRight(100); // Right wheel forward
// motorController1.TurnLeft(100); // Right wheel backward
void moveCar(int speed, int turndirection) {
  switch(turndirection) {
    case straight:
      motorController1.TurnRight(speed);
      motorController2.TurnLeft(speed*2/3); // for some reason motor2 is more powerful than motor 1 for the same speed input. 
      break;
    case back:
      motorController1.TurnLeft(speed);
      motorController2.TurnRight(speed*2/3); 
      break;
    case left:
      motorController2.Stop();    
      motorController1.TurnRight(speed);
      break;
    case right:
      motorController1.Stop();
      motorController2.TurnLeft(speed);
      break;
    case stop:
      stopCar();
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

void stopCar() {
  motorController1.Stop();
  motorController2.Stop();
}

// Functions for distance sensing
/* ================================================================================================================================================
================================================================================================================================================ */
void checkSonar1(void)
{   
  unsigned long time;
  int dist;
  // sonar.ping_median(iterations [, max_cm_distance]) 
  // Do multiple pings (default=5), discard out of range pings and return median in microseconds. 
  // [max_cm_distance] allows you to optionally set a new max distance.
  time = sonar.ping_median(5, MAX_DISTANCE_CM);
  dist = NewPing::convert_in(time);             // get distance in inches from the sensor
  if (dist == 0)                                // if too far to measure, return max distance;
    dist = MAX_DISTANCE_IN;  
  sonarDistance = dist;      // add the new value into moving average, use resulting average
} 

void checkSonar2(void)
{   
  unsigned long time;
  int dist;
  // sonar.ping_median(iterations [, max_cm_distance]) 
  // Do multiple pings (default=5), discard out of range pings and return median in microseconds. 
  // [max_cm_distance] allows you to optionally set a new max distance.
  time = sonar_left.ping_median(5, MAX_DISTANCE_CM);
  dist = NewPing::convert_in(time);             // get distance in inches from the sensor
  if (dist == 0)                                // if too far to measure, return max distance;
    dist = MAX_DISTANCE_IN;  
  sonarDistance_left = dist;      // add the new value into moving average, use resulting average
}

// Temporary debug functions
void printDirection(int turnDirection) {
  switch(turnDirection) {
    case straight:
      Serial.println("Go forward");
      break;
    case left:
      Serial.println("Go to your left");
      break;
    case right:
      Serial.println("Go to your right");
      break;
    case back:
      Serial.println("Turn around");
      break;
  }
}

// Functions for reading user input data from telegram
// ================================================================================================================================================
void recvOneChar() {
  if (Serial2.available()>0) {
    recvDirection=Serial2.read();
    newData=true;
    Serial.println(recvDirection);
  }
}

void newDirection() {
  int newDir=10;
  bool newHeading=false;
  if (newData==true) {
    if (recvDirection=='r') {
      targetHeading=currentHeading+45;
    }
    else if (recvDirection=='l') {
      targetHeading=currentHeading-45;
    }
    else if (recvDirection=='f') {
      targetHeading=currentHeading;
    }
    else if (recvDirection=='b') {
      turnDirection=back;
    }
    else if (recvDirection=='h') {
      newDir=stop;
    }
    else if (recvDirection=='n') {
      targetHeading=10;
      newHeading=true;
    }
    else if (recvDirection=='e') {
      targetHeading=90;
      newHeading=true;
    }
    else if (recvDirection=='s') {
      targetHeading=180;
      newHeading=true;
    }
    else if (recvDirection=='w') {
      targetHeading=340;
      newHeading=true;
    }
    newData=false;
  }
  if (newDir!=10 || newHeading==true) {
    calcDesiredTurn();
    turnDirection=newDir;
    UserDirection=newDir;
  }
}

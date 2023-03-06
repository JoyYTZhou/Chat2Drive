#include <TinyGPS++.h>
static const uint32_t GPSBaud = 9600; // NEO-6M has a GPS baud rate of 9600
TinyGPSPlus gps; 

// Compass navigation
#include <Wire.h>
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass(&Wire, /*I2C addr*/QMC5883_ADDRESS); // Default init status for QMC5883 compass
int targetHeading;              // the direction of intended angle
int currentHeading;             // current angular direction
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 8     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

// GPS location
#include "math.h"
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial3); 
float currentLat,
      currentLong;

int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it

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
#define MAX_DISTANCE_CM 250                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches
int sonarDistance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);     // NewPing setup of pins and maximum distance.

// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 40
#define STOP_DISTANCE 12

// Directions
enum directions {straight=0, left=1, right=2, back=3} ;
directions turnDirection = straight;

// Speeds (range: 0 - 255)
#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
int speed = NORMAL_SPEED;

// Waypoints Definition
#define WAYPOINT_DIST_TOLERANE 3   //tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define stopwaypoint 5                  //second
int current_waypoint = 0;

// Slide South -lat
float waypointList[10][2] = {
  { 32.874959, -117.240641}, // Fairbank Coffee
  { 32.874787, -117.241373}, 
  { 32.874727, -117.241808}, 
};

float targetLat = waypointList[0][0];
float targetLong = waypointList[0][1];

void setup()
{
  Serial.begin(9600);         //Debug
  Serial3.begin(9600);        //GPS

  GPS.begin(9600);                                // 9600 NMEA default speed
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);     // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);       // 1 Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);                // turn off antenna status info
  delay(1000);

  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(QMC5883_RANGE_2GA);
  // Set measurement mode
  compass.setMeasurementMode(QMC5883_CONTINOUS);
  // Set data rate
  compass.setDataRate(QMC5883_DATARATE_50HZ);
  // Set number of samples averaged
  compass.setSamples(QMC5883_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  // compass.setOffset(0, 0);
  Serial.println("Start");

  Serial.println(waypointList[0][0], 6);
  Serial.println(waypointList[0][1], 6);

  Serial.println(waypointList[1][0], 6);
  Serial.println(waypointList[1][1], 6);

}

int id = 0;
int mode;

String val[5];
int nprint = 0;
int aprint = 0;

void loop()
{
  
  while (Serial3.available() > 0)
    if (gps.encode(Serial3.read()))
      processGPS();


  currentHeading = readCompass();
  calcDesiredTurn();

  // distance in front of us, move, and avoid obstacles as necessary
  // checkSonar();
  sonarDistance=100;
  moveAndAvoid(); 
  delay(2000);

  Serial.print("_ch "); Serial.print(currentHeading);
  Serial.print("_th "); Serial.print(targetHeading);
  Serial.print("_eh "); Serial.print(headingError);
  Serial.print("_di "); Serial.print(distanceToTarget);
  Serial.println();
}

// Functions for reading GPS module and navigate to the next waypoint
/* ================================================================================================================================================
================================================================================================================================================ */
void processGPS()
{
  if (gps.location.isValid())
  {
    currentLat = gps.location.lat();
    currentLong = gps.location.lng();

    // update the course and distance to waypoint based on our new position
    distanceToWaypoint();
    courseToWaypoint();
  }
}

// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
int courseToWaypoint()
{
  float dlon = radians(targetLong - currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}

// returns distance in meters between two positions, both specified
// as signed decimal-degrees latitude and longitude. Uses great-circle
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint()
{
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  distanceToTarget =  delta * 6372795;

  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE-1)
  {
    stop();
    delay(stopwaypoint*1000);
    nextWaypoint();
  }
  return distanceToTarget;
}

// Get the current Heading Degree
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
  Declination angle is 7'11E (positive)
  */
  float declinationAngle = (7.0 + (11.0 / 60.0)) / (180 / M_PI);
  compass.setDeclinationAngle(declinationAngle);
  float headingDegrees=compass.getHeadingDegrees();
  return ((int)headingDegrees);

}

void nextWaypoint()
{
  current_waypoint++;
  targetLat = waypointList[current_waypoint][0];
  targetLong = waypointList[current_waypoint][1];

  if ((targetLat == 0 && targetLong == 0))    // last waypoint reached?
  {
    stop();
    Serial2.println("FINISH");
    while (1); //PRESS RESET BUTTON
  }
  else
  {
    processGPS();
    distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
    courseToWaypoint();
  }
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
      motorController1.TurnRight(speed);
      motorController2.Stop();
    case right:
      motorController1.Stop();
      motorController2.TurnRight(speed);
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

// Functions for distance sensing
/* ================================================================================================================================================
================================================================================================================================================ */
void checkSonar(void)
{   
  unsigned long time;
  int dist;
  time = sonar.ping_median(5, MAX_DISTANCE_CM);
  dist = NewPing::convert_in(time);                   // get distqnce in inches from the sensor
  if (dist == 0)                                // if too far to measure, return max distance;
    dist = MAX_DISTANCE_IN;  
  sonarDistance = dist;      // add the new value into moving average, use resulting average
} 

// Temporary debug functions
void printDirection(int turnDirection) {
  switch(turnDirection) {
    case straight:
      Serial.println("Go forward");
    case left:
      Serial.println("Go to your left");
    case right:
      Serial.println("Go to your right");
    case back:
      Serial.println("Turn around");
  }
}

void moveAndAvoid(void)
{
    if (sonarDistance >= SAFE_DISTANCE) {       // no close objects in front of car
      {
          if (turnDirection == straight)
            speed = FAST_SPEED;
          else {
            speed = TURN_SPEED;
            move(speed, turnDirection);
            printDirection(turnDirection);
          }
          move(speed, straight);
          return;
      }
    }
    if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE) // not yet time to turn, but slow down
    {   
      {
        if (turnDirection == straight)
          speed = NORMAL_SPEED;
        else
        {
          speed = TURN_SPEED;
          move(speed, turnDirection);      // alraedy turning to navigate
          printDirection(turnDirection);
        }
        move(speed, straight);       
        return;
      }
    }
     
    if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
    {
      speed = SLOW_SPEED;
      move(speed, straight);      // slow down
      switch (turnDirection)
      {
        case straight:                  // going straight currently, so start new turn
        {
          if (headingError <= 0)
            turnDirection = left;
          else
            turnDirection = right;
          move(TURN_SPEED, turnDirection);  // turn in the new direction
          break;
        }
        case left:                         // if already turning left, try right
        {
          move(TURN_SPEED, turnDirection);    
          break;  
        }
        case right:                       // if already turning right, try left
        {
          move(TURN_SPEED, turnDirection);
          break;
        }
      }
      return;
    }  

    if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
    {
      stop();            // stop 
      turnDirection=back;
      move(SLOW_SPEED, turnDirection); // straighten up
      delay(200);
      move(NORMAL_SPEED, turnDirection); // go back at higher speed
      while (sonarDistance < TURN_DISTANCE)       // backup until we get safe clearance
      {
        if(GPS.parse(GPS.lastNMEA()) )
            processGPS();  
        currentHeading = readCompass();    // get our current heading
        calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      
        checkSonar();
        delay(100);
      }
      stop(); // stop backing up
      return;
    } // end of IF TOO CLOSE
} 

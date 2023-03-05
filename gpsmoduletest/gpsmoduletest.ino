#include <TinyGPS++.h>
#include <Adafruit_GPS.h>

TinyGPSPlus gps;
Adafruit_GPS GPS(&Serial3);

float currentLat,
      currentLong;

// Choose two Arduino pins to use for software serial
// int RXPin = 51;
// int TXPin = 10;

//Default baud of NEO-6M is 9600
int GPSBaud = 9600;

// Create a software serial port called "gpsSerial"
// SoftwareSerial gpsSerial(RXPin, TXPin);

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  // Serial.begin(115200);

  // Start the software serial port at the GPS's default baud
  // gpsSerial.begin(GPSBaud);

  Serial.begin(9600);         //Debug
  Serial3.begin(9600);       //GPS
  
  GPS.begin(9600);                                // 9600 NMEA default speed
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);     // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);       // 1 Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);                // turn off antenna status info
  delay(1000);  
}

void processGPS()
{
  if (gps.location.isValid())
  {
    currentLat = gps.location.lat();
    currentLong = gps.location.lng();
    Serial.println(currentLat);
    Serial.println(currentLong);
  }
}

void loop()
{
  // Displays information when new sentence is available.
  // while (gpsSerial.available() > 0) {
  //   byte gpsData = gpsSerial.read();
  //   Serial.write(gpsData);
  // }
  while (Serial3.available() > 0)
  if (gps.encode(Serial3.read()))
    processGPS();
}

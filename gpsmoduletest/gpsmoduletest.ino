#include <SoftwareSerial.h>

// Choose two Arduino pins to use for software serial
int RXPin = 51;
int TXPin = 10;

//Default baud of NEO-6M is 9600
int GPSBaud = 9600;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(115200);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
}

void loop()
{
  // Displays information when new sentence is available.
  while (gpsSerial.available() > 0) {
    byte gpsData = gpsSerial.read();
    Serial.write(gpsData);
  }
}

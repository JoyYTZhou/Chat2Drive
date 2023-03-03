// #include <SoftwareSerial.h>
// use mega Serial 2 for serial monitor; Serial 1 on pins 19 (RX) and 18 (TX);// Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX).
#define SSID "jo's"
#define PASS "20011114"

// SoftwareSerial dbgSerial(10, 11); // RX, TX
void setup()
{
  // Open serial communications and wait for port to open:
  // serial 2 is to esp8266
  Serial2.begin(115200); // 9600 (mine), 57600, 115200
  Serial2.setTimeout(1000);

  // serial 0 is to usb
  Serial.begin(9600);

  while (!Serial)
    ;
  while (!Serial2)
    ;

  // dbgSerial.begin(9600); //can't be faster than 19200 for softserial
  // dbgSerial.println("ESP8266 Demo");
  Serial.println("ESP8266 Demo on Mega2560");
  while (Serial2.available() > 0)
    Serial2.read();

  delay(1000);
  // test if the module is ready
  Serial2.println("AT+RST");
  // delay(1000);
  // delay(1000);
  Serial.println("Resetting module");
  Serial2.flush();

  // if(Serial2.find("ready"))
  if (Serial2.find("Ready") || Serial2.find("ready"))
  {
    // dbgSerial.println("Module is ready");
    Serial.println("Module is ready");
  }
  else
  {
    // dbgSerial.println("Module have no response.");
    Serial.println("Module have no response.");
    while (1);
  }
  delay(1000);
  // connect to the wifi
  bool connected = false;
  for (int i = 0; i < 5; i++)
  {
    if (connectWiFi())
    {
      connected = true;
      break;
    }
  }
  if (!connected)
  {
    while (1)
      ;
  }
  delay(5000);
  // print the ip addr
  /*
  Serial2.println("AT+CIFSR");
  Serial.println("ip address:");
  while (Serial2.available())
  Serial.write(Serial2.read());

  */
  // set the single connection mode
  Serial2.println("AT+CIPMUX=0");
}

void loop()
{ 
  
}

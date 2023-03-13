#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>


// Initialize Wifi connection to the router
const char* ssid     = "Jo's";
const char* password = "20011114";


// Initialize Telegram BOT
const char BotToken[] = "5798862019:AAHw1L1SIugLDb7JiVgXwlQFkrZX6x_f_5E";

WiFiClientSecure net_ssl;
TelegramBot bot (BotToken, net_ssl);

void setup() 
{  
 Serial.begin(115200);
 while (!Serial) {}  //Start running when the serial is open 
 delay(3000);  
 // attempt to connect to Wifi network:  
 Serial.print("Connecting Wifi: ");  
 Serial.println(ssid);  
 while (WiFi.begin(ssid, password) != WL_CONNECTED) 
       {  
   Serial.print(".");  
   delay(500);  
 }  
 Serial.println("");  
 Serial.println("WiFi connected");  
 bot.begin();
}

void loop() 
{  
 message m = bot.getUpdates(); // Read new messages  
 Serial.println(m.text);
}  

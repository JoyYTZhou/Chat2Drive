#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// Wifi network station credentials
#define WIFI_SSID "Jo"
#define WIFI_PASSWORD "20140128"
// Telegram BOT Token (Get from Botfather)
#define BOT_TOKEN "5798862019:AAHw1L1SIugLDb7JiVgXwlQFkrZX6x_f_5E"

const unsigned long BOT_MTBS = 1000; // mean time between scan messages

X509List cert(TELEGRAM_CERTIFICATE_ROOT);
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);
unsigned long bot_lasttime; // last time messages' scan has been done

void handleNewMessages(int numNewMessages)
{
  for (int i = 0; i < numNewMessages; i++)
  {
    char str;
    if (bot.messages[i].text=="right" || bot.messages[i].text=="Right") {
      str='r';
    }
    else if (bot.messages[i].text=="left" || bot.messages[i].text=="Left") {
      str='l';
    }
    else if (bot.messages[i].text=="forward" || bot.messages[i].text=="Forward") {
      str='f';
    }
    else if (bot.messages[i].text=="back" || bot.messages[i].text=="Back") {
      str='b';
    }
    Serial.write(str);
    bot.sendMessage(bot.messages[i].chat_id, bot.messages[i].text, "");
  }
}

void setup()
{
  Serial.begin(115200);
  // Serial.println("Serial communication started.");

  // attempt to connect to Wifi network:
  // Serial.print("Connecting to Wifi SSID ");
  Serial.write(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secured_client.setTrustAnchors(&cert); // Add root certificate for api.telegram.org
  
  while (WiFi.status() != WL_CONNECTED)
  {
    // Serial.write(".");
    delay(500);
  }

  configTime(0, 0, "pool.ntp.org"); // get UTC time via NTP
  time_t now = time(nullptr);
  while (now < 24 * 3600)
  {
    delay(100);
    now = time(nullptr);
  }
}

void loop()
{
  if (millis() - bot_lasttime > BOT_MTBS)
  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages)
    {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
  }
}

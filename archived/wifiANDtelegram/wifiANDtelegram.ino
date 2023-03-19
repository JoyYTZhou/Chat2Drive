#include <TelegramBot.h>
#include <WiFiEsp.h>

char ssid[] = "UCSD-GUEST";            // your network SSID (name)
char pass[] = "";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

const char BotToken[] = "5798862019";
WiFiEspClient client;
TelegramBot bot (BotToken, client);

void setup()
{
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  Serial2.begin(115200);
  // initialize ESP module
  WiFi.init(&Serial2);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");

  bot.begin();
}

void loop() {
  message m = bot.getUpdates(); // Read new messages
  if ( m.chat_id != 0 ){ // Checks if there are some updates
    Serial.println(m.text);
    bot.sendMessage(m.chat_id, m.text);  // Reply to the same chat with the same text
  } else {
    Serial.println("no new message");
  }
}

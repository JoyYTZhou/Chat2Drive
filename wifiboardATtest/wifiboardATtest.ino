void setup() {
  // put your setup code here, to run once:
  Serial2.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial2.available()) Serial.write(Serial2.read());
  while(Serial.available()) Serial.write(Serial.read());
}

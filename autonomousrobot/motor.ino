#include <Arduino.h>

#define Pin_D1_R  40
#define Pin_D2_R  42
#define Pin_E_R   44
#define Pin_E_L   46
#define Pin_D1_L  48
#define Pin_D2_L  50

void setup_motor()
{
  pinMode(Pin_D1_L, OUTPUT);
  pinMode(Pin_D2_L, OUTPUT);
  pinMode(Pin_E_L, OUTPUT);
  pinMode(Pin_D1_R, OUTPUT);
  pinMode(Pin_D2_R, OUTPUT);
  pinMode(Pin_E_R, OUTPUT);
}

void stop()
{
  analogWrite(Pin_E_L, 0);
  digitalWrite(Pin_D1_L, LOW);
  digitalWrite(Pin_D2_L, LOW);
  analogWrite(Pin_E_R, 0);
  digitalWrite(Pin_D1_R, LOW);
  digitalWrite(Pin_D2_R, LOW);
}


void advance(int left, int right)
{
  if (left < 0)
  {
    digitalWrite(Pin_D1_L, HIGH);
    digitalWrite(Pin_D2_L, LOW);
    left = -1 * left;
  }
  else
  {
    digitalWrite(Pin_D1_L, LOW);
    digitalWrite(Pin_D2_L, HIGH);
  }
  analogWrite(Pin_E_L, left);

  if (right < 0)
  {
    digitalWrite(Pin_D1_R, HIGH);
    digitalWrite(Pin_D2_R, LOW);
    right = -1 * right;
  }
  else
  {
    digitalWrite(Pin_D1_R, LOW);
    digitalWrite(Pin_D2_R, HIGH);
  }
  analogWrite(Pin_E_R, right);
}

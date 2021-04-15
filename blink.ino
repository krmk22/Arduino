#include <Arduino.h>
#include <avr/io.h>

#ifndef LEDConfig
#define LEDConfig
#define LED 13
#endif

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop()
{
  digitalWrite(LED, !digitalRead(LED));
  delay(1000);
}

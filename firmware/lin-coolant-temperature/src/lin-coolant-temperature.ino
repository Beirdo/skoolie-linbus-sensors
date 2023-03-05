#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>
#include <linbus_interface.h>

#include "project.h"

void reset_isr(void);

void reset_isr(void)
{
  void (* reboot)(void) = 0;
  reboot();
}


void setup() 
{
  // Analog Input, actually
  analogReference(VDD);
  analogReadResolution(12);
  pinMode(PIN_VTHERMO0, INPUT);
  pinMode(PIN_VTHERMO1, INPUT);

  pinMode(PIN_RESET, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RESET), reset_isr, FALLING);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  init_linbus(PIN_LIN_SLP, PIN_LIN_WAKE, PIN_LED);
}

void loop() 
{
  static uint16_t ledCounter = 0;
  int topOfLoop = millis();

  bool ledOn = ((ledCounter++ & 0x07) == 0x01);
  digitalWrite(PIN_LED, ledOn);

  update_linbus();
  process_linbus();

  int elapsed = millis() - topOfLoop;
  int delayMs = clamp<int>(100 - elapsed, 1, 100);
  delay(delayMs);
}

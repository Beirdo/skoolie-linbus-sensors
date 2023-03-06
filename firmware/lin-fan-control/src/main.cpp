#include <Arduino.h>
#include <Wire.h>
#include <LM96163.h>
#include <Adafruit_LiquidCrystal.h>
#include <linbus_interface.h>

#ifndef DISABLE_LOGGING
#define DISABLE_LOGGING
#endif

#include "project.h"

void reset_isr(void);

void reset_isr(void)
{
  void (* reboot)(void) = 0;
  reboot();
}

void setup() 
{
  pinMode(PIN_ALERT, INPUT);
  pinMode(PIN_TCRIT, INPUT);

  pinMode(PIN_RESET, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RESET), reset_isr, FALLING);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  lm96163.begin(&Wire, PIN_ALERT, PIN_TCRIT);
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

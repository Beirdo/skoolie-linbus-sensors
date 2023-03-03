#include <Arduino.h>
#include <Wire.h>
#include <LINBus_stack.h>
#include <Adafruit_LiquidCrystal.h>
#include <linbus_interface.h>

#ifndef DISABLE_LOGGING
#define DISABLE_LOGGING
#endif

#include "project.h"
#include "ina219.h"

void reset_isr(void);

void reset_isr(void)
{
  void (* reboot)(void) = 0;
  reboot();
}

void setup()
{
  pinMode(PIN_PUMP_EN, OUTPUT);
  digitalWrite(PIN_PUMP_EN, LOW);

  pinMode(PIN_RESET, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RESET), reset_isr, FALLING);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  ina219.begin(12, 320, 100, 12, 64);    

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

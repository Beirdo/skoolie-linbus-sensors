#include <Arduino.h>
#include <Wire.h>
#include <TCA9534-GPIO.h>
#include <Adafruit_LiquidCrystal.h>
#include <linbus_interface.h>

#ifndef DISABLE_LOGGING
#define DISABLE_LOGGING
#endif

#include "project.h"
#include "linbus_interface.h"

TCA9534 tca9534;

void reset_isr(void);
void i2c_int_isr(void);

void reset_isr(void)
{
  void (* reboot)(void) = 0;
  reboot();
}

void i2c_int_isr(void)
{
  // Do squat for now.  This SHOULD trigger a read of all the inputs.
}

void setPhasePWM(bool cw, bool ccw)
{
  uint8_t duty_cycle;

  int8_t direction = ((cw ^ ccw) ? (cw ? 1 : -1) : 0);
  duty_cycle = 50 + (direction * 50);

  analogWrite(PIN_PHASE_PWM, map<uint8_t>(duty_cycle, 0, 100, 0, 255)); 
}

void setup() 
{
  // Analog Input, actually
  analogReference(INTERNAL2V5);
  analogReadResolution(12);
  pinMode(PIN_VPROPI, INPUT);

  pinMode(PIN_I2C_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_I2C_INT), i2c_int_isr, FALLING);

  pinMode(PIN_RESET, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RESET), reset_isr, FALLING);

  pinMode(PIN_PHASE_PWM, OUTPUT);
  setPhasePWM(false, false);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // Setup inputs and outputs on the TCA9534
  tca9534.begin(Wire, I2C_ADDR_TCA9534);

  tca9534.pinMode(PIN_ENABLE, OUTPUT);
  tca9534.digitalWrite(PIN_ENABLE, LOW);  

  tca9534.pinMode(PIN_SLEEP, OUTPUT);
  tca9534.invertPin(PIN_SLEEP, TCA9534_INVERT);
  tca9534.digitalWrite(PIN_SLEEP, HIGH);    // Actually (active) low.

  tca9534.pinMode(PIN_VALVE_CLOSED, INPUT);
  tca9534.pinMode(PIN_VALVE_OPENED, INPUT);

  tca9534.pinMode(PIN_FAULT, INPUT);
  tca9534.invertPin(PIN_FAULT, TCA9534_INVERT);

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

#include <Arduino.h>
#include <Wire.h>
#include <LINBus_stack.h>
#include <PCA9536D.h>
#include <Adafruit_LiquidCrystal.h>

#include "project.h"
#include "linbus_interface.h"

PCA9536 pca9536;

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

  pinMode(PIN_LIN_SLP, OUTPUT);
  digitalWrite(PIN_LIN_SLP, LOW);
    
  pinMode(PIN_LIN_WAKE, OUTPUT);
  digitalWrite(PIN_LIN_WAKE, LOW);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Get the LINBus ID (0x00-0x0F) from PCA9536 - we could do 0x00-0x1F if needed
  pca9536.begin();

  linbus_address = 0x00;
  if (pca9536.isConnected()) { 
    for (int i = 0; i < 4; i++) {
      pca9536.pinMode(i, INPUT);    
      linbus_address |= pca9536.digitalRead(i) ? BIT(i) : 0;
    }
  }

  init_linbus(linbus_address);
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

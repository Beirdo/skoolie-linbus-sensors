#include <Arduino.h>
#include <Wire.h>
#include <LINBus_stack.h>
#include <PCA9536D.h>
#include <SparkFun_TCA9534.h>
#include <Adafruit_LiquidCrystal.h>

#include "project.h"
#include "linbus_interface.h"

PCA9536 pca9536;
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

  // Get the LINBus ID (0x00-0x0F) from PCA9536 - we could do 0x00-0x1F if needed
  pca9536.begin();

  linbus_address = 0x00;
  if (pca9536.isConnected()) { 
    for (int i = 0; i < 4; i++) {
      pca9536.pinMode(i, INPUT);    
      linbus_address |= pca9536.digitalRead(i) ? BIT(i) : 0;
    }
  }

  // Setup inputs and outputs on the TCA9534
  tca9534.begin(Wire, I2C_ADDR_TCA9534);

  tca9534.pinMode(PIN_LED, OUTPUT);
  tca9534.digitalWrite(PIN_LED, HIGH);

  tca9534.pinMode(PIN_ENABLE, OUTPUT);
  tca9534.digitalWrite(PIN_ENABLE, LOW);  

  tca9534.pinMode(PIN_SLEEP, OUTPUT);
  tca9534.invertPin(PIN_SLEEP, INVERT);
  tca9534.digitalWrite(PIN_SLEEP, HIGH);    // Actually (active) low.

  tca9534.pinMode(PIN_VALVE_CLOSED, INPUT);
  tca9534.pinMode(PIN_VALVE_OPENED, INPUT);

  tca9534.pinMode(PIN_FAULT, INPUT);
  tca9534.invertPin(PIN_FAULT, INVERT);

  tca9534.pinMode(PIN_LIN_SLP, OUTPUT);
  tca9534.digitalWrite(PIN_LIN_SLP, LOW);
    
  tca9534.pinMode(PIN_LIN_WAKE, OUTPUT);
  tca9534.digitalWrite(PIN_LIN_WAKE, LOW);

  init_linbus(linbus_address);
}

void loop() 
{
  static uint16_t ledCounter = 0;
  int topOfLoop = millis();

  bool ledOn = ((ledCounter++ & 0x07) == 0x01);
  tca9534.digitalWrite(PIN_LED, ledOn);

  update_linbus();
  process_linbus();

  int elapsed = millis() - topOfLoop;
  int delayMs = clamp<int>(100 - elapsed, 1, 100);
  delay(delayMs);
}

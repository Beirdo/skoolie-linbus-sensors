#include <Arduino.h>
#include <Wire.h>
#include <lin_stack.h>
#include <EEPROM.h>

#include "project.h"
#include "ntc_thermistor.h"

enum {
  REG_CONTROL,
  REG_MAX_WRITEABLE,
};

enum {
  REG_STATUS = REG_MAX_WRITEABLE,
  REG_FLOW_HI,
  REG_FLOW_LO,
  REG_TEMP_HI,
  REG_TEMP_LO,
  REG_MAX_READ_ONLY,
};

#define MAX_REGISTERS REG_MAX_READ_ONLY

uint8_t registerIndex = 0xFF;
uint8_t registerBank[MAX_REGISTERS] = {0};
int flowPulseCount = 0;
int flowPulseLastClear = 0; 

uint8_t linbus_address;
uint8_t linbus_buf[2];
uint8_t linbus_buf_len = 2;

void setOpenDrainOutput(uint8_t pin, bool value, bool invert = false);
int getFlowPulsePerSec(void);
void flow_pulse_isr(void);

lin_stack *linbus;
NTCThermistor thermistor(25, 50, 10000, 3545, coolant_ntc_resistance_table, coolant_ntc_resistance_count, coolant_ntc_offset);


void flow_pulse_isr(void)
{
  flowPulseCount++;
}

void setOpenDrainOutput(uint8_t pin, bool value, bool invert)
{
  if (invert) {
    value = !value;
  }

  if (!value) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  } else {
    pinMode(pin, INPUT_PULLUP);
    digitalWrite(pin, HIGH);
  }
}

int getFlowPulsePerSecond(void)
{
  int result = 0;
  int delta;

  noInterrupts();
  int now = millis();
  delta = now - flowPulseLastClear;
  if (delta > 0 && delta <= 1000) {
    result = flowPulseCount * 1000 / delta;
  }

  flowPulseCount = 0;
  flowPulseLastClear = now;
  interrupts();

  return result;
}

void setup() 
{
  pinMode(PIN_ONBOARD_LED, OUTPUT);
  digitalWrite(PIN_ONBOARD_LED, HIGH);

  linbus_address = EEPROM.read(0);
  if (linbus_address == 0xFF) {
    linbus_address = 0x1F;
    EEPROM.update(0, linbus_address);
  }
  linbus = new lin_stack(Serial, 19200, PIN_LIN_WAKE, linbus_address);

  setOpenDrainOutput(PIN_LIN_SLP, false, true);
  setOpenDrainOutput(PIN_LIN_WAKE, false, true);

  digitalWrite(PIN_VALVE1_INP, LOW);
  pinMode(PIN_VALVE1_INP, OUTPUT);

  digitalWrite(PIN_VALVE1_INN, LOW);
  pinMode(PIN_VALVE1_INN, OUTPUT);

  pinMode(PIN_VALVE1_CLOSED, INPUT);
  pinMode(PIN_VALVE1_OPENED, INPUT);

  pinMode(PIN_FLOW_PULSE, INPUT);

  digitalWrite(PIN_VALVE2_INP, LOW);
  pinMode(PIN_VALVE2_INP, OUTPUT);

  digitalWrite(PIN_VALVE2_INN, LOW);
  pinMode(PIN_VALVE2_INN, OUTPUT);

  pinMode(PIN_VALVE2_CLOSED, INPUT);
  pinMode(PIN_VALVE2_OPENED, INPUT);

  pinMode(PIN_MOTOR_FAULT, INPUT_PULLUP);

  digitalWrite(PIN_MOTOR_EN, LOW);
  pinMode(PIN_MOTOR_EN, OUTPUT);

  pinMode(PIN_COOLANT_NTC, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_PULSE), flow_pulse_isr, RISING);

  getFlowPulsePerSecond();
}

void loop() 
{
  static uint16_t ledCounter = 0;
  int topOfLoop = millis();

  bool ledOn = ((ledCounter++ & 0x07) == 0x01);
  digitalWrite(PIN_ONBOARD_LED, ledOn);

  // Let's do this.  Filter the control of the valves
  uint8_t mask = VALVE2_CLOSED | VALVE2_OPENED | VALVE1_CLOSED | VALVE1_OPENED;
  uint8_t control = registerBank[REG_CONTROL] & (mask >> 2);
  uint8_t status  = (registerBank[REG_STATUS] & mask) >> 2;

  // Clear the request if it's already done
  control ^= status;

  // If both open and close are requested, just stop
  if ((control & 0x30) == 0x30) {
    control &= 0x0F;
  }

  if ((control & 0x03) == 0x03) {
    control &= 0xF0;
  }

  digitalWrite(PIN_MOTOR_EN, control);

  digitalWrite(PIN_VALVE2_INP, control & VALVE2_OPEN);
  digitalWrite(PIN_VALVE2_INN, control & VALVE2_CLOSE);

  digitalWrite(PIN_VALVE1_INP, control & VALVE1_OPEN);
  digitalWrite(PIN_VALVE1_INN, control & VALVE1_CLOSE);

  status = control;
  status |= digitalRead(PIN_VALVE2_CLOSED) ? VALVE2_CLOSED : 0;
  status |= digitalRead(PIN_VALVE2_OPENED) ? VALVE2_OPENED : 0;
  status |= digitalRead(PIN_VALVE1_CLOSED) ? VALVE1_CLOSED : 0;
  status |= digitalRead(PIN_VALVE1_OPENED) ? VALVE1_OPENED : 0;

  registerBank[REG_STATUS] = status;

  int value = analogRead(PIN_COOLANT_NTC);
  int resistance = value * 10000 / (4096 - value);
  int temperature = thermistor.lookup(resistance);

  registerBank[REG_TEMP_HI] = HI_BYTE(temperature);
  registerBank[REG_TEMP_LO] = LO_BYTE(temperature);

  value = getFlowPulsePerSecond();
  // F = 5.5 * Q where Q = L/min, max of 60L/min
  value = map<int>(value, 0, 330, 0, 60000);

  registerBank[REG_FLOW_HI] = HI_BYTE(value);
  registerBank[REG_FLOW_LO] = LO_BYTE(value);

  int read;
  if (linbus->read(linbus_buf, linbus_buf_len, &read)) {
    if (read) { 
      // this was a packet written to us
      if (linbus_buf[0] == 0x00) {
        registerBank[REG_CONTROL] = linbus_buf[1];
      } else if ((linbus_buf[0] & 0x80) == 0x80) {
        registerIndex = linbus_buf[0] & 0x7F;
      }
    } else {
      registerIndex = clamp<int>(registerIndex, 0, MAX_REGISTERS - 1);
      linbus_buf[0] = registerBank[registerIndex++];
      registerIndex = clamp<int>(registerIndex, 0, MAX_REGISTERS - 1);
      linbus_buf[1] = registerBank[registerIndex++];
      linbus->writeResponse(linbus_buf, 2);
    } 
  } else {
    linbus->sleep(STATE_SLEEP);
  }

  int elapsed = millis() - topOfLoop;
  int delayMs = clamp<int>(100 - elapsed, 1, 100);
  delay(delayMs);
}

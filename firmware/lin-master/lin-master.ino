#include <Arduino.h>
#include <MultiWire.h>
#include <LINBus_stack.h>
#include <EEPROM.h>

#include "project.h"
#include "sensor_eeprom.h"


void setOpenDrainOutput(uint8_t pin, bool value, bool invert = false);
void i2c_request_event(void);
void i2c_receive_event(int count);
void i2c_eeprom_request_event(void);
void i2c_eeprom_receive_event(uint8_t *buf, int count);
void i2c_bridge_request_event(void);
void i2c_bridge_receive_event(uint8_t *buf, int count);

LINBus_stack *linbus;
MultiWire wire = MultiWire();
uint8_t lastAddress = 0xFF;
uint16_t eepromAddress;
uint8_t current_linbus_id;
uint8_t current_linbus_regnum;

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


void setup() 
{
  pinMode(PIN_ONBOARD_LED, OUTPUT);
  digitalWrite(PIN_ONBOARD_LED, HIGH);

  linbus = new LINBus_stack(Serial, 19200, PIN_LIN_WAKE, PIN_LIN_SLP);

  setOpenDrainOutput(PIN_LIN_SLP, false, true);
  setOpenDrainOutput(PIN_LIN_WAKE, false, true);

  pinMode(PIN_I2C_A0, INPUT);
  pinMode(PIN_I2C_A1, INPUT);
  pinMode(PIN_I2C_A2, INPUT);

  init_eeprom();

  const uint8_t addrs[] = { eeprom_i2c_addr, bridge_i2c_addr };
  wire.begin(addrs, 2);  
  wire.onReceive(i2c_receive_event);
  wire.onRequest(i2c_request_event);
}

uint32_t linbus_probe(void)
{
  uint32_t slaves = 0x00000000;
  uint8_t  base_addr = 0x00;
  uint8_t  buf[3];
  int len;  

  linbus->busWakeUp();
  for (int i = 0; i < 32; i++) {
    linbus->writeRequest(base_addr + i);
    len = linbus->readStream(buf, 3);
    if (len == 3 && linbus->validateChecksum(buf, 3)) {
      bitSet(slaves, i);      
    }
  }

  return slaves;
}


void loop() 
{
  static uint16_t ledCounter = 0;
  int topOfLoop = millis();

  bool ledOn = ((ledCounter++ & 0x07) == 0x01);
  digitalWrite(PIN_ONBOARD_LED, ledOn);



  int elapsed = millis() - topOfLoop;
  int delayMs = clamp<int>(100 - elapsed, 1, 100);
  delay(delayMs);
}


void i2c_request_event(void)
{
  lastAddress = wire.lastAddress();

  if (lastAddress == eeprom_i2c_addr) {
    i2c_eeprom_request_event();
  } else if (lastAddress == bridge_i2c_addr) {
    i2c_bridge_request_event();
  }
}

void i2c_receive_event(int count)
{
  uint8_t buffer[count];
  count = wire.readBytes(buffer, count);

  lastAddress = wire.lastAddress();

  if (lastAddress == eeprom_i2c_addr) {
    i2c_eeprom_receive_event(buffer, count);
  } else if (lastAddress == bridge_i2c_addr) {
    i2c_bridge_receive_event(buffer, count);
  } 
}

void i2c_eeprom_request_event(void)
{
  // Reads from the EEPROM.  send off a page per read.
  uint8_t buffer[INTERNAL_EEPROM_PAGE_SIZE];
  for (int i = 0, addr = eepromAddress; i < INTERNAL_EEPROM_PAGE_SIZE; i++, addr++) {
    if( addr < 0 || addr >= INTERNAL_EEPROM_SIZE) {
      buffer[i] = 0xFF;
    } else {
      buffer[i] = EEPROM.read(addr);
    }
  }
  wire.write(buffer, INTERNAL_EEPROM_PAGE_SIZE);
}

void i2c_eeprom_receive_event(uint8_t *buf, int count)
{
  if (count < 2) {
    return;
  }

  count = clamp<int>(count - 2, 0, INTERNAL_EEPROM_PAGE_SIZE);
  eepromAddress = (buf[0] << 1) | buf[1];

  for (int i = 0, addr = eepromAddress; i < count && addr < INTERNAL_EEPROM_SIZE; i++, addr++) {
    EEPROM.update(addr, buf[i + 2]);
  }
}

void i2c_bridge_request_event(void)
{
  uint16_t data;
  uint8_t *buf = (uint8_t *)&data;
  
  // reads - we always will return two bytes, which we request over the LINBus.  The I2C side has id, register
  // if we know the device isn't there, let I2C timeout.
  if (!(eeprom_data.current.linbus_slaves & BIT(current_linbus_id))) {
    return;
  }
  
  linbus->write(current_linbus_id, &current_linbus_regnum, 1);
  linbus->writeRequest(current_linbus_id);
  linbus->readStream(buf, 2);
  wire.write(buf, 2);
}

void i2c_bridge_receive_event(uint8_t *buf, int count)
{
  uint16_t data;
  
  if (count < 2) {
    return;
  }

  current_linbus_id = buf[0] & 0x1F;
  current_linbus_regnum = buf[1];

  // writes - we always will send the register number (and 1 byte if count > 2), 
  // which we send over the LINBus.
  
  if (!(eeprom_data.current.linbus_slaves & BIT(current_linbus_id))) {
    return;
  }
  
  count = clamp<int>(count, 2, 3);

  linbus->write(current_linbus_id, buf, count);
}

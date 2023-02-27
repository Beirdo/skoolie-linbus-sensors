#include <Arduino.h>
#include <LINBus_stack.h>
#include <EEPROM.h>

#include "project.h"
#include "linbus_interface.h"
#include "linbus_registers.h"
#include "ntc_thermistor.h"

uint8_t registerIndex = 0xFF;
uint8_t linbus_address;
uint8_t linbus_buf[2];
uint8_t linbus_buf_len = 2;

LINBusRegister registers[] = {
  LINBusRegister(0x00, BOARD_TYPE_COOLANT_TEMP),
  LINBusRegister(0xFF, 0xFF),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

LINBus_stack linbus(Serial, 19200);

void init_linbus(uint8_t address)
{
  uint8_t location = EEPROM.read(0);
  registers[REG_LOCATION].write(location);

  linbus.begin(PIN_LIN_WAKE, PIN_LIN_SLP, linbus_address);
  linbus_address = address & 0x1F;
}

void update_linbus(void)
{
  int value = analogRead(PIN_VTHERMO0);
  int resistance = value * 10000 / (4096 - value);
  int temperature = thermistor.lookup(resistance);

  registers[REG_COOLANT_TEMP0_HI].write(HI_BYTE(temperature), true);
  registers[REG_COOLANT_TEMP0_LO].write(LO_BYTE(temperature), true);

  value = analogRead(PIN_VTHERMO1);
  resistance = value * 10000 / (4096 / value);
  temperature = thermistor.lookup(resistance);

  registers[REG_COOLANT_TEMP1_HI].write(HI_BYTE(temperature), true);
  registers[REG_COOLANT_TEMP1_LO].write(LO_BYTE(temperature), true);
}

void process_linbus(void)
{
  size_t read_;

  if (!linbus.waitBreak(50)) {
    return;    
  }

  if (linbus.read(linbus_buf, linbus_buf_len, &read_)) {
    if (read_) { 
      uint8_t addr = linbus_buf[0];
      uint8_t data = linbus_buf[1];

      if (addr & 0x80) {
        registerIndex = addr & 0x7F;
      } else if (addr < MAX_REGISTERS) {
        // this was a packet written to us
        registers[addr].write(data);
      }

      if (addr == REG_LOCATION) {
        EEPROM.update(0, data);
      }
    } else {
      registerIndex = clamp<int>(registerIndex, 0, MAX_REGISTERS - 1);
      linbus_buf[0] = registerIndex < MAX_REGISTERS ? registers[registerIndex++].read() : 0x00;

      registerIndex = clamp<int>(registerIndex, 0, MAX_REGISTERS - 1);
      linbus_buf[1] = registerIndex < MAX_REGISTERS ? registers[registerIndex++].read() : 0x00;

      linbus.writeResponse(linbus_buf, 2);
    } 
  } else {
    linbus.sleep(STATE_SLEEP);
  }  
}

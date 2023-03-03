#include <Arduino.h>
#include <LINBus_stack.h>
#include <EEPROM.h>

#include "project.h"
#include "linbus_interface.h"
#include "linbus_registers.h"
#include "ina219.h"

uint8_t registerIndex = 0xFF;
uint8_t linbus_address;
uint8_t linbus_buf[2];
uint8_t linbus_buf_len = 2;

LINBusRegister registers[] = {
  LINBusRegister(0x00, BOARD_TYPE_PELTIER_CONTROL),
  LINBusRegister(0xFF, 0xFF),
  LINBusRegister(0x01, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

LINBus_stack linbus(Serial, 19200);

void init_linbus(uint8_t address)
{
  uint8_t location = EEPROM.read(0);
  registers[REG_LOCATION].write(location);

  linbus_address = address & 0x1F;
  linbus.begin(PIN_LIN_WAKE, PIN_LIN_SLP, linbus_address);
}

void update_linbus(void)
{
  uint16_t current = abs(ina219.getCurrent_mA());
  registers[REG_PELTIER_CURRENT_HI] = HI_BYTE(current);
  registers[REG_PELTIER_CURRENT_LO] = LO_BYTE(current);
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

        if (addr == REG_PELTIER_CONTROL) {
          digitalWrite(PIN_PELTIER_EN, data);
        } else if (addr == REG_LOCATION) {
          EEPROM.update(1, data);
        }
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

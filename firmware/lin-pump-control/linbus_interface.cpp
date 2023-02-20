#include <Arduino.h>
#include <LINBus_stack.h>

#include "project.h"
#include "linbus_interface.h"
#include "linbus_registers.h"

uint8_t registerIndex = 0xFF;
uint8_t linbus_address;
uint8_t linbus_buf[2];
uint8_t linbus_buf_len = 2;

LINBusRegister registers[] = {
  LINBusRegister(0x00, BOARD_TYPE_PUMP_CONTROL),
  LINBusRegister(0x00, 0x01),
};

LINBus_stack linbus(Serial, 19200);

void init_linbus(uint8_t address)
{
  linbus_address = address & 0x1F;
  linbus.begin(PIN_LIN_WAKE, PIN_LIN_SLP, linbus_address);
}

void update_linbus(void)
{
  // Nothing to update yet
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

        if (addr == REG_PUMP_CONTROL) {
          digitalWrite(PIN_PUMP_EN, data);
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
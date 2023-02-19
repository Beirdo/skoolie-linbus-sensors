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
  LINBusRegister(0x00, BOARD_TYPE_FLOW_SENSOR),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

LINBus_stack linbus(Serial, 19200);

void init_linbus(uint8_t address)
{
  linbus_address = address & 0x1F;
  linbus.begin(PIN_LIN_WAKE, PIN_LIN_SLP, linbus_address);
}

void update_linbus(void)
{
  int now = millis();
  int value0 = pulse0_count;
  int value1 = pulse1_count;
  int duration = now - last_millis;
  uint16_t flow0, flow1;

  last_millis = now;
  pulse0_count = 0;
  pulse1_count = 0;

  if (!duration) {
    flow0 = 0;
    flow1 = 0;
  } else {
    int uL = value0 * 22500;
    int mL_per_sec = uL / duration;   // * 1000 ms/s, / 1000 uL/mL
    int mL_per_min = mL_per_sec * 60;
    flow0 = clamp<uint16_t>(mL_per_min, 0, 65535);

    uL = value1 * 22500;
    mL_per_sec = uL / duration;
    mL_per_min = mL_per_sec * 60;
    flow1 = clamp<uint16_t>(mL_per_min, 0, 65535);
  }

  registers[REG_FLOW_RATE0_HI].write(HI_BYTE(flow0), true);
  registers[REG_FLOW_RATE0_LO].write(LO_BYTE(flow0), true);
  registers[REG_FLOW_RATE1_HI].write(HI_BYTE(flow1), true);
  registers[REG_FLOW_RATE1_LO].write(LO_BYTE(flow1), true);
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
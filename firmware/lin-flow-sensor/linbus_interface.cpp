#include <Arduino.h>
#include <linbus_map.h>
#include <linbus_registers.h>
#include <linbus_interface.h>

#include "project.h"
#include "register_bank.h"

int maxRegisters = MAX_REGISTERS;
int regLocation = REG_LOCATION;

LINBusRegister registers[MAX_REGISTERS] = {
  LINBusRegister(0x00, BOARD_TYPE_FLOW_SENSOR),
  LINBusRegister(0xFF, 0xFF),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

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

void dispatch_linbus(uint8_t index, uint8_t value)
{
  (void)index;
  (void)value;  
}

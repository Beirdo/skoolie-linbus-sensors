#include <Arduino.h>
#include <linbus_map.h>
#include <linbus_registers.h>
#include <linbus_interface.h>

#include "project.h"
#include "register_bank.h"
#include "ina219.h"

int maxRegisters = MAX_REGISTERS;
int regLocation = REG_LOCATION;

LINBusRegister registers[MAX_REGISTERS] = {
  LINBusRegister(0x00, BOARD_TYPE_PELTIER_CONTROL),
  LINBusRegister(0xFF, 0xFF),
  LINBusRegister(0x01, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

void dispatch_linbus(uint8_t index, uint8_t value)
{
  if (index == REG_PELTIER_CONTROL) {
    digitalWrite(PIN_PELTIER_EN, value);
  }
}

void update_linbus(void)
{
  uint16_t current = abs(ina219.getCurrent_mA());
  registers[REG_PELTIER_CURRENT_HI] = HI_BYTE(current);
  registers[REG_PELTIER_CURRENT_LO] = LO_BYTE(current);
}

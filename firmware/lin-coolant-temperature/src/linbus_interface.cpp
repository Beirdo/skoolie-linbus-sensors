#include <Arduino.h>
#include <linbus_map.h>
#include <linbus_registers.h>
#include <linbus_interface.h>

#include "project.h"
#include "register_bank.h"
#include "ntc_thermistor.h"

int maxRegisters = MAX_REGISTERS;
int regLocation = REG_LOCATION;

LINBusRegister registers[MAX_REGISTERS] = {
  LINBusRegister(0x00, BOARD_TYPE_COOLANT_TEMP),
  LINBusRegister(0xFF, 0xFF),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

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

void dispatch_linbus(uint8_t index, uint8_t value) 
{
  (void)index;
  (void)value;
}

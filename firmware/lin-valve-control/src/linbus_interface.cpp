#include <Arduino.h>
#include <linbus_map.h>
#include <linbus_registers.h>
#include <linbus_interface.h>

#include "project.h"
#include "register_bank.h"

int maxRegisters = MAX_REGISTERS;
int regLocation = REG_LOCATION;

LINBusRegister registers[MAX_REGISTERS] = {
  LINBusRegister(0x00, BOARD_TYPE_VALVE_CONTROL),
  LINBusRegister(0xFF, 0xFF),
  LINBusRegister(0x03, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

void update_linbus(void)
{
  // Let's do this.  Filter the control of the valves
  uint8_t mask = VALVE_CLOSED | VALVE_OPENED;
  uint8_t control = registers[REG_VALVE_CONTROL].read() & (mask >> 2);
  uint8_t status  = (registers[REG_VALVE_STATUS].read() & mask) >> 2;

  // Clear the request if it's already done
  control ^= status;

  // If both open and close are requested, just stop
  if ((control & 0x03) == 0x03) {
    control &= 0xF0;
  }

  setPhasePWM(control & VALVE_OPEN, control & VALVE_CLOSE);
  tca9534.digitalWrite(PIN_ENABLE, control);

  status = control;
  status |= tca9534.digitalRead(PIN_VALVE_CLOSED) ? VALVE_CLOSED : 0;
  status |= tca9534.digitalRead(PIN_VALVE_OPENED) ? VALVE_OPENED : 0;

  registers[REG_VALVE_STATUS].write(status, true);

  int value = analogRead(PIN_VPROPI);     // 2.5V at 2.5A, measured with VRef of 3.3V
  int milliamps = map<int>(value, 0, 3103, 0, 2500);

  registers[REG_VALVE_CURRENT_HI].write(HI_BYTE(milliamps), true);
  registers[REG_VALVE_CURRENT_LO].write(LO_BYTE(milliamps), true);
}

void dispatch_linbus(uint8_t index, uint8_t value)
{
  (void)index;
  (void)value;
}
#include <Arduino.h>
#include <LINBus_stack.h>
#include <EEPROM.h>

#include "project.h"
#include "linbus_interface.h"
#include "linbus_registers.h"

uint8_t registerIndex = 0xFF;
uint8_t linbus_address;
uint8_t linbus_buf[2];
uint8_t linbus_buf_len = 2;

LINBusRegister registers[] = {
  LINBusRegister(0x00, BOARD_TYPE_VALVE_CONTROL),
  LINBusRegister(0xFF, 0xFF),
  LINBusRegister(0x03, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
};

LINBus_stack linbus(Serial, 19200);

void i2c_pinMode(uint8_t pin, uint8_t mode);
void i2c_digitalWrite(uint8_t pin, uint8_t value);


void i2c_pinMode(uint8_t pin, uint8_t mode)
{
  tca9534.pinMode(pin, mode);
}

void i2c_digitalWrite(uint8_t pin, uint8_t value)
{
  tca9534.digitalWrite(pin, value);
}

void init_linbus(uint8_t address)
{
  uint8_t location = EEPROM.read(0);
  registers[REG_LOCATION].write(location);

  linbus_address = address & 0x1F;
  linbus.setPinMode(i2c_pinMode);
  linbus.setDigitalWrite(i2c_digitalWrite);
  linbus.begin(PIN_LIN_WAKE, PIN_LIN_SLP, linbus_address);
}

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

        if (addr == REG_LOCATION) {
          EEPROM.update(0, data);
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

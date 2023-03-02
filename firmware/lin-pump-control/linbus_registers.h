#ifndef __linbus_registers_h_
#define __linbus_registers_h_

#include <Arduino.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_LOCATION,
  REG_PUMP_CONTROL,
  REG_PUMP_CURRENT_HI,
  REG_PUMP_CURRENT_LO,
  MAX_REGISTERS,
} register_pump_controller_t;

typedef enum {
  BOARD_TYPE_VALVE_CONTROL = 0,
  BOARD_TYPE_COOLANT_TEMP,
  BOARD_TYPE_FLOW_SENSOR,
  BOARD_TYPE_FAN_CONTROL,
  BOARD_TYPE_PUMP_CONTROL,
  BOARD_TYPE_PELTIER_CONTROL,
} board_types_t;


class LINBusRegister {
  public:
    LINBusRegister(uint8_t write_mask, uint8_t default_value = 0x00) :
      _write_mask(write_mask), _default_value(default_value), _value(default_value) {}
    void write(uint8_t value, bool raw = false);
    uint8_t read(void);
    void changeBit(uint8_t bitNum, bool newValue, bool raw = false);
  private:
    uint8_t _write_mask;
    uint8_t _default_value;
    uint8_t _value;
};

#endif

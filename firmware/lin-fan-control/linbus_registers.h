#ifndef __linbus_registers_h_
#define __linbus_registers_h_

#include <Arduino.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_FAN_PWM,
  REG_FAN_RPM_HI,
  REG_FAN_RPM_LO,
  REG_INTERNAL_TEMP_HI,
  REG_INTERNAL_TEMP_LO,
  REG_EXTERNAL_TEMP_HI,
  REG_EXTERNAL_TEMP_LO,
  REG_INTERNAL_LO_THRESH,
  REG_INTERNAL_HI_THRESH,
  REG_EXTERNAL_LO_THRESH,
  REG_EXTERNAL_HI_THRESH,  
  MAX_REGISTERS,
} register_valve_controller_t;

typedef enum {
  BOARD_TYPE_VALVE_CONTROL = 0,
  BOARD_TYPE_COOLANT_TEMP,
  BOARD_TYPE_FLOW_SENSOR,
  BOARD_TYPE_FAN_CONTROL,
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

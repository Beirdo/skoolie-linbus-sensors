#ifndef __project_h_
#define __project_h_

#include <Arduino.h>

#define PIN_RXD           0
#define PIN_TXD           1
#define PIN_FLOW_PULSE    2
#define PIN_FAN_PWM       3
#define PIN_VALVE1_INP    4
#define PIN_VALVE1_INN    5
#define PIN_VALVE1_CLOSED 6
#define PIN_VALVE1_OPENED 7
#define PIN_LIN_SLP       8
#define PIN_VALVE2_INP    9
#define PIN_VALVE2_INN    10
#define PIN_VALVE2_CLOSED 11
#define PIN_VALVE2_OPENED 12
#define PIN_ONBOARD_LED   13
#define PIN_MOTOR_EN      14
#define PIN_LIN_WAKE      15
#define PIN_MOTOR_FAULT   16
#define PIN_PUMP_EN       17
#define PIN_SDA           18
#define PIN_SCL           19
#define PIN_COOLANT_NTC   A7

#define VALVE2_CLOSED     0x80
#define VALVE2_OPENED     0x40
#define VALVE2_CLOSE      0x20
#define VALVE2_OPEN       0x10
#define VALVE1_CLOSED     0x08
#define VALVE1_OPENED     0x04
#define VALVE1_CLOSE      0x02
#define VALVE1_OPEN       0x01

enum {
  REG_VALVE_CONTROL,
  REG_COOLING_CONTROL,
  REG_MAX_WRITEABLE,
};

enum {
  REG_VALVE_STATUS = REG_MAX_WRITEABLE,
  REG_COOLING_STATUS,
  REG_FLOW_HI,
  REG_FLOW_LO,
  REG_TEMP_HI,
  REG_TEMP_LO,
  REG_MAX_READ_ONLY,
};

#define MAX_REGISTERS REG_MAX_READ_ONLY

#define HI_BYTE(x)     ((uint8_t)(((int)(x) >> 8) & 0xFF))
#define LO_BYTE(x)     ((uint8_t)(((int)(x) & 0xFF)))

#define HI_NIBBLE(x)  ((uint8_t)(((int)(x) >> 4) & 0x0F))
#define LO_NIBBLE(x)  ((uint8_t)(((int)(x) & 0x0F)))


template <typename T>
inline T clamp(T value, T minval, T maxval)
{
  return max(min(value, maxval), minval);
}

template <typename T>
inline T map(T x, T in_min, T in_max, T out_min, T out_max)
{
  // the perfect map fonction, with constraining and float handling
  x = clamp<T>(x, in_min, in_max);
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#endif

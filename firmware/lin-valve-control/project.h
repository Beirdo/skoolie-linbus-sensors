#ifndef __project_h_
#define __project_h_

#include <Arduino.h>

#define PIN_SCL           PIN_PB0
#define PIN_SDA           PIN_PB1
#define PIN_TXD           PIN_PB2
#define PIN_RXD           PIN_PB3
#define PIN_UPDI          PIN_PA0
#define PIN_VPROPI        PIN_PA1
#define PIN_I2C_INT       PIN_PA2
#define PIN_RESET         PIN_PA3
#define PIN_PHASE_PWM     PIN_PA4

// On the PCA9536
#define PIN_ADDR3         3
#define PIN_ADDR2         2
#define PIN_ADDR1         1
#define PIN_ADDR0         0

// On the TCA9534
#define PIN_LED           0
#define PIN_ENABLE        1
#define PIN_SLEEP         2
#define PIN_VALVE_CLOSED  3
#define PIN_VALVE_OPENED  4
#define PIN_FAULT         5
#define PIN_LIN_SLP       6
#define PIN_LIN_WAKE      7

#define VALVE_CLOSED      BIT(3)
#define VALVE_OPENED      BIT(2)
#define VALVE_CLOSE       BIT(1)
#define VALVE_OPEN        BIT(0)

#define I2C_ADDR0_LCD     0x27
#define I2C_ADDR1_LCD     0x3F
#define I2C_ADDR_PCA9536  0x41
#define I2C_ADDR_TCA9534  0x20


#define HI_BYTE(x)     ((uint8_t)(((int)(x) >> 8) & 0xFF))
#define LO_BYTE(x)     ((uint8_t)(((int)(x) & 0xFF)))

#define HI_NIBBLE(x)  ((uint8_t)(((int)(x) >> 4) & 0x0F))
#define LO_NIBBLE(x)  ((uint8_t)(((int)(x) & 0x0F)))

#define BIT(x)        ((uint32_t)(1 << (x)))

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

void setPhasePWM(bool cw, bool ccw);


#endif

#ifndef __project_h_
#define __project_h_

#include <Arduino.h>

#define PIN_RXD           0
#define PIN_TXD           1
#define PIN_LIN_SLP       2
#define PIN_LIN_WAKE      3
#define PIN_I2C_A0        7
#define PIN_I2C_A1        8
#define PIN_I2C_A2        9
#define PIN_ONBOARD_LED   13
#define PIN_I2C_SDA       18
#define PIN_I2C_SCL       19

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

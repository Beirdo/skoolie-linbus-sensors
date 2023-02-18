#ifndef __project_h_
#define __project_h_

#include <Arduino.h>

#define PIN_SCL           PIN_PB0
#define PIN_SDA           PIN_PB1
#define PIN_TXD           PIN_PB2
#define PIN_RXD           PIN_PB3
#define PIN_UPDI          PIN_PA0
#define PIN_VTHERMO0      PIN_PA1
#define PIN_VTHERMO1      PIN_PA2
#define PIN_RESET         PIN_PA3
#define PIN_LIN_SLP       PIN_PA4
#define PIN_LIN_WAKE      PIN_PA5
#define PIN_LED           PIN_PA7

// On the PCA9536
#define PIN_ADDR3         3
#define PIN_ADDR2         2
#define PIN_ADDR1         1
#define PIN_ADDR0         0

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

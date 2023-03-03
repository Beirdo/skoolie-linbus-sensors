#include <Arduino.h>
#include <Wire.h>
#include <Beirdo-Utilities.h>

#include "project.h"
#include "ina219.h"

INA219 ina219(I2C_ADDRESS_INA219);

void INA219::begin(uint8_t max_voltage, uint16_t mv_range, uint16_t r_milliohms, uint8_t bits, uint8_t samples)
{
  Wire.begin();
  Wire.setClock(400000);

  i2c_write_register_word(0x00, 0x0000);          // Power it down while we configure

  _max_voltage = max_voltage <= 16 ? 16 : 32;
  _r_milliohms = r_milliohms;
  
  _mv_range = 0x03;
  
  mv_range /= 40;
  for (int i = 0; i <= 4; i++) {
    if (mv_range < BIT(i)) {
      _mv_range = i;
    }
  }

  _resolution = 0x0F;
  if (samples == 1) {
    bits = clamp<uint8_t>(bits, 9, 12);
    _resolution = bits - 9;
  } else {
    for (int i = 0; i <= 7; i++) {
      if (samples <= BIT(i)) {
        _resolution = i | 0x08;
      }
    }
  }

  _device_config = 0;
  _device_config |= _max_voltage == 32 ? BIT(13) : 0;
  _device_config |= _mv_range << 11;
  _device_config |= _resolution << 7;
  _device_config |= _resolution << 3;
  _device_config |= 0x0007;             // continous readings on shunt and bus

  uint32_t max_mA = 1000 * (40 << _mv_range) / r_milliohms;
  _nA_LSB = (max_mA * 1000000) >> 15; 
  // This is super-tricky to do in 32-bit integer math without overflow/underflow
  // cal = 0.04096 / (current_LSB [A] * R_shunt [Ohms])
  //     = 40960000 / (current_LSB [uA] * R_shunt [mOhms])
  //     = ((40960000 / R_shunt [mOhms]) * 1000) / current_LSB [nA]

  uint32_t cal = 40960000 / r_milliohms;
  cal *= 1000;
  cal /= _nA_LSB;
  _calibration = (uint16_t)cal;

  i2c_write_register_word(0x05, _calibration);    // Set the calibration value
  i2c_write_register_word(0x00, _device_config);  // Configure, and run continuously
}

int16_t INA219::getShuntVoltage_mV(void)
{
  int16_t reading = (int16_t)i2c_read_register_word(0x01);
  _shunt_voltage_mV = reading / 100;            // VShunt LSB = 10uV fixed.
  return _shunt_voltage_mV;
}

int16_t INA219::getBusVoltage_mV(void)
{
  uint16_t reading = i2c_read_register_word(0x02);
  _bus_voltage_mV = (reading & 0xFFF8) << 3;   // VBus LSB = 4mV fixed, mask off status bits
  return _bus_voltage_mV;
}

int16_t INA219::getCurrent_mA(void)
{
  int16_t reading = (int16_t)(i2c_read_register_word(0x04));
  int32_t current_nA = reading * _nA_LSB;
  _current_mA = current_nA / 1000000;
  return (int16_t)_current_mA;
}

int16_t INA219::getPower_mW(void)
{
  int16_t reading = (int16_t)(i2c_read_register_word(0x03));
  int32_t power_nW = reading * _nA_LSB * 20;
  _power_mW = power_nW / 1000000;
  return (int16_t)_power_mW;
}

uint16_t INA219::i2c_read_register_word(uint8_t index)
{
  Wire.beginTransmission(_i2c_address);
  Wire.write(index);
  Wire.endTransmission();
  Wire.requestFrom(_i2c_address, 2);

  uint16_t value = 0;
  for (int i = 0; i < 2; i++) {
    value <<= 8;
    if (Wire.available()) {
      value |= Wire.read();
    }
  }

  return value;
}

void INA219::i2c_write_register_word(uint8_t index, uint16_t value)
{
  Wire.beginTransmission(_i2c_address);
  Wire.write(index);
  Wire.write(HI_BYTE(value));
  Wire.write(LO_BYTE(value));
  Wire.endTransmission();  
}

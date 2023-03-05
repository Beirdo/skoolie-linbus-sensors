#ifndef __ina219_h_
#define __ina219_h_

class INA219 {
  public:
    INA219(uint8_t i2c_address) : _i2c_address(i2c_address) {};
    void begin(uint8_t max_voltage, uint16_t mv_range, uint16_t r_milliohms, uint8_t bits = 12, uint8_t samples = 64);
    int16_t getShuntVoltage_mV(void);
    int16_t getBusVoltage_mV(void);
    int16_t getCurrent_mA(void);
    int16_t getPower_mW(void);

  protected:
    uint16_t i2c_read_register_word(uint8_t index);
    void i2c_write_register_word(uint8_t index, uint16_t value);

    uint8_t _i2c_address;
    bool _connected;

    uint8_t _resolution;
    uint8_t _max_voltage;
    uint16_t _mv_range;
    uint16_t _r_milliohms;
    uint8_t _max_current;
    uint32_t _nA_LSB;

    uint16_t _device_config;
    uint16_t _calibration;

    int16_t _shunt_voltage_mV;
    int16_t _bus_voltage_mV;
    int16_t _current_mA;
    int16_t _power_mW;
};

extern INA219 ina219;

#endif
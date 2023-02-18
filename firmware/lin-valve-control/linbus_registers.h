#ifndef __linbus_registers_h_
#define __linbus_registers_h_

#include <Arduino.h>

class LINBusRegister {
  public:
    LINBusRegister(const char *name, uint8_t write_mask, uint8_t default_value = 0x00) :
      _name(name), _write_mask(write_mask), _default_value(default_value), _value(default_value) {}
    void write(uint8_t value, bool raw = false);
    uint8_t read(void);
    void changeBit(uint8_t bitNum, bool newValue, bool raw = false);
  private:
    const char *_name;
    uint8_t _write_mask;
    uint8_t _default_value;
    uint8_t _value;
};

struct LINBusRegisterItem_s; 
typedef struct LINBusRegisterItem_s {
  uint8_t index;
  LINBusRegister *reg;
  struct LINBusRegisterItem_s *prev;
  struct LINBusRegisterItem_s *next;
} LINBusRegisterItem_t;

class LINBusRegisterFile {
  public:
    LINBusRegisterFile();
    void addRegister(uint8_t index, LINBusRegister *register_);
    LINBusRegister *getRegister(uint8_t index);
  private:
    LINBusRegisterItem_t *_head;
};

extern LINBusRegisterFile linbusRegisterFile;

#endif
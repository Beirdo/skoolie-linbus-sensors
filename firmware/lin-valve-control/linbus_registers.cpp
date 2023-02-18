#include <Arduino.h>

#include "project.h"
#include "linbus_registers.h"


void LINBusRegister::write(uint8_t value, bool raw)
{
  if (raw) {
    _value = value;
  } else {
    _value = (_value & ~(_write_mask)) | (value & _write_mask);
  }
}

uint8_t LINBusRegister::read(void)
{
  uint8_t value;
  value = _value;

  return value;
}

void LINBusRegister::changeBit(uint8_t bitNum, bool newValue, bool raw)
{
  uint8_t bit_mask = BIT(bitNum);
  if (raw || (_write_mask & bit_mask)) {
    _value &= ~bit_mask;
    if (newValue) {
      _value |= bit_mask;
    }
  }
}


LINBusRegisterFile::LINBusRegisterFile()
{
  _head = 0;
}

void LINBusRegisterFile::addRegister(uint8_t index, LINBusRegister *register_)
{
  LINBusRegisterItem_t *item = new LINBusRegisterItem_t;
  LINBusRegisterItem_t *last = 0;
  LINBusRegisterItem_t *curr;
  item->index = index;
  item->reg = register_;

  for (curr = _head; curr && curr->index < index; last = curr, curr = curr->next);

  if (curr) {
    LINBusRegisterItem_t *prev = curr->prev;
    item->prev = prev;
    if (!prev) {
      _head = item; 
    } else {
      item->next = prev->next;
      prev->next = item;
    }
  } else {
    item->next = 0;
    if (last) {
      item->prev = last;
      last->next = item;
    } else {
      _head = item;
      item->prev = 0;      
    }
  }
}


LINBusRegister *LINBusRegisterFile::getRegister(uint8_t index)
{
  for (LINBusRegisterItem_t *curr = _head; curr; curr = curr->next) {
    if (curr->index == index) {
      return curr->reg;
    }
  }
  return 0;
}


LINBusRegisterFile linbusRegisterFile;
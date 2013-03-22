// Hardware Abstraction Layer for MultiWii (AVR/Arduino implementation)
// March 2013
//
// Changelog:
//  -
//
// feel free to play with it ;-)

#ifndef _MULTIWII_HAL_H
#define _MULTIWII_HAL_H

#include "MultiWii_HAL_AVR.h"


static struct {
  int16_t errors_count;
  void (* init)();
  void (* setFastClock)();
  void (* setSlowClock)();
  void (* stop)();
  void (* repStart)(uint8_t address);
  void (* write)(uint8_t data);
  uint8_t (* read)(uint8_t ack);
  void (* waitTransmission)();
} HAL_I2C = {
  0,
#if defined(PROMINI) || defined(PROMICRO) || defined(MEGA)
  avr_i2c_init,
  avr_i2c_setFastClock,
  avr_i2c_setSlowClock,
  avr_i2c_stop,
  avr_i2c_repStart,
  avr_i2c_write,
  avr_i2c_read,
  avr_i2c_waitTransmission
#elif defined(STM32)
  stm32_i2c_init,
  stm32_i2c_setFastClock,
  stm32_i2c_setSlowClock,
  stm32_i2c_stop,
  stm32_i2c_repStart,
  stm32_i2c_write,
  stm32_i2c_read,
  stm32_i2c_waitTransmission
#elif defined(X86)
  x86_i2c_init,
  x86_i2c_setFastClock,
  x86_i2c_setSlowClock,
  x86_i2c_stop,
  x86_i2c_repStart,
  x86_i2c_write,
  x86_i2c_read,
  x86_i2c_waitTransmission
#endif  
};

#endif

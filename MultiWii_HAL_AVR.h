// Hardware Abstraction Layer for MultiWii (AVR/Arduino implementation)
// March 2013
//
// Changelog:
//  -
//
// feel free to play with it ;-)

#ifndef _MULTIWII_HAL_AVR_H
#define _MULTIWII_HAL_AVR_H

#ifdef ARDUINO
    #if ARDUINO < 100
        #include "WProgram.h"
    #else
        #include "Arduino.h"
    #endif
#else
    #include "ArduinoWrapper.h"
#endif

// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || defined(TEENSY20)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif

#if defined(PROMINI)
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
#endif
#if defined(PROMICRO)
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#endif
#if defined(MEGA)
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#endif


void avr_i2c_setFastClock(void);
void avr_i2c_setSlowClock(void);
void avr_i2c_init(void);
void avr_i2c_stop(void);
void avr_i2c_repStart(uint8_t address);
void avr_i2c_write(uint8_t data);
uint8_t avr_i2c_read(uint8_t ack);
void avr_i2c_waitTransmission(void);

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



#if defined(PROMINI) || defined(PROMICRO) || defined(MEGA)
void avr_i2c_setFastClock() { TWBR = ((F_CPU / 400000L) - 16) / 2; }
void avr_i2c_setSlowClock() { TWBR = ((F_CPU / 100000L) - 16) / 2; }

void avr_i2c_init() {
  TWSR = 0;                                    // no prescaler => prescaler = 1
  i2c_setSlowClock();						   // just set to always slow (sensors who can handle fast, set it accordingly)
  //TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}



void avr_i2c_stop() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void avr_i2c_waitTransmission() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      //neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      HAL_I2C.errors_count++;
      break;
    }
  }
}

	
void avr_i2c_repStart(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  i2c_waitTransmission();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  i2c_waitTransmission();                       // wail until transmission completed
}
void avr_i2c_write(uint8_t data) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  i2c_waitTransmission();
}
uint8_t avr_i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  i2c_waitTransmission();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}


#endif







#endif // _MULTIWII_HAL_AVR_H

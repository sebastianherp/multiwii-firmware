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



class MultiWii_HAL_AVR {
  public:
    static void i2c_init();
    static void i2c_setFastClock() { TWBR = ((F_CPU / 400000L) - 16) / 2; }
    static void i2c_setSlowClock() { TWBR = ((F_CPU / 100000L) - 16) / 2; }
    static void i2c_stop();
    static void i2c_repStart(uint8_t address);
    static void i2c_write(uint8_t data);
    static uint8_t i2c_read(uint8_t ack);
    static int16_t i2c_errors_count;
    
  private:
    static void i2c_waitTransmission();

};

#endif

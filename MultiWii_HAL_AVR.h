// Hardware Abstraction Layer for MultiWii (AVR/Arduino implementation)
// March 2013
//
// Changelog:
//  -
//
// feel free to play with it ;-)

#ifndef _MULTIWII_HAL_AVR_H
#define _MULTIWII_HAL_AVR_H

#include "MultiWii_HAL.h"

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


class MultiWii_HAL_AVR : public MultiWii_HAL {
    public:
        MultiWii_HAL_AVR();
};

class AVR_Driver_I2C : public Driver_I2C {
	public:
		void init(bool enablePullUps);
		void writeReg(uint8_t add, uint8_t reg, uint8_t val);
		uint8_t readReg(uint8_t add, uint8_t reg);
		size_t readRegToBuffer(uint8_t add, uint8_t reg, void *buf, size_t size);
		size_t readToBuffer(uint8_t add, void *buf, size_t size);
		void setFastClock();
		void setSlowClock();
                void setEnablePullUps(bool active);
		
	private:	
		void stop();
		void repStart(uint8_t address);
		void write(uint8_t data);
		uint8_t read(uint8_t ack);
		uint8_t readAck();
		uint8_t readNak();
		void waitTransmission();
		
		uint32_t neutralizeTime;
};


#endif

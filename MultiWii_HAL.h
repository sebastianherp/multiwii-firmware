// Hardware Abstraction Layer for MultiWii
// March 2013
//
// Changelog:
//  -
//
// feel free to play with it ;-)

#ifndef _MULTIWII_HAL_H
#define _MULTIWII_HAL_H

#include <avr/io.h>
#include <avr/pgmspace.h>

class Driver_I2C {

public:
	virtual void init(bool enablePullUps);
	virtual void writeReg(uint8_t add, uint8_t reg, uint8_t val) = 0;
	virtual uint8_t readReg(uint8_t add, uint8_t reg) = 0;
	virtual size_t readToBuffer(uint8_t add, void *buf, size_t size) = 0;
	virtual size_t readRegToBuffer(uint8_t add, uint8_t reg, void *buf, size_t size) = 0;
	virtual void setFastClock();
	virtual void setSlowClock();
	virtual void stop();
	virtual void repStart(uint8_t address);
	virtual void write(uint8_t data);
	virtual uint8_t read(uint8_t ack);
	virtual uint8_t readAck();
	virtual uint8_t readNak();

	int16_t errors_count;
	
};

class MultiWii_HAL {
    public:
        MultiWii_HAL(Driver_I2C* _i2c) : i2c(_i2c) {}
        
		// "drivers"
		Driver_I2C* i2c;
};

#endif

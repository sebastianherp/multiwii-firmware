// Hardware Abstraction Layer for MultiWii
// March 2013
//
// Changelog:
//  -
//
// feel free to play with it ;-)

#include "MultiWii_HAL_AVR.h"

static AVR_Driver_I2C avrDriverI2C;

MultiWii_HAL_AVR::MultiWii_HAL_AVR() : MultiWii_HAL(&avrDriverI2C) {}


void AVR_Driver_I2C::init(bool enablePullUps) {
  if(enablePullUps) {
    I2C_PULLUPS_ENABLE
  } else {
    I2C_PULLUPS_DISABLE
  }
  TWSR = 0;                                    // no prescaler => prescaler = 1
  setSlowClock();						   // just set to always slow (sensors who can handle fast, set it accordingly)
  //TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void AVR_Driver_I2C::stop() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}
	
void AVR_Driver_I2C::repStart(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmission();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmission();                       // wail until transmission completed
}
void AVR_Driver_I2C::write(uint8_t data) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmission();
}
uint8_t AVR_Driver_I2C::read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmission();
  uint8_t r = TWDR;
  if (!ack) stop();
  return r;
}
uint8_t AVR_Driver_I2C::readAck() {
	return read(1);
}
uint8_t AVR_Driver_I2C::readNak() {
	return read(0);
}
void AVR_Driver_I2C::waitTransmission() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      errors_count++;
      break;
    }
  }
}
void AVR_Driver_I2C::setFastClock() {
	TWBR = ((F_CPU / 400000L) - 16) / 2;
}

void AVR_Driver_I2C::setSlowClock() {
	TWBR = ((F_CPU / 100000L) - 16) / 2;
}

void setEnablePullUps(bool active) {
  
}
	
size_t AVR_Driver_I2C::readToBuffer(uint8_t add, void *buf, size_t size) {
  repStart((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}
size_t AVR_Driver_I2C::readRegToBuffer(uint8_t add, uint8_t reg, void *buf, size_t size) {
  repStart(add<<1); // I2C write direction
  write(reg);        // register selection
  return readToBuffer(add, buf, size);
}

void AVR_Driver_I2C::writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  repStart(add<<1); // I2C write direction
  write(reg);        // register selection
  write(val);        // value to write in register
  stop();
}
uint8_t AVR_Driver_I2C::readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  readRegToBuffer(add, reg, &val, 1);
  return val;
}

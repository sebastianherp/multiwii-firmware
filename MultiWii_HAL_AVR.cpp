// Hardware Abstraction Layer for MultiWii (AVR/Arduino implementation)
// March 2013
//
// Changelog:
//  -
//
// feel free to play with it ;-)

#include "MultiWii_HAL.h"
#include "MultiWii_HAL_AVR.h"

void avr_i2c_setFastClock() { TWBR = ((F_CPU / 400000L) - 16) / 2; }
void avr_i2c_setSlowClock() { TWBR = ((F_CPU / 100000L) - 16) / 2; }

void avr_i2c_init() {
  TWSR = 0;                                    // no prescaler => prescaler = 1
  avr_i2c_setSlowClock();						   // just set to always slow (sensors who can handle fast, set it accordingly)
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
  avr_i2c_waitTransmission();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  avr_i2c_waitTransmission();                       // wail until transmission completed
}
void avr_i2c_write(uint8_t data) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  avr_i2c_waitTransmission();
}
uint8_t avr_i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  avr_i2c_waitTransmission();
  uint8_t r = TWDR;
  if (!ack) avr_i2c_stop();
  return r;
}

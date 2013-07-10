#ifndef SERIAL_H_
#define SERIAL_H_

#include "config.h"
#include "def.h"

#if defined(MEGA)
  #define UART_NUMBER 4
#elif defined(PROMICRO)
  #define UART_NUMBER 2
#else
  #define UART_NUMBER 1
#endif

#if defined(GPS_SERIAL)
  #define RX_BUFFER_SIZE 256 // 256 RX buffer is needed for GPS communication (64 or 128 was too short)
#else
  #define RX_BUFFER_SIZE 64
#endif
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t indRX[UART_NUMBER];

void serialCom();
void SerialOpen(uint8_t port, uint32_t baud);
uint8_t SerialRead(uint8_t port);
void SerialWrite(uint8_t port,uint8_t c);
uint8_t SerialAvailable(uint8_t port);
void debugmsg_append_str(const char *str);
void SerialEnd(uint8_t port);
uint8_t SerialPeek(uint8_t port);
#if defined(GPS_SERIAL)
  bool SerialTXfree(uint8_t port);
#endif
void serializeChar(uint8_t port, uint8_t a);
uint32_t read32(uint8_t port);
uint16_t read16(uint8_t port);
uint8_t read8(uint8_t port);
void UartSendData(uint8_t port);
void evaluateOtherData(uint8_t sr);

#endif /* SERIAL_H_ */

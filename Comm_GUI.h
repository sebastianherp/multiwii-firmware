#ifndef COMM_GUI_H_
#define COMM_GUI_H_

#include "Serial.h"

static uint8_t checksum[UART_NUMBER];
uint8_t guiParser(uint8_t port, uint8_t c);

#endif /* COMM_GUI_H_ */

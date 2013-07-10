#ifndef COMM_GUI_H_
#define COMM_GUI_H_

#include "Serial.h"

uint8_t guiParser(uint8_t port, uint8_t c);
void debugmsg_append_str(const char *str);

#endif /* COMM_GUI_H_ */


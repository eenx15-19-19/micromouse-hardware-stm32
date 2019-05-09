#ifndef COM_H
#define COM_H

#include "main.h"

extern uint8_t s;
extern uint8_t dataBufferUART[200];

void sendWalls(uint8_t wall[]);
void readLine(void);


#endif

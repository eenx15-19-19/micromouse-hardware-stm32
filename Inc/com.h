#ifndef COM_H
#define COM_H

#include "main.h"

extern uint8_t s;

void sendWalls(uint8_t wall[]);
void receiveSingle(void);
void receiveMulti(uint8_t map, UART_HandleTypeDef *uart);

void recieveSingleCommand(void);


#endif

#include "main.h"
#include "com.h"

uint8_t s = 1;
uint8_t sent[6] = {0};
void sendWalls(uint8_t wall[])
{
	
	sent[0] = s;
	
	for(int i = 1; i < 5; i++)
	{
		sent[i]=wall[i-1];
	}
	sent[5] = '\n';
	HAL_UART_Transmit(&huart3, sent, 6, 100);
}
/*
void receiveSingle()
{
    HAL_UART_Receive(&huart3, &direction, 1, 100);
}

void receiveMulti(uint8_t map, UART_HandleTypeDef *uart)
{
    HAL_UART_Receive(uart, commands , 100, 100);
}
*/





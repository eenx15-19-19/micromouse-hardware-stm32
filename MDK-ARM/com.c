#include "main.h"
#include "stm32f1xx_hal_def.h"


void sendWall(uint8_t wall[], UART_HandleTypeDef *uart, uint8_t sent[], uint8_t s)
{
	sent[0]=s;
	for(int i=1; i<4; i++)
	{
		sent[i]=wall[i-1];
	}
	HAL_UART_Transmit(uart, sent, sizeof(sent)/sizeof(sent[0]), 100);
}






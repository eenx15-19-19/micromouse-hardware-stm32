#include "main.h"
#include "com.h"

uint8_t s = 1;
uint8_t sent[6] = {0};

//For readline
uint8_t data = 0;
uint8_t endIndex = 0;
uint8_t dataBufferUART[200] = {'0'};


void sendWalls(uint8_t wall[]){
	
	sent[0] = s;
	
	for(int i = 1; i < 5; i++)
	{
		sent[i]=wall[i-1];
	}
	sent[5] = '\n';
	HAL_UART_Transmit(&huart3, sent, 6, 100);
}

void readLine(void){
	data = '0';
	for(int i = 0; i < 200; i++){
		dataBufferUART[i] = '0';
	}
	endIndex = 0;
	
	while(data != '\n' /*'\n'*/){
		data = '0';
		HAL_UART_Receive(&huart3, &data, 1, 10);
		
		
		if(data != '0' && data != '\n'){
			if(endIndex < 200){
				dataBufferUART[endIndex] = data;
				endIndex++;
			}else{
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				while(1);	//You done fucked up boi
			}
		}
		
	}
}








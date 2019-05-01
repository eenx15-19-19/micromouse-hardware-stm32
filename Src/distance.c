#include "main.h"
#include "distance.h"
#include "adc.h"

double value;
char wall[3];
double value_adc[3];
double p1[3]={-685.4, -89.66, 93.38};
double p2[3]={11430.0, 875.0, 7481.0};
double q1[3]={4.008, 2.18, 2.519};

void equation(double p1[], double q1[], double p2[], double arr[]){
	// get value of p1 and q1 from lineraisation from matlabs cftool, use rational with numerator degree=0, denominator degree=1. 
	//the array is the array with measurment data from IR sensors(3 of them). 
	for (int i=0; i<3; i++){
		arr[i] = (p1[i]*arr[i]+p2[i])/(arr[i]+q1[i]);
	}
}

void sensorDist(double raw_distance[], ADC_HandleTypeDef hadc){
	HAL_ADC_Start(&hadc);
	
	HAL_ADC_PollForConversion(&hadc, 1);
	raw_distance[0] = HAL_ADC_GetValue(&hadc); //left
	
	HAL_ADC_PollForConversion(&hadc, 1);
	raw_distance[1] = HAL_ADC_GetValue(&hadc); //middle
	
	HAL_ADC_PollForConversion(&hadc, 1);
	raw_distance[2] = HAL_ADC_GetValue(&hadc); //right
	HAL_ADC_Stop(&hadc);
}

void wallDet(double raw_distance[], char walls[], ADC_HandleTypeDef hadc)
{
	// takes two arguments, the first a double array where the three sensors data is stored the second a char array where the walls are stored
	sensorDist(raw_distance, hadc);
	equation(p1, p2, q1, raw_distance);
	if (raw_distance[1]+raw_distance[2]>8)
	{
		if (raw_distance[2]>8)
			walls[2]='n';
		else
			walls[2]='y';
		if (raw_distance[1]>8)
			walls[1]='n';
		else
			walls[1]='y';
	}
	else
	{
		walls[1]='y';
		walls[2]='y';
	}
	
	if (raw_distance[0]>8)
		walls[0]='n';
	else
		walls[0]='y';
}

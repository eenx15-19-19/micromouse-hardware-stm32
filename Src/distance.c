#include "main.h"
#include "distance.h"
#include "adc.h"
#include "motor_control.h"

uint8_t walls[4] = "0111";
float p1[3]={-5.579, -1.668, -3.89};
float p2[3]={14720, 7745, 10370};
float q1[3]={955.9, 13.31, 101.1};
float calcDistances[3];

void calcSensorDistances(void){
	// get value of p1 and q1 from lineraisation from matlabs cftool, use rational with numerator degree=0, denominator degree=1. 
	//the array is the array with measurment data from IR sensors(3 of them). 
	for(int i=0; i<3; i++){
		calcDistances[i] = (p1[i]*sensorValues[i] + p2[i]) / (sensorValues[i]+q1[i]);
	}
}


void wallDet(void){
	// takes two arguments, the first a double array where the three sensors data is stored the second a char array where the walls are stored
	//calcSensorDistances();
	
	if (calcDistances[0] > 8)
		walls[2] = '0';
	else
		walls[2] = '1';
	if (calcDistances[2]>8)
		walls[3] = '0';
	else
		walls[3] = '1';
	if (calcDistances[1]>8)
		walls[0] = '0';
	else
		walls[0] = '1';
	
	walls[1] = '0';
}


void calibrateSensors(void){	// Measure the sensorMiddle values and store them
	HAL_Delay(300); // Stabalize
	
	leftSensorMiddleValue = calcDistances[0];
	frontSensorTreshhold = calcDistances[1];
	rightSensorMiddleValue = calcDistances[2];
}

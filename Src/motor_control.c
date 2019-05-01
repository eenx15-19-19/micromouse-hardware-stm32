#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "motor_control.h"
#include "pwm.h"


// Used for control loop
int leftEncoder;
int rightEncoder;
int leftEncoderOld;
int rightEncoderOld;
int leftEncoderChange;
int rightEncoderChange;
int leftEncoderCount;
int rightEncoderCount;
float encoderCount;
float rotationCount;
float oldEncoderCount;
float oldRotationCount;
float encoderChange;
float rotationChange;
float distanceLeft;
float rotationLeft;
int leftBaseSpeed;
int rightBaseSpeed;

float curSpeedX = 0;
float curSpeedW = 0;
int targetSpeedX = 0;
int targetSpeedW = 0;
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;
float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
int posPwmX = 0;
int posPwmW = 0;
float kpX = 2, kdX = 4;
float kpW = 1, kdW = 12;//used in straight
float accX = 300;// cm/s^2 => 0.1 m/s^2 
float decX = 300; 
float accW = 1; //cm/s^2
float decW = 1;

int oneCellDistance = distanceToCounts(180);
int quarterTurn = rotToCounts(90);

void motorSetup(void){
	/* Start the encoder timer */
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
		
		/*Enable the motor*/
		// Not needded anymore since it is hardwired to 3.3V.
		//HAL_GPIO_WritePin(H_Bridge_Enable_GPIO_Port, H_Bridge_Enable_Pin, GPIO_PIN_SET);
		/* Orientation Right motor -> CW */
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		/* Orientation Left motor -> CCW */
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}


void speedProfile(void)
{	
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();      
}


void getEncoderStatus(void)
{
	leftEncoder = TIM4->CNT;//read current encoder ticks from register of 32 bit general purpose timer 2
	rightEncoder = TIM2->CNT;//read current encoder ticks from register of 32 bit general purpose timer 5

	leftEncoderChange = leftEncoder - leftEncoderOld;
	rightEncoderChange = rightEncoder - rightEncoderOld;
	
	// Overflow correction - Left Encoder
	if(leftEncoderChange > 0xF000){
		leftEncoderChange = leftEncoder - 0xFFFF - leftEncoderOld;
	}else if(leftEncoderChange < - 0xF000){
		leftEncoderChange = 0xFFFF - leftEncoderOld + leftEncoder;
	}
	
	// Overflow correction - Right Encoder
	if(rightEncoderChange > 0xF000){
		rightEncoderChange = rightEncoder - 0xFFFF - rightEncoderOld;
	}else if(rightEncoderChange < - 0xF000){
		rightEncoderChange = 0xFFFF - rightEncoderOld + rightEncoder;
	}
	
	encoderChange = ( rightEncoderChange + leftEncoderChange)/2.0;	 
	rotationChange = (rightEncoderChange - leftEncoderChange)/2.0;

	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;
					
	leftEncoderCount += leftEncoderChange;
	rightEncoderCount += rightEncoderChange;
	encoderCount =  (leftEncoderCount+rightEncoderCount)/2.0;	
	rotationCount = (rightEncoderCount - leftEncoderCount)/2.0;
	
	distanceLeft -= encoderChange;// update distanceLeft	
	rotationLeft -= rotationChange;
}



void updateCurrentSpeed(void)
{
	if(curSpeedX < targetSpeedX)
	{
		curSpeedX += (float)(speedToCounts(accX*2)/100);
		if(curSpeedX > targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	else if(curSpeedX > targetSpeedX)
	{
		curSpeedX -= (float)speedToCounts(decX*2)/100;
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	if(curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}
	else if(curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}	
}


void calculateMotorPwm(void) // encoder PD controller
{	
	
    /* simple PD loop to generate base speed for both motors */	
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;	

	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW += curSpeedW - encoderFeedbackW;
	
	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
	
	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;
	
	leftBaseSpeed = posPwmX - posPwmW;
	rightBaseSpeed = posPwmX + posPwmW;

	setLeftPWM(leftBaseSpeed);
	setRightPWM(rightBaseSpeed);	
}


int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)//speed are in encoder counts/ms, dist is in encoder counts 
{
	if (curSpd<0) curSpd = -curSpd;
	if (endSpd<0) endSpd = -endSpd;
	if (dist<0) dist = 1;//-dist;
	if (dist == 0) dist = 1;  //prevent divide by 0

	int a = countsToSpeed((curSpd*curSpd - endSpd*endSpd)*100/dist/4/2);
	
	if(a < 0)
		a = -a;
	
	
	return a;
	
	
	//calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
	//use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
	//because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
}

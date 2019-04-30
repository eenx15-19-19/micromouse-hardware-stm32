#ifndef motor_control_h
#define motor_control_h

#include <stdint.h>

#define WHEEL_TO_WHEEL 89.0 // Center of wheel to the center of the other wheel. 
#define WHEEL_D 30.6
#define CPR 2048.0
#define PI 3.141592
#define GEARING (56.0/10.0)

extern int targetSpeedX;
extern int targetSpeedW;
extern float decX; 
extern float decW;

// Needed for running in main
extern float distanceLeft;
extern float rotationLeft;
extern float curSpeedX;
extern float curSpeedW;
extern float encoderCount;
extern float rotationCount;
extern float oldEncoderCount;
extern float oldRotationCount;
extern int oneCellDistance;
extern int quarterTurn;

void motorSetup(void);
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int needToDecelerate(int32_t distanceLeft, int16_t currentSpeed, int16_t endSpeed);

// 1000 is for s => ms
#define speedToCounts(speed) ((speed) / (PI * WHEEL_D) / 1000.0 * CPR * GEARING)
#define countsToSpeed(counts) (counts * (PI * WHEEL_D) * 1000.0 / CPR / GEARING)
#define distanceToCounts(distance) (distance / (PI * WHEEL_D) * CPR * GEARING)
#define countsToDistance(counts) (counts * PI * WHEEL_D / CPR / GEARING)

#define degreesToRadians(deg) (deg / 360.0 * 2 * PI)
//#define rotSpeedToCounts(rotSpeed) ( speedToCounts(degreesToRadians(rotSpeed) * WHEEL_D/2.0) )
#define rotToCounts(deg) ( distanceToCounts(deg / 360.0 * (PI * WHEEL_TO_WHEEL)) ) 
#define countsToRot(counts) ( countsToDistance(counts) / (PI * WHEEL_TO_WHEEL) * 360.0  ) 
#endif 

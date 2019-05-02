#ifndef DISTANCE_H
#define DISTANCE_H

#include "main.h"

extern float calcDistances[3];

void calibrateSensors(void);
void calcSensorDistances(void);
void wallDet(float arr1[], char arr2[]);


#endif

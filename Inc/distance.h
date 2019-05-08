#ifndef DISTANCE_H
#define DISTANCE_H

#include "main.h"
extern uint8_t walls[];
extern float calcDistances[3];

void calibrateSensors(void);
void calcSensorDistances(void);
void wallDet(void);


#endif

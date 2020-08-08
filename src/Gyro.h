/* Turning Stuff for Zumo */
#pragma once

#include <Zumo32U4.h>

// constant representing a turn of 45 degrees, conversion from
// degrees to internal angles
const int32_t turnAngle45 = 0x20000000;

extern Zumo32U4ButtonA buttonA;
extern Zumo32U4LCD lcd;
extern L3G gyro;

// Found in Gyro.cpp
void gyroInit();
void gyroReset();
void gyroUpdate();
extern uint32_t turnAngle;
extern int16_t turnRate;


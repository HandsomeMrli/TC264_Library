#ifndef _PRINT_H_
#define _PRINT_H_

#include "zf_common_headfile.h"
#include "define.h"

void printAcc();
void printGyro();
void printEularAngle(const FusionEuler *euler);
void printAllAttitudeSolution(const FusionEuler *euler);
void printMotorSpeed(int16 motorLeftSpeed, int16 motorRightSpeed, int16 motorBottomSpeed);

#endif
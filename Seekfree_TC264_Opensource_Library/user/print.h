#ifndef _PRINT_H_
#define _PRINT_H_

#include "zf_common_headfile.h"
#include "define.h"

void printEularAngle(const FusionEuler *euler);
void printMotorSpeed();

extern int16 motorLeftSpeed;
extern int16 motorRightSpeed;
extern int16 motorBottomSpeed;

#endif
#ifndef _PID_H_
#define _PID_H_

#include "define.h"

typedef struct{
    int32_t pCorr, iCorr, dCorr;
    int32_t error[3];
    int32_t errorInt;
    int32_t errorIntMax;
    int32_t pCoef, iCoef, dCoef;
    int32_t target;
    int32_t measurement;
    int32_t deltaOutput;
}PIDValue;

void __initPID(PIDValue *pid,int32_t pCoef, int32_t iCoef, int32_t dCoef, int32_t target, int32_t errorIntMax);
int32_t __updatePID(PIDValue *pid);

#endif
#include "pid.h"

void __initPID(PIDValue *pid,int32_t pCoef, int32_t iCoef, int32_t dCoef, int32_t target, int32_t errorIntMax){
    pid->pCorr = pCoef; pid->iCorr = iCoef; pid->dCorr = dCoef;
    pid->error[0] = 0; pid->error[1] = 0; pid->error[2] = 0;
    pid->errorInt = 0;
    pid->errorIntMax = errorIntMax;
    pid->pCoef = pCoef; pid->iCoef = iCoef; pid->dCoef = dCoef;
    pid->target = target;
    pid->measurement = 0;
    pid->deltaOutput = 0;
}

/* 位置式PID */
int32_t __updatePID(PIDValue *pid){
    
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->target - pid->measurement;
    
    pid->errorInt += pid->error[0];
    pid->errorInt = (pid->errorInt > pid->errorIntMax) ? pid->errorIntMax : pid->errorInt;
    pid->errorInt = (pid->errorInt < -pid->errorIntMax) ? -pid->errorIntMax : pid->errorInt;

    pid->pCorr = pid->pCoef * pid->error[0] / 10;
    pid->iCorr = pid->iCoef * pid->errorInt / 10;
    pid->dCorr = pid->dCoef * (pid->error[0] - pid->error[1]) / 10;

    pid->deltaOutput = pid->pCorr + pid->iCorr + pid->dCorr;

    return pid->deltaOutput;

}

/* 增量式PID */
// int32_t updatePID(PIDValue *pid){

//     pid->error[0] = pid->target - pid->measurement;

//     pid->pCorr = pid->pCoef  * (pid->error[0] - pid->error[1]) / 10;
//     pid->iCorr = pid->iCoef  * pid->error[0] / 10;
//     pid->dCorr = pid->dCoef  * (pid->error[0] - 2 * pid->error[1] + pid->error[2]) / 10;

//     pid->error[2] = pid->error[1];
//     pid->error[1] = pid->error[0];
//     pid->deltaOutput = pid->pCorr + pid->iCorr + pid->dCorr;

//     return pid->deltaOutput;
    
// }

// void test(){
//     PIDValue velocityPID, anglePID, angleVelocityPID;
//     initPID(&velocityPID, 10, 0, 0, 0, 200);
//     initPID(&anglePID, 10, 10, 10, 0, 200);
//     initPID(&angleVelocityPID, 10, 10, 10, 0, 200);

//     velocityPID.measurement = 2;
//     updatePID(&velocityPID);
//     anglePID.target += velocityPID.deltaOutput;
    
//     anglePID.measurement = 20; // 由omega积分得到
//     updatePID(&anglePID);
//     angleVelocityPID.target += anglePID.deltaOutput;

//     angleVelocityPID.measurement = 30; // 由陀螺仪得到
//     updatePID(&angleVelocityPID);
//     // 底轮控制量 += angleVelocityPID.deltaOutput;

// }
#include "pid.h"




void initPIDValue(PIDValue *pid,int32_t pCoef, int32_t iCoef, int32_t dCoef, int32_t target){
    pid->pCorr = 0; pid->iCorr = 0; pid->dCorr = 0;
    pid->error[0] = 0; pid->error[1] = 0; pid->error[2] = 0;
    pid->pCoef = pCoef; pid->iCoef = iCoef; pid->dCoef = dCoef;
    pid->target = target;
    pid->measurement = 0;
    pid->deltaOutput = 0;
}


int32_t updatePID(PIDValue *pid){

    pid->error[0] = pid->target - pid->measurement;

    pid->pCorr = pid->pCoef  * (pid->error[0] - pid->error[1]) / 10;
    pid->iCorr = pid->iCoef  * pid->error[0] / 10;
    pid->dCorr = pid->dCoef  * (pid->error[0] - 2 * pid->error[1] + pid->error[2]) / 10;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->deltaOutput = pid->pCorr + pid->iCorr + pid->dCorr;

    return pid->deltaOutput;
    
}

void test(){
    PIDValue velocityPID, anglePID, angleVelocityPID;
    initPIDValue(&velocityPID, 10, 0, 0, 0);
    initPIDValue(&anglePID, 10, 10, 10, 0);
    initPIDValue(&angleVelocityPID, 10, 10, 10, 0);

    velocityPID.measurement = 2;
    updatePID(&velocityPID);
    anglePID.target += velocityPID.deltaOutput;
    
    anglePID.measurement = 20; // 由omega积分得到
    updatePID(&anglePID);
    angleVelocityPID.target += anglePID.deltaOutput;

    angleVelocityPID.measurement = 30; // 由陀螺仪得到
    updatePID(&angleVelocityPID);
    // 底轮控制量 += angleVelocityPID.deltaOutput;

}
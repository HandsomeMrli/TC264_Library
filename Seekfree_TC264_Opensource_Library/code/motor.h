#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"
#include "define.h"
#include "pid.h"

typedef struct {
    PIDValue pid;
    uint32_t freq;
    int32_t pwm; // 同时表示大小和方向
    pwm_channel_enum pwmChannel;
    gpio_pin_enum dirPin;
}Motor;

typedef enum {
    PLUS,
    MINUS,
    ASSIGN,
    OPPOSE
}Operation;

extern Motor motorLeft, motorRight, motorBottom;
extern PIDValue velPIDy, velPIDl, velPIDr;
extern PIDValue angPIDx, angPIDy, angPIDz;
extern PIDValue angVelPIDx, angVelPIDy, angVelPIDz; 

void __initMotor(Motor *motor, uint32 freq, int32 pwm, 
        pwm_channel_enum pwmChannel, gpio_pin_enum dirPin,
        int32_t pCoef, int32_t iCoef, int32_t dCoef, int32_t target, int32_t errorIntMax);
void initMotors();
void __updateMotor(Motor *motor);
void setMotor(Motor *motor, Operation op, int32_t offset);
void updateMotors(
        int16 motorLeftSpeed, int16 motorRightSpeed, int16 motorBottomSpeed, 
        int32 cameraSpeedTarget, int32 cameraTurnTarget, 
        int32 pitchX, int32 rollY, int32 yawZ,
        int32 angVelX, int32 angVelY, int32 angVelZ);

#endif
#ifndef _MOTOR_H_
#define _MOTOR_H

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

Motor motorLeft, motorRight, motorBottom;
PIDValue velPIDy, velPIDl, velPIDr;
PIDValue angPIDx, angPIDy, angPIDz;
PIDValue angVelPIDx, angVelPIDy, angVelPIDz; 

void initMotors();
void __initMotor(Motor *motor, uint32 freq, int32 pwm, 
        pwm_channel_enum pwmChannel, gpio_pin_enum dirPin,
        int32_t pCoef, int32_t iCoef, int32_t dCoef, int32_t target, int32_t errorIntMax);
void setMotor(Motor *motor, Operation op, int32_t offset);

#endif
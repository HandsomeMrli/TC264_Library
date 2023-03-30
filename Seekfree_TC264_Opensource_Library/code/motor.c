#include "motor.h"

void initMotors(){

    __initMotor(&motorLeft, 17000, 1000, WHEEL_1_PWM_PIN, WHEEL_1_DIR_PIN, 5, 1, 1, 0, 300);
    gpio_init(WHEEL_1_DIR_PIN, GPO, 0, GPO_PUSH_PULL);
    pwm_init(WHEEL_1_PWM_PIN, 17000, 1000);
    encoder_quad_init(WHEEL_1_ENCODER, WHEEL_1_ENCODER_A_PIN, WHEEL_1_ENCODER_B_PIN);
    
    __initMotor(&motorLeft, 17000, 1000, WHEEL_1_PWM_PIN, WHEEL_1_DIR_PIN, 5, 1, 1, 0, 300);
    gpio_init(WHEEL_2_DIR_PIN, GPO, 0, GPO_PUSH_PULL);
    pwm_init(WHEEL_2_PWM_PIN, 5000, 5000);
    encoder_quad_init(WHEEL_2_ENCODER, WHEEL_2_ENCODER_A_PIN, WHEEL_2_ENCODER_B_PIN);

    // TODO:底轮的电机还未初始化，需要得知其PWM_PIN与DIR_PIN

}

void __initMotor(Motor *motor, uint32 freq, int32 pwm, pwm_channel_enum pwmChannel, gpio_pin_enum dirPin, int32_t pCoef, int32_t iCoef, int32_t dCoef, int32_t target, int32_t errorIntMax){
    __initPID(&(motor->pid), pCoef, iCoef, dCoef, target, errorIntMax);
    motor->freq = freq;
    motor->pwm = pwm;
    motor->pwmChannel = pwmChannel;
    motor->dirPin = dirPin;
}

void setMotor(Motor *motor, Operation op, int32_t offset){
    int32_t pwmTemp;
    switch (op){
        case PLUS:
            pwmTemp = motor->pwm + offset;
            motor->pwm = (pwmTemp > WHEEL_PWM_MAX) ? WHEEL_PWM_MAX : pwmTemp;
            break;
        case MINUS:
            pwmTemp = motor->pwm - offset;
            motor->pwm = (pwmTemp < -WHEEL_PWM_MAX) ? -WHEEL_PWM_MAX : pwmTemp;
            break;
        case ASSIGN:
            pwmTemp = minValue(offset, WHEEL_PWM_MAX);
            pwmTemp = maxValue(offset, -WHEEL_PWM_MAX);
            motor->pwm = pwmTemp;
            break;
        case OPPOSE:
            motor->pwm = -motor->pwm;
            break;
    }
    pwm_set_duty(motor->pwmChannel, (uint32)absValue(motor->pwm));
    gpio_set_level(motor->dirPin, (uint8)(motor->pwm >= 0));
}

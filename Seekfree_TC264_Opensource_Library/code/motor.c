#include "motor.h"

Motor motorLeft, motorRight, motorBottom;
PIDValue velPIDy, velPIDl, velPIDr;
PIDValue angPIDx, angPIDy, angPIDz;
PIDValue angVelPIDx, angVelPIDy, angVelPIDz;

void updatexxx(){
    
    // 速度环更新
    velPIDl.target = 0; velPIDl.measurement = 左动量轮转速; __updatePID(&velPIDl);
    velPIDr.target = 0; velPIDr.measurement = 右动量轮转速; __updatePID(&velPIDr);
    velPIDy.target = 摄像头目标值; velPIDy.measurement = 底轮转速; __updatePID(&velPIDy);
    
    // 角度环更新
    angPIDx.target = velPIDl.deltaOutput + velPIDr.deltaOutput; angPIDx.measurement = 陀螺仪欧拉角x; __updatePID(&angPIDx); // TODO:左右两轮的deltaOutput是相加还是相减?
    angPIDy.target = velPIDy.deltaOutput; angPIDy.measurement = 陀螺仪欧拉角y; __updatePID(&angPIDy);
    angPIDz.target = 摄像头目标值;          angPIDz.measurement = 陀螺仪欧拉角z; __updatePID(&angPIDz);

    // 角速度环更新
    angVelPIDx.target = angPIDx.deltaOutput; angVelPIDx.measurement = icm20602_gyro_x(陀螺仪角速度x); __updatePID(&angVelPIDx);   
    angVelPIDy.target = angPIDy.deltaOutput; angVelPIDy.measurement = icm20602_gyro_y(陀螺仪角速度y); __updatePID(&angVelPIDy);   
    angVelPIDz.target = angPIDz.deltaOutput; angVelPIDz.measurement = icm20602_gyro_z(陀螺仪角速度z); __updatePID(&angVelPIDz);   

    // PWM更新
    setMotor(&motorLeft, ASSIGN, angVelPIDx.deltaOutput + angVelPIDz.deltaOutput); // TODO:左右两轮的deltaOutput是相加还是相减?
    setMotor(&motorRight, ASSIGN, angVelPIDx.deltaOutput + angVelPIDz.deltaOutput); // TODO:左右两轮的deltaOutput是相加还是相减?
    setMotor(&motorBottom, ASSIGN, angVelPIDy.deltaOutput);
}


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
    __updateMotor(motor);
}

void __updateMotor(Motor *motor){
    pwm_set_duty(motor->pwmChannel, (uint32)absValue(motor->pwm));
    gpio_set_level(motor->dirPin, (uint8)(motor->pwm >= 0));
}
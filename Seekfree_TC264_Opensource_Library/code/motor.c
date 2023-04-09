#include "motor.h"

Motor motorLeft, motorRight, motorBottom;
PIDValue velPIDy, velPIDl, velPIDr;
PIDValue angPIDx, angPIDy, angPIDz;
PIDValue angVelPIDx, angVelPIDy, angVelPIDz;

/**
 * @brief 仅初始化Motor结构体,不负责初始化引脚与PWM
*/
void __initMotor(Motor *motor, uint32 freq, int32 pwm,
        pwm_channel_enum pwmChannel, gpio_pin_enum dirPin,
        int32_t pCoef, int32_t iCoef, int32_t dCoef, int32_t target, int32_t errorIntMax){
    __initPID(&(motor->pid), pCoef, iCoef, dCoef, target, errorIntMax);
    motor->freq = freq;
    motor->pwm = pwm;
    motor->pwmChannel = pwmChannel;
    motor->dirPin = dirPin;
}

void initMotors(){
    /*
        WHEEL_1: 方向引脚为正时,角动量=(+,0,+),原测速为负,经过人为纠正变为正
        WHEEL_2: 方向引脚为正时,角动量=(-,0,+),原测速为负,经过人为纠正变为正,导致roll减小
    */

    __initMotor(&motorLeft, 17000, 0, WHEEL_1_PWM_PIN, WHEEL_1_DIR_PIN, 10, 3, 1, 0, 300);
    __initMotor(&motorRight, 17000, 0, WHEEL_2_PWM_PIN, WHEEL_2_DIR_PIN, 10, 3, 1, 0, 300);
    __initMotor(&motorBottom, 17000, 0, WHEEL_3_PWM_PIN, WHEEL_3_DIR_PIN, 10, 3, 1, 0, 300);

    // 初始化方向引脚
    gpio_init(WHEEL_1_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(WHEEL_2_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(WHEEL_3_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // 初始化PWM信号
    pwm_init(WHEEL_1_PWM_PIN, motorRight.freq, PWM_DUTY_MAX);
    pwm_init(WHEEL_2_PWM_PIN, motorLeft.freq, PWM_DUTY_MAX);
    pwm_init(WHEEL_3_PWM_PIN, motorBottom.freq, PWM_DUTY_MAX);

    // 初始化刹车引脚(低电平有效)
    gpio_init(WHEEL_1_SC_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(WHEEL_2_SC_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // 初始化测速编码器
    encoder_quad_init(WHEEL_1_ENCODER, WHEEL_1_ENCODER_A_PIN, WHEEL_1_ENCODER_B_PIN);
    encoder_quad_init(WHEEL_2_ENCODER, WHEEL_2_ENCODER_A_PIN, WHEEL_2_ENCODER_B_PIN);
    encoder_quad_init(WHEEL_3_ENCODER, WHEEL_3_ENCODER_A_PIN, WHEEL_3_ENCODER_B_PIN);

}

void __updateMotor(Motor *motor){
    #ifdef _DEBUG_BELL_
    if(WHEEL_PWM_MAX - absValue(motor->pwm) < 0){
        while(1){
            gpio_set_level(BELL_PIN, 1);
        }
    }
    #endif
    // 无论方向引脚高低电平，实际pwm = 10000 - pwm
    pwm_set_duty(motor->pwmChannel, PWM_DUTY_MAX - absValue(motor->pwm));
    gpio_set_level(motor->dirPin, (uint8)(motor->pwm >= 0));
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

void updateMotors(
        int16 motorLeftSpeed, int16 motorRightSpeed, int16 motorBottomSpeed, 
        int32 cameraSpeedTarget, int32 cameraTurnTarget, 
        int32 rollX, int32 pitchY, int32 yawZ,
        int32 angVelX, int32 angVelY, int32 angVelZ){

    // // 速度环更新
    // velPIDl.target = 0; velPIDl.measurement = motorLeftSpeed; __updatePID(&velPIDl);
    // velPIDr.target = 0; velPIDr.measurement = motorRightSpeed; __updatePID(&velPIDr);
    // velPIDy.target = cameraSpeedTarget; velPIDy.measurement = motorBottomSpeed; __updatePID(&velPIDy);
    
    // // 角度环更新
    // angPIDx.target = velPIDl.deltaOutput + velPIDr.deltaOutput; angPIDx.measurement = pitchX; __updatePID(&angPIDx); // TODO:左右两轮的deltaOutput是相加还是相减?
    // angPIDy.target = velPIDy.deltaOutput; angPIDy.measurement = rollY; __updatePID(&angPIDy);
    // angPIDz.target = cameraTurnTarget;    angPIDz.measurement = yawZ; __updatePID(&angPIDz);

    // 角速度环更新
    // angVelPIDx.target = angPIDx.deltaOutput; angVelPIDx.measurement = angVelX; __updatePID(&angVelPIDx);   
    // angVelPIDy.target = angPIDy.deltaOutput; angVelPIDy.measurement = angVelY; __updatePID(&angVelPIDy);   
    // angVelPIDz.target = angPIDz.deltaOutput; angVelPIDz.measurement = angVelZ; __updatePID(&angVelPIDz);   

    /* 角速度环验证
        当roll↑时,Δroll>0. 应该让roll↓,而左轮pwm↑时,角动量=(+,0,+),原测速↓,经过人为纠正↑,导致roll↑ ∴pwmL += -ΔrollX
        当roll↑时,Δroll>0. 应该让roll↓,而右轮pwm↑时,角动量=(-,0,+),原测速↓,经过人为纠正↑,导致roll↓ ∴pwmR += +ΔrollX
        当yaw↑时,Δyaw>0. 应该让yaw↓,而左轮pwm↑时,角动量=(+,0,+),原测速↓,经过人为纠正↑,导致yaw↓ ∴pwmL += ΔyawY
        当yaw↑时,Δyaw>0. 应该让yaw↓,而右轮pwm↑时,角动量=(+,0,+),原测速↓,经过人为纠正↑,导致yaw↓ ∴pwmR += ΔyawY
    */
    angVelPIDx.target = 0; angVelPIDx.measurement = angVelX; __updatePID(&angVelPIDx);   
    angVelPIDy.target = 0; angVelPIDy.measurement = angVelY; __updatePID(&angVelPIDy);   
    angVelPIDz.target = 0; angVelPIDz.measurement = angVelZ; __updatePID(&angVelPIDz);   

    // PWM更新
    // setMotor(&motorLeft, ASSIGN, angVelPIDx.deltaOutput + angVelPIDz.deltaOutput); // TODO:左右两轮的deltaOutput是相加还是相减?
    // setMotor(&motorRight, ASSIGN, angVelPIDx.deltaOutput + angVelPIDz.deltaOutput); // TODO:左右两轮的deltaOutput是相加还是相减?
    setMotor(&motorLeft, ASSIGN, -angVelPIDx.deltaOutput + angVelPIDy.deltaOutput);
    setMotor(&motorRight, ASSIGN, angVelPIDx.deltaOutput + angVelPIDy.deltaOutput);
    // setMotor(&motorBottom, ASSIGN, angVelPIDy.deltaOutput);
}

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

    // 初始化电机的DIR与PWM引脚   
    __initMotor(&motorLeft, 17000, 0, WHEEL_1_PWM_PIN, WHEEL_1_DIR_PIN, 10, 3, 1, 0, 300);
    __initMotor(&motorRight, 17000, 0, WHEEL_2_PWM_PIN, WHEEL_2_DIR_PIN, 10, 3, 1, 0, 300);
    __initMotor(&motorBottom, 17000, 0, WHEEL_3_PWM_PIN, WHEEL_3_DIR_PIN, 10, 3, 1, 0, 300);

    // 初始化PID
    __initPID(&velPIDl, 10, 5, 2, 0, 1000);
    __initPID(&velPIDr, 10, 5, 2, 0, 1000);
    __initPID(&velPIDy, 10, 5, 2, 0, 1000);
    __initPID(&angPIDx, 10, 5, 2, 0, 1000);
    __initPID(&angPIDy, 10, 5, 2, 0, 1000);
    __initPID(&angPIDz, 10, 5, 2, 0, 1000);
    __initPID(&angVelPIDx, 100, 5, 2, 0, 1000);
    __initPID(&angVelPIDy, 100, 5, 2, 0, 1000);
    __initPID(&angVelPIDz, 100, 5, 2, 0, 1000);


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
            pwmTemp = offset;
            pwmTemp = minValue(pwmTemp, WHEEL_PWM_MAX);
            pwmTemp = maxValue(pwmTemp, -WHEEL_PWM_MAX);
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

    // 在不考虑上一层PID环的情况下,我们期望车身直立平衡,angPIDx与angPIDy的target均为0,angPIDz的target随意.
    angPIDx.target = 0; angPIDx.measurement = rollX; __updatePID(&angPIDx);
    angPIDy.target = 0; angPIDy.measurement = pitchY; __updatePID(&angPIDy);
    angPIDz.target = (int)(0);    angPIDz.measurement = yawZ; __updatePID(&angPIDz);

    /* 通过角度环输出, 决定角速度环target
        已知:
            当车身有角动量(+,0,0)时,roll↓
            当车身有角动量(0,+,0)时,pitch↓
            当车身有角动量(0,0,+)时,yaw↑
            当车身有角动量(+,0,0)时,gyroY->angVelX为负.反过来说gyroY->angVelX为正时,说明车身受到了一个等价的(-,0,0)角动量
            当车身有角动量(0,+,0)时,gyroZ->angVelY为负.反过来说gyroZ->angVelY为正时,说明车身受到了一个等价的(0,-,0)角动量
            当车身有角动量(0,0,+)时,gyroX->angVelZ为正.反过来说gyroX->angVelZ为正时,说明车身受到了一个等价的(0,0,+)角动量
        分析:
            X: 当车身roll<0时,measurement<0. 我们期望车身角动量=(-,0,0). 此时target - measurement更可能为正, 即angPIDx输出更可能为正.
                我们期望accVelPIDx的target↓,使得deltaoutput=target-measurement减小,等价于measurement↑,即angVelX↑,效果等价于给车身一个角动量(-,0,0)
                ∴angVelPIDx.target -= angPIDx
            Y: 当车身pitch<0时,measurement<0. 我们期望车身角动量=(0,-,0). 此时target - measurement更可能为正, 即angPIDy输出更可能为正.
                我们期望accVelPIDy的target↓,使得deltaoutput=target-measurement减小,等价于measurement↑,即angVelY↑,效果等价于给车身一个角动量(0,-,0)
                ∴angVelPIDy.target -= angPIDy
            Z: 当车身yaw<target时,measurement<target. 我们期望车身角动量=(0,0,+). 此时target - measurement > 0, 即angPIDz输出为正.
                我们期望accVelPIDz的target↓,使得deltaoutput=target-measurement减小,等价于measurement↑,即angVelZ↑,效果等价于给车身一个角动量(0,0,+)
                ∴angVelPIDz.target -= angPIDy

            当roll>0时,左轮角动量=(+,0,+),反作用角动量=(-,0,-)。
    */

    // 在不考虑上一层PID环的情况下,我们期望车身不动,因此angVelPID的target均为0.
    // angVelPIDx.target = -angPIDx.deltaOutput; angVelPIDx.measurement = angVelX; __updatePID(&angVelPIDx);   
    // angVelPIDy.target = -angPIDy.deltaOutput; angVelPIDy.measurement = angVelY; __updatePID(&angVelPIDy);   
    angVelPIDx.target = 0; angVelPIDx.measurement = angVelX; __updatePID(&angVelPIDx);   
    angVelPIDy.target = 0; angVelPIDy.measurement = angVelY; __updatePID(&angVelPIDy); 
    angVelPIDz.target = 0; angVelPIDz.measurement = angVelZ; __updatePID(&angVelPIDz);   

    /* 通过角速度环输出,决定PWM
        已知:
            当车身有角动量(0,+,0)时,gyroZ->angVelY为负
            当车身有角动量(+,0,0)时,gyroY->angVelX为负
            当车身有角动量(0,0,+)时,gyroX->angVelZ为正
        分析:
            X: 当roll↑时,角动量为(-,0,0),gyroY->angVelX为正. 此时measurement↑,angVelPIDx输出值target-measurement减小
                我们期望左轮pwm↓,角动量=(-,0,-),反作用角动量=(+,0,+),可以抵消X方向角动量. 
                ∴pwmL += angVelPIDx
            X: 当roll↑时,角动量为(-,0,0),gyroY->angVelX为正. 此时measurement↑,angVelPIDx输出值target-measurement减小
                我们期望右轮pwm↑,角动量=(-,0,+),反作用角动量=(+,0,-),可以抵消X方向角动量. 
                ∴pwmR -= angVelPIDx
            Y: 当pitch↑时,角动量为(0,-,0),gyroZ->angVelY为正. 此时measurement↑,angvelPIDy输出值target-measurement减小
                ......
                ......
            Z: 当yaw↑时,角动量为(0,0,+),gyroX->angVelZ为正. 此时measurement↑,angVelPIDz输出值target-measurement减小
                我们期望左轮pwm↑,角动量=(+,0,+),反作用角动量=(-,0,-),可以抵消Z方向角动量.
                ∴pwmL -= angVelPIDz
            Z: 当yaw↑时,角动量为(0,0,+),gyroX->angVelZ为正. 此时measurement↑,angVelPIDz输出值target-measurement减小
                我们期望右轮pwm↑,角动量=(-,0,+),反作用角动量=(+,0,-),可以抵消Z方向角动量.
                ∴pwmR -= angVelPIDz
        实践测试:
            当车身角动量为(+,0,0)时,右轮角动量=(+,0,-),反作用角动量为(-,0,+),阻止了X方向的角动量变化.
    */
    setMotor(&motorLeft, ASSIGN, angVelPIDx.deltaOutput - angVelPIDz.deltaOutput);
    setMotor(&motorRight, ASSIGN, -angVelPIDx.deltaOutput - angVelPIDz.deltaOutput);
    // setMotor(&motorBottom, ASSIGN, angVelPIDy.deltaOutput);

}

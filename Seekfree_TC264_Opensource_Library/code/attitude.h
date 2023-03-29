/*
 * attitude_solution.h
 *
 *  Created on: Jan 6, 2023
 *      Author: admin
 */

#ifndef ATTITUDE_SOLUTION_H_
#define ATTITUDE_SOLUTION_H_

#include "zf_common_headfile.h"

#define DEG_TO_RAD 0.0174f
#define RAD_TO_DEG 57.3f
#define Acc_Gain    0.0002441f        //加速度变成G (初始化加速度满量程-+8g LSBa = 2*8/65535.0)
#define Gyro_Gain   0.0609756f        //角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)

void Init_MPU6050_GYRO();
void attitude_solution_func(int16_t accx, int16_t accy, int16_t accz, int16_t gyrox, int16_t gyroy, int16_t gyroz, float *yaw, float *rol, float *pitch);
void Angle_Calcu(int16_t accx, int16_t accy, int16_t accz, int16_t gyrox, int16_t gyroy, int16_t gyroz, float *pitch, float *rol, float *yaw);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);

extern float Angle_X_Final;        //X最终倾斜角度
extern float Angle_Y_Final;        //Y最终倾斜角度

typedef struct{
    float x;
    float y;
    float z;
}MPU6050_DATAINIT_GYRO;
extern MPU6050_DATAINIT_GYRO MPU6050_datainit_gyro;
#endif /* ATTITUDE_SOLUTION_H_ */

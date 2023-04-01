/*
 * attitude_solution.c
 *
 *  Created on: Jan 6, 2023
 *      Author: admin
 */
#include <math.h>
#include "attitude.h"

double q0=1,q1=0,q2=0,q3=0;
/**********************************************************************************************
Function: invSqrt
Description: 快速平方根倒数，针对float 32位浮点数
Input: 单精度（32位）浮点数number
Output: 无
Input_Output: 无
Return: 快速平方根倒数的值
***********************************************************************************************/
float invSqrt(float x){
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);//0x5f3759df，0x5f37642f，0x5f375a86 三个可选初值
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));//牛顿迭代法，迭代两次精度更高
    return y;
}
/**********************************************************************************************
Function: fast_invSqrt64
Description: 快速平方根倒数
Input: 双精度浮点数number
Output: 无
Input_Output: 无
Return: 快速平方根倒数的值
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
double fast_invSqrt64(double number)
{
    long long i;
    double x2, y;
    const double threehalfs = 1.5;

    x2 = number * 0.5;
    y = number;
    i = *(long long*)& y;
    i = 0x5fe6eb50c7b537a9 - (i >> 1);
    y = *(double*) &i;
    y = y * (threehalfs - (x2 * y * y));
    y = y * (threehalfs - (x2 * y * y));

    return y;
}

/******************************************************************************************************
 * 一阶低通滤波器
 * 函数名：FirstOrderLowPassFilter
 * 输入：
 * 待滤波的数据，当前值和上一个值
 * 滤波系数
 * 输出：
 * 滤波输出
 ******************************************************************************************************/
float FirstOrderLowPassFilter(float data, float data_last, float alpha){
    return (alpha * data + (1 - alpha) * data_last);
}

MPU6050_DATAINIT_GYRO MPU6050_datainit_gyro = {
        0,0,0
};
void Init_MPU6050_GYRO(){
    icm20602_get_gyro();
    system_delay_ms(9);
    icm20602_get_gyro();
    system_delay_ms(9);
    icm20602_get_gyro();
    system_delay_ms(9);
    for (int i = 0; i < 100; ++i) {
        icm20602_get_gyro();
        MPU6050_datainit_gyro.x += (float)icm20602_gyro_x;
        MPU6050_datainit_gyro.y += (float)icm20602_gyro_y;
        MPU6050_datainit_gyro.z += (float)icm20602_gyro_z;
        system_delay_ms(9);
    }
    MPU6050_datainit_gyro.x = MPU6050_datainit_gyro.x / 100;
    MPU6050_datainit_gyro.y = MPU6050_datainit_gyro.y / 100;
    MPU6050_datainit_gyro.z = MPU6050_datainit_gyro.z / 100;
}

float accex = 0, accey = 0, accez = 0;
#define KI  0.008f
#define KP  5.0f
float kp = KP, ki = KI;
#define dt  0.01
#define half_dt  0.005 //dt的一半，不要写dt/2，
void attitude_solution_func(int16_t accx, int16_t accy, int16_t accz, int16_t gyrox, int16_t gyroy, int16_t gyroz, float *pitch, float *rol, float *yaw){
    static float accx_last,accy_last,accz_last;
    float ax = FirstOrderLowPassFilter((float)accx, accx_last, 0.7)
            , ay = FirstOrderLowPassFilter((float)accy, accy_last, 0.7)
            , az = FirstOrderLowPassFilter((float)accz, accz_last, 0.7);
    accx_last = ax; accy_last = ay; accz_last = az;
    float gx = (float)gyrox, gy = (float)gyroy, gz = (float)gyroz;
    //角度转弧度
    gx = (gx - MPU6050_datainit_gyro.x) * DEG_TO_RAD * Gyro_Gain;
    gy = (gy - MPU6050_datainit_gyro.y) * DEG_TO_RAD * Gyro_Gain;
    gz = (gz - MPU6050_datainit_gyro.z) * DEG_TO_RAD * Gyro_Gain;
    /*加速度归一化（不需要转化成m/s？）
    *加速度过大会导致估算角度有误，直接不考虑
    *不考虑的话，角速度会有零飘（bias），怎么办？
    *去掉bias呗，那剩下的就是热噪声了，再用卡尔曼滤波！
    *那中间呢？静止大约1.1*9.8，中间部分呢？
    */
    if((ax * ax + ay * ay + az * az) > 33527921.f){
        ki = 0;//KI/10;
        kp = 0;//KP*10;
    }else {
        ki = KI;
        kp = KP;
    }
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * recipNorm;
    ay = ay * recipNorm;
    az = az * recipNorm;
    //提取物体坐标系下的重力分量
    float Vx,Vy,Vz;
    Vx = 2.0f * (q1*q3 - q0*q2);
    Vy = 2.0f * (q0*q1 + q2*q3);
    Vz = 1.0f - 2.0f * (q1*q1 + q2*q2);//??q0q0 - q1q1 - q2q2 + q3q3 ;由于四元数是归一化的，前种计算可以减少计算量
    //计算误差，姿态误差，加速度和估算出的重力分量的误差
    float ex,ey,ez;
    ex = (ay * Vz - az * Vy);
    ey = (az * Vx - ax * Vz);
    ez = (ax * Vy - ay * Vx);
    /**
     * 为什么不是
     * ex = Vx - ax
     * ey = Vy - ay
    */

    //对误差积分
    accex = accex + ex * ki;
    accey = accey + ey * ki;
    accez = accez + ez * ki;
    //互补滤波,测得的角速度
    gx = gx + kp * ex + accex;
    gy = gy + kp * ey + accey;
    gz = gz + kp * ez + accez;
    //解四元数微分方程
    float q0l = q0,q1l = q1,q2l = q2;//记录
    q0 = q0 + half_dt * (-q1 * gx - q2 * gy -q3 * gz);
    q1 = q1 + half_dt * (q0l * gx + q2 * gz - q3 * gy);
    q2 = q2 + half_dt * (q0l * gy - q1l * gz + q3 * gx);
    q3 = q3 + half_dt * (q0l * gz + q1l * gy - q2l * gx);
    //四元数归一化
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * recipNorm;
    q1 = q1 * recipNorm;
    q2 = q2 * recipNorm;
    q3 = q3 * recipNorm;
    //计算姿态角
    float g1,g2,g3,g4,g5;
    g1 = 2.0f * (q1*q3 - q0*q2);
    g2 = 2.0f * (q0*q1 + q2*q3);
    g3 = 1.0f - 2.0f * (q1*q1 + q2*q2);//??q0q0 - q1q1 - q2q2 + q3q3 ;由于四元数是归一化的，前种计算可以减少计算量
    g4 = 2.0f * (q1*q2 + q0*q3);
    g5 = 1.0f - 2.0f * (q2*q2 + q3*q3);
    *rol = atan2f(g2, g3) * RAD_TO_DEG;
    *pitch = -asinf(g1) * RAD_TO_DEG;
    *yaw = atan2f(g4, g5) * RAD_TO_DEG;
    *yaw +=  (gyroz - MPU6050_datainit_gyro.z) * Gyro_Gain * dt;
}


//卡尔曼解算法库
/***********
 * 和我上面的四元数算出来的角度正交，很奇怪，需要转一下套进去。
 * 直接拿加速度算角度太纯了
 * 还是用四元数算角度，然后用卡尔曼滤波，感觉会好一点。
 * 但是两个置信度和噪声的值还是蛮有说法的。
 * ht 2023.1.16
 * 卡尔曼滤波器先是认为我们测得（也就是计算出来的角度）不靠谱，当然，确实不靠谱。
 * 然后我们就有置信度的说法，其实就是协方差（代表的这个数据里面有多大的噪声）
 * 实际上我们是不知道这个噪声有多大的（因为我们的陀螺仪乱动，如果不乱动的话就可以查数据手册），幸好卡尔曼的容忍度蛮大的。
 * 所以这个噪声有一部分其实是乱动导致的，也就是陀螺仪角度发生改变，实际上就是我们给卡尔曼滤波器输入的角度，
 * 好像是噪声越大我们越不信任他，但是我们会认为这没办法，反而会更加跟随这个值，很奇怪？
 *
 */
float Angle_x_temp;         //由加速度计算的x倾斜角度
float Angle_y_temp;         //由加速度计算的y倾斜角度
float Angle_X_Final;        //X最终倾斜角度
float Angle_Y_Final;        //Y最终倾斜角度

//读取数据预处理
/*
 *
 */
void Angle_Calcu(int16_t accx, int16_t accy, int16_t accz, int16_t gyrox, int16_t gyroy, int16_t gyroz, float *pitch, float *rol, float *yaw)
{
//    accx = accx * Acc_Gain;
//    accy = accy * Acc_Gain;
//    accz = accz * Acc_Gain;
    //加速度反正切公式计算三个轴和水平面坐标系之间的夹角
    *pitch=(atanf(accy/accz))*180.f/3.14;
    *rol=(atanf(accx/accz))*180.f/3.14;

    //3.角速度原始值处理过程
    //陀螺仪配置寄存器0X1B内写入0x18，设置范围为±2000deg/s。换算关系：2^16/4000=16.4LSB/(°/S)
    ////计算角速度
    gyrox = (gyrox - MPU6050_datainit_gyro.x) * Gyro_Gain;
    gyroy = (gyroy - MPU6050_datainit_gyro.y) * Gyro_Gain;

    //4.调用卡尔曼函数
    Kalman_Filter_X(*pitch,gyroy);  //卡尔曼滤波计算X倾角
    Kalman_Filter_Y(*rol,gyrox);  //卡尔曼滤波计算Y倾角
}


//卡尔曼参数
float Q_angle = 0.001;      //角度数据置信度，角度噪声的协方差
float Q_gyro  = 0.003;      //角速度数据置信度，角速度噪声的协方差
float R_angle = 0.5;        //加速度计测量噪声的协方差
char  C_0     = 1;          //H矩阵值
float Q_biasx, Angle_errx;    //Q_bias:陀螺仪的偏差  Angle_err:角度偏量
float Q_biasy, Angle_erry;    //Q_bias:陀螺仪的偏差  Angle_err:角度偏量
float PCt_0, PCt_1, E;      //计算的过程量
float K_0, K_1, t_0, t_1;   //卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
float P[4] ={0,0,0,0};  //过程协方差矩阵的微分矩阵，中间变量
float PPx[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P
float PPy[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P

void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数
{
    //步骤一，先验估计
    //公式：X(k|k-1) = AX(k-1|k-1) + BU(k)
    //X = (Angle,Q_bias)
    //A(1,1) = 1,A(1,2) = -dt
    //A(2,1) = 0,A(2,2) = 1
    Angle_X_Final = Angle_X_Final+(Gyro - Q_biasx) * dt; //状态方程,角度值等于上次最优角度加角速度减零漂后积分

    //步骤二，计算过程协方差矩阵的微分矩阵
    //公式：P(k|k-1)=AP(k-1|k-1)A^T + Q
    //Q(1,1) = cov(Angle,Angle) Q(1,2) = cov(Q_bias,Angle)
    //Q(2,1) = cov(Angle,Q_bias)    Q(2,2) = cov(Q_bias,Q_bias)
    P[0]= Q_angle - PPx[0][1] - PPx[1][0];
    P[1]= -PPx[1][1];// 先验估计误差协方差
    P[2]= -PPx[1][1];
    P[3]= Q_gyro;
    PPx[0][0] += P[0] * dt;
    PPx[0][1] += P[1] * dt;
    PPx[1][0] += P[2] * dt;
    PPx[1][1] += P[3] * dt;

    //步骤三，计算卡尔曼增益
    //公式：Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
    //Kg = (K_0,K_1) 对应Angle,Q_bias增益
    //H = (1,0) 可由z=HX+v求出z:Accel
    PCt_0 = C_0 * PPx[0][0];
    PCt_1 = C_0 * PPx[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    //步骤四，后验估计误差协方差
    //公式：P(k|k)=(I-Kg(k)H)P(k|k-1)
    //也可写为：P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
    t_0 = PCt_0;
    t_1 = C_0 * PPx[0][1];
    PPx[0][0] -= K_0 * t_0;
    PPx[0][1] -= K_0 * t_1;
    PPx[1][0] -= K_1 * t_0;
    PPx[1][1] -= K_1 * t_1;

    //步骤五，计算最优角速度值
    //公式：X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
    Angle_errx = Accel - Angle_X_Final;  //Z(k)先验估计 计算角度偏差
    Angle_X_Final += K_0 * Angle_errx;    //后验估计，给出最优估计值
    Q_biasx        += K_1 * Angle_errx;    //后验估计，跟新最优估计值偏差
}

void Kalman_Filter_Y(float Accel,float Gyro)
{
    Angle_Y_Final += (Gyro - Q_biasy) * dt;
    P[0]=Q_angle - PPy[0][1] - PPy[1][0];
    P[1]=-PPy[1][1];
    P[2]=-PPy[1][1];
    P[3]=Q_gyro;
    PPy[0][0] += P[0] * dt;
    PPy[0][1] += P[1] * dt;
    PPy[1][0] += P[2] * dt;
    PPy[1][1] += P[3] * dt;
    Angle_erry = Accel - Angle_Y_Final;
    PCt_0 = C_0 * PPy[0][0];
    PCt_1 = C_0 * PPy[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * PPy[0][1];
    PPy[0][0] -= K_0 * t_0;
    PPy[0][1] -= K_0 * t_1;
    PPy[1][0] -= K_1 * t_0;
    PPy[1][1] -= K_1 * t_1;
    Angle_Y_Final   += K_0 * Angle_erry;
    Q_biasy  += K_1 * Angle_erry;
}


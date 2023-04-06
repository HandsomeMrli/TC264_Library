#ifndef _DEFINE_H_
#define _DEFINE_H_


// #define GYRO_ICM

// #if defined(GYRO_ICM20602) 
//     #define gyro_x icm20602_gyro_x
//     #define gyro_y icm20602_gyro_y
//     #define gyro_z icm20602_gyro_z
// #endif

#define minValue(a,b) (((a)<(b))?(a):(b))
#define maxValue(a,b) (((a)>(b))?(a):(b))
#define absValue(a) (((a)>=0)?(a):(-(a)))
#define signValue(a) (((a)>0) ? (1) : (((a)<0)?(-1):(0)))
#define squareValue(a) ((a)*(a))
#define swapValue(a,b) (a)^=(b);(b)^=(a);(a)^=(b);
#define diffSumRatio(a,b,k) ((k * absValue((a)-(b))) / ((a)+(b)))
#define isInRange(a,min,max) ((a)>=(min)&&(a)<=(max))

#define CAMERA_WIDTH 160
#define CAMERA_HEIGHT 128

#ifndef _STDINT_H
    typedef signed char int8_t;
    typedef unsigned char   uint8_t;
    typedef short  int16_t;
    typedef unsigned short  uint16_t;
    typedef int  int32_t;
    typedef unsigned   uint32_t;
#endif

// 拨码开关
#define SW_4_PIN P11_3
#define SW_3_PIN P13_3
#define SW_2_PIN P13_2
#define SW_1_PIN P13_1

// 按键
#define BTN_1_PIN P02_6
#define BTN_2_PIN P11_12

// 串口
#define UART_CHANNEL UART_2
#define UART_TXD_PIN UART2_TX_P10_5
#define UART_RXD_PIN UART2_RX_P10_6
#define UART_RTS_PIN P10_2

// LED
#define LED_1_PIN P00_8

// 蜂鸣器
#define BELL_PIN P20_0

// 电机(编码器/PWM)
#define WHEEL_1_SC_PIN P22_2
#define WHEEL_1_PWM_PIN ATOM0_CH2_P21_4
#define WHEEL_1_DIR_PIN P22_1
#define WHEEL_1_ENCODER TIM2_ENCODER
#define WHEEL_1_ENCODER_A_PIN TIM2_ENCODER_CH1_P33_7
#define WHEEL_1_ENCODER_B_PIN TIM2_ENCODER_CH2_P33_6
#define WHEEL_2_SC_PIN P11_3
#define WHEEL_2_PWM_PIN ATOM0_CH4_P14_1
#define WHEEL_2_DIR_PIN P15_8
#define WHEEL_2_ENCODER TIM5_ENCODER
#define WHEEL_2_ENCODER_A_PIN TIM5_ENCODER_CH1_P10_3
#define WHEEL_2_ENCODER_B_PIN TIM5_ENCODER_CH2_P10_1

#define WHEEL_3_DIR_PIN P21_2
#define WHEEL_3_PWM_PIN ATOM0_CH1_P21_3
#define WHEEL_3_ENCODER TIM4_ENCODER
#define WHEEL_3_ENCODER_A_PIN TIM4_ENCODER_CH1_P02_8
#define WHEEL_3_ENCODER_B_PIN TIM4_ENCODER_CH2_P00_9

#define WHEEL_PWM_MAX 9000

#endif
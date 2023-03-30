#ifndef _DEFINE_H_
#define _DEFINE_H_

#include "zf_common_headfile.h"

// #define GYRO_ICM

#define absValue(a) (((a)>=0)?(a):(-(a)))
#define signValue(a) (((a)>0)?(1):(((a)==0)?(0):(-1)))

// #if defined(GYRO_ICM20602) 
//     #define gyro_x icm20602_gyro_x
//     #define gyro_y icm20602_gyro_y
//     #define gyro_z icm20602_gyro_z
// #endif


#endif
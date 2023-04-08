#include "print.h"

/**
 * 10ms中断对无线串口的收发影响测试结果
 * × 123
 * √ 3 12 13 23
*/
void printEularAngle(const FusionEuler *euler){
    tft180_show_string(0, 0, "accX");  
    tft180_show_string(0, 16, "accY"); 
    tft180_show_string(0, 32, "accZ"); 
    tft180_show_string(0, 48, "gyroX");
    tft180_show_string(0, 64, "gyroY");
    tft180_show_string(0, 80, "gyroZ");

    tft180_show_int(44, 0, icm20602_acc_x, 6);  
    tft180_show_int(44, 16, icm20602_acc_y, 6); 
    tft180_show_int(44, 32, icm20602_acc_z, 6); 
    tft180_show_int(44, 48, icm20602_gyro_x, 6);
    tft180_show_int(44, 64, icm20602_gyro_y, 6);
    tft180_show_int(44, 80, icm20602_gyro_z, 6);

    tft180_show_float(78, 0, icm20602_acc_transition(icm20602_acc_x), 2, 2);
    tft180_show_float(78, 16, icm20602_acc_transition(icm20602_acc_y), 2, 2);
    tft180_show_float(78, 32, icm20602_acc_transition(icm20602_acc_z), 2, 2);
    tft180_show_float(78, 48, icm20602_gyro_transition(icm20602_gyro_x), 2, 2);
    tft180_show_float(78, 64, icm20602_gyro_transition(icm20602_gyro_y), 2, 2);
    tft180_show_float(78, 80, icm20602_gyro_transition(icm20602_gyro_z), 2, 2);

    tft180_show_string(0, 96, "yaw");
    tft180_show_string(0, 112, "rol");
    tft180_show_string(0, 128, "pitch");

    tft180_show_float(43, 96, euler->angle.yaw, 3, 1);
    tft180_show_float(43, 112, euler->angle.roll, 3, 1);
    tft180_show_float(43, 128, euler->angle.pitch, 3, 1);

    // tft180_show_int(42, 96, euler->angle.yaw, 3);
    // tft180_show_int(42, 112, euler->angle.roll, 3);
    // tft180_show_int(42, 128, euler->angle.pitch, 3);

}

void printMotorSpeed(){
    tft180_show_string(0, 0, "motorL");  
    tft180_show_string(0, 16, "motorR"); 
    tft180_show_string(0, 32, "motorB"); 
    // tft180_show_string(0, 48, "gyroX");
    // tft180_show_string(0, 64, "gyroY");
    // tft180_show_string(0, 80, "gyroZ");

    tft180_show_int(50, 0, motorLeftSpeed, 6);  
    tft180_show_int(50, 16, motorRightSpeed, 6); 
    tft180_show_int(50, 32, motorBottomSpeed, 6);
    // tft180_show_int(44, 32, icm20602_acc_z, 6); 
    // tft180_show_int(44, 48, icm20602_gyro_x, 6);
    // tft180_show_int(44, 64, icm20602_gyro_y, 6);
    // tft180_show_int(44, 80, icm20602_gyro_z, 6);
}
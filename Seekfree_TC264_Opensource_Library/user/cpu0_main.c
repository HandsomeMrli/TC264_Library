/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。

// 本例程是开源库移植用空工程
// 本例程是开源库移植用空工程
// 本例程是开源库移植用空工程

#include "attitude.h"

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    tft180_init();
    icm20602_init();
    Init_MPU6050_GYRO();

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码

        icm20602_get_acc();
        icm20602_get_gyro();

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

        float yaw, rol, pitch;
        
        attitude_solution_func(icm20602_acc_x, icm20602_acc_y, icm20602_acc_z, icm20602_gyro_x, icm20602_gyro_y, icm20602_gyro_z, &yaw, &rol, &pitch);
        
        tft180_show_string(0, 96, "yawZ");
        tft180_show_string(0, 112, "rolY");
        tft180_show_string(0, 128, "pitchX");
        tft180_show_float(50, 96, yaw, 3, 3);
        tft180_show_float(50, 112, rol, 3, 3);
        tft180_show_float(50, 128, pitch, 3, 3);
        
        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************


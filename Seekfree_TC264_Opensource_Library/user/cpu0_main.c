/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cpu0_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.8.0
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "define.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��


// ���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
// ����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
// Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
// һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��

// �������ǿ�Դ����ֲ�ÿչ���
// �������ǿ�Դ����ֲ�ÿչ���
// �������ǿ�Դ����ֲ�ÿչ���

// **************************** �������� ****************************

uint8 uart_get_data[64];
uint8 fifo_get_data[64];
uint8 get_data = 0;
uint32 fifo_data_count = 0;
fifo_struct uart_data_file;

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������

    gpio_init(LED_1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    gpio_init(BTN_1_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(BTN_2_PIN, GPI, GPIO_LOW, GPI_PULL_UP);

    gpio_init(SW_1_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(SW_2_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(SW_3_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(SW_4_PIN, GPI, GPIO_LOW, GPI_PULL_UP);


    fifo_init(&uart_data_file, FIFO_DATA_8BIT, uart_get_data, 64);
    uart_init(UART_CHANNEL, 9600, UART2_TX_P10_5, UART2_RX_P10_6);
    uart_rx_interrupt(UART_CHANNEL, 1);
    uart_write_string(UART_CHANNEL, "UART init successful!");
    uart_write_byte(UART_CHANNEL, '\r');
    uart_write_byte(UART_CHANNEL, '\n');

    pwm_init(WHEEL_1_PWM_PIN, 17000, 2000);
    pwm_init(WHEEL_2_PWM_PIN, 5000, 5000);

    encoder_quad_init(WHEEL_1_ENCODER, WHEEL_1_ENCODER_A_PIN, WHEEL_1_ENCODER_B_PIN);
    encoder_quad_init(WHEEL_2_ENCODER, WHEEL_2_ENCODER_A_PIN, WHEEL_2_ENCODER_B_PIN);


    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���




        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
// **************************** �������� ****************************


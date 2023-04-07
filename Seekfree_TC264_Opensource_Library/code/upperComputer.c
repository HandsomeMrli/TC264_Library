#include "upperComputer.h"

void wireless_uart_LingLi_send(
        uint16 a1, uint16 a2, uint16 a3, uint16 a4,
        uint16 b1, uint16 b2, uint16 b3, uint16 b4,
        uint16 c1, uint16 c2, uint16 c3, uint16 c4
        ) {

    int i, j;
    static unsigned short int send_data[3][4] = { { 0 }, { 0 }, { 0 } };
    short int checksum = 0;
    unsigned char xorsum = 0, high, low;

    send_data[0][0] = (unsigned short int)(a1);
    send_data[0][1] = (unsigned short int)(a2);
    send_data[0][2] = (unsigned short int)(a3);
    send_data[0][3] = (unsigned short int)(a4);

    send_data[1][0] = (unsigned short int)(b1);
    send_data[1][1] = (unsigned short int)(b2);
    send_data[1][2] = (unsigned short int)(b3);
    send_data[1][3] = (unsigned short int)(b4);

    send_data[2][0] = (unsigned short int)(c1);
    send_data[2][1] = (unsigned short int)(c2);
    send_data[2][2] = (unsigned short int)(c3);
    send_data[2][3] = (unsigned short int)(c4);

    wireless_uart_send_byte('S');
    wireless_uart_send_byte('T');
    for (i = 0; i < 3; i++)
        for (j = 0; j < 4; j++)
        {
            low = (unsigned char)(send_data[i][j] & 0x00ff);
            high = (unsigned char)(send_data[i][j] >> 8u);
            wireless_uart_send_byte(low); wireless_uart_send_byte(high);
            checksum += low; checksum += high;
            xorsum ^= low; xorsum ^= high;
        }
    wireless_uart_send_byte((unsigned char)(checksum & 0x00ff));
    wireless_uart_send_byte(xorsum);
}
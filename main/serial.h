#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "sx1280_rx.h"

typedef union
{
    struct
    {
        // 160 bits of data (10 bits per channel * 16 channels) = 20 bytes.
        unsigned int chan1 : 10;
        unsigned int chan2 : 10;
        unsigned int chan3 : 10;
        unsigned int chan4 : 10;
        unsigned int chan5 : 10;
        unsigned int chan6 : 10;
        unsigned int chan7 : 10;
        unsigned int chan8 : 10;
        unsigned int chan9 : 10;
        unsigned int chan10 : 10;
        unsigned int chan11 : 10;
        unsigned int chan12 : 10;
        unsigned int chan13 : 10;
        unsigned int chan14 : 10;
        unsigned int chan15 : 10;
        unsigned int chan16 : 10;
    } __attribute__((packed));
    uint8_t crsfpayload[20];
} crsfPayloadRc_t;

void UART0_Init();
#endif
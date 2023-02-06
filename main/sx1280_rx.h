/*    Protocol description:


 2.4Ghz LORA modulation

  Operation modes:

# RATE_LORA_100HZ
- LORA modulation
- Frame rate 100 HZ (10ms)
- Data Rate ~44kb/s (-112dBm)
- Bw-812 ; SF7 ; CR - LI - 4/7
- Preamble 12 symbols
- Fixed length packet format(implicit)->12 bytes

# RATE_LORA_50HZ
- LORA modulation
- Frame rate 50 HZ (20ms)
- Data Rate ~25kb/s (-115dBm)
- Bw-812 ; SF8 ; CR - LI - 4/7
- Preamble 12 symbols
- Fixed length packet format(implicit)->16 bytes

telemetry rate (1:8)

#  bind frame (channel 96), LCG would not using 96 channel
    ----------------------------------------------------
    0.
       - bit 7     = reserved
       - bit 6     = channel 16 (arm) failsafe
       - bits 5..4 =  Aux channel (2 bit channel13) failsafe
       - bits 3..2 =  Aux channel (2 bit channel14) failsafe
       - bits 1..0 =  Aux channel (2 bit channel15) failsafe
    1. channels   10bit/channel channel 1-8 failsafe
    2. channels
    3. channels
    4. channels
    5. channels
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11.  fail safe for 9-12 (current only support Max/Mid/Min)      
       - bits 7..6 =  Aux channel  (2 bit channel9) 
       - bits 5..4 =  Aux channel  (2 bit channel10)
       - bits 3..2 =  Aux channel  (2 bit channel11)
       - bits 1..0 =  Aux channel  (2 bit channel12)
    12.
       - bits 7..6 = failsafe mode 00: Hold, 01: Custom, 10: No pulses, 11: Receiver
       - bit  5..0 = Rx_Num (6 bits) /FHSS Hope Step = Rx_Num+1.
    13. sync word (uint16)
    14. sync word
    15. 
       - bit 7     = 0:sbus output, 1: crsf output.
       - bit 6     = 0:sbus normal, 1: sbus inverted.
       - bit 5..0  = reserved


   #  normal frame 50Hz
       ----------------------------------------------------
    0.
       - bit 7     = 0 next frame will be normal, 1: next frame will be telemetry
       - bit 6     = channel 16 (arm)
       - bits 5..4 =  Aux channel (2 bit channel13)
       - bits 3..2 =  Aux channel (2 bit channel14)
       - bits 1..0 =  Aux channel (2 bit channel15)
    1. channels   10bit/channel channel 1-12 (12 full channel),
    2. channels
    3. channels
    4. channels
    5. channels
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11. channels
    12. channels
    13. channels
    14. channels
    15. channels

    #  normal frame 100Hz
   ----------------------------------------------------
    0.
    - bit 7     = 0 next frame will be normal, 1: next frame will be telemetry
    - bit 6     = channel 16 (arm)
    - bits 5..4 =  Aux channel (2 bit channel13)
    - bits 3..2 =  Aux channel (2 bit channel14)
    - bits 1..0 =  Aux channel (2 bit channel15)
    1. channels   10bit/channel channel 1-8 (8 full channel),
    2. channels
    3. channels
    4. channels
    5. channels
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11
       - bits 7..6 =  Aux channel  (2 bit channel9)
       - bits 5..4 =  Aux channel  (2 bit channel10)
       - bits 3..2 =  Aux channel  (2 bit channel11)
       - bits 1..0 =  Aux channel  (2 bit channel12)


*/

#ifndef __SX1280_RX_H__
#define __SX1280_RX_H__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <sys/time.h>
#include "time.h"
#include "esp_task_wdt.h"

typedef union
{
   struct
   {
      uint32_t send_buf_32[8];
      uint32_t recv_buf_32[8];
   };
   struct
   {
      uint16_t send_buf_16[16];
      uint16_t recv_buf_16[16];
   };
   struct
   {
      uint8_t send_buf_8[32];
      uint8_t recv_buf_8[32];
   };
   /* data */
} sx1280_buff_t;

typedef enum
{
   LRMODE = 0, // Longe range mode
   LLMODE = 1, // Low latency mode
} rc_mode_t;

typedef struct
{
   uint8_t telemetry : 1;
   uint8_t ch16 : 1; // ARM
   uint8_t ch13 : 2; // Aux
   uint8_t ch14 : 2; // Aux
   uint8_t ch15 : 2; // Aux
} frame_header_t;

typedef struct
{
   uint8_t ch9 : 2; //for 100Hz
   uint8_t ch10 : 2;
   uint8_t ch11 : 2;
   uint8_t ch12 : 2;
   uint8_t sync_h;
   uint8_t sync_l;
   uint8_t failsafe : 2;
   uint8_t rx_num : 6;
   uint8_t sbus : 1;
   uint8_t sbus_inv : 1;
   uint8_t reserve : 6;

} bind_info_t;

typedef struct
{
   frame_header_t frame_header;
   uint8_t ch1_8[10]; // channel 1-8
   union
   {
      bind_info_t bind_info;
      uint8_t ch9_12[5];
   } last5byte;
} frame_struct_t;

#endif //__SX1280_RX_H__
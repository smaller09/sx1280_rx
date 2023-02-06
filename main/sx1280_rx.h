/*    Protocol description:


 2.4Ghz LORA modulation

  Operation modes:
  
# RATE_LORA_100HZ
- LORA modulation
- Frame rate 100 HZ (10ms)
- Data Rate ~44kb/s (-112dBm)
- Bw-812 ; SF7 ; CR - LI - 4/7 
- Preamble 12 symbols
- Fixed length packet format(implicit)->14 bytes

# RATE_LORA_50HZ 
- LORA modulation
- Frame rate 50 HZ (20ms)
- Data Rate ~25kb/s (-115dBm)
- Bw-812 ; SF8 ; CR - LI - 4/7 
- Preamble 12 symbols
- Fixed length packet format(implicit)->18 bytes

telemetry rate (1:8)

   #  bind frame (channel 96), LCG would not using 96 channel
       ----------------------------------------------------
    0. - bits 7..6 = reserve (2 bits)
       - bits 5..4 = next expected telemetry down link frame counter(sequence) (2 bits=4 val)
       - bit 3     = Flag EU LBT
       - bits 2..0 = Frame type (3 bits)
    1. txid1 TXID on 16 bits
    2. txid2
    3. - bit 7     = flag next frame must be dwn tlm frame
       - bit 6     = flag requesing starting WIFI
       - bits 5..0 = Model ID /Rx_Num (6 bits) 
    4. channels 8 channels/frame ; 11bits/channel
    5. channels total 11 bytes of channels data in the packet frame
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11. channels
    12. channels
    13. channels
    14. channels
   



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
   LRMODE = 0x00, // Longe range mode
   LLMODE = 0xff, // Low latency mode
} rc_mode_t;

typedef union
{
   struct
   {
      uint8_t islinked;
      rc_mode_t rc_mode;
      uint16_t syncword;
   };
   uint32_t val;
}bind_status_t;



#endif //__SX1280_RX_H__
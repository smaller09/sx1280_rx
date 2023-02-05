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
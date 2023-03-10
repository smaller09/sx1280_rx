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
//#include "sx1280.h"
//#include "serial.h"

typedef union sx1280_buff_u
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

typedef union frameheader_u
{
   struct
   {
      uint8_t telemetry : 1; // frame_mode in bind frame
      uint8_t ch13 : 2;      // Aux
      uint8_t ch14 : 2;      // Aux
      uint8_t ch15 : 2;      // Aux
      uint8_t ch16 : 1;      // ARM
   };
   uint8_t val;
} frameheader_t;

typedef union ch9_12switch_u
{
   struct
   {
      uint8_t ch9 : 2;
      uint8_t ch10 : 2;
      uint8_t ch11 : 2;
      uint8_t ch12 : 2; /* data */
   };
   uint8_t val;
} ch9_12switch_t;

typedef struct bind_info_s
{
   uint8_t failsafe : 2; // failsafe mode
   uint8_t rx_num : 6;
   uint8_t sbus : 1;
   uint8_t sbus_inv : 1;
   uint8_t frame_mode : 2;
   uint8_t reserved : 4;
} bind_info_t;

typedef union frame_struct_u
{
   struct
   {
      frameheader_t frameheader;
      union
      {
         uint8_t ch1_8[10]; // channel 1-8
         struct
         {
            uint16_t chan01 : 10;
            uint16_t chan02 : 10;
            uint16_t chan03 : 10;
            uint16_t chan04 : 10;
            uint16_t chan05 : 10;
            uint16_t chan06 : 10;
            uint16_t chan07 : 10;
            uint16_t chan08 : 10;
         } __attribute__((packed));
      };
      union
      {
         struct
         {
            uint16_t chan09 : 10;
            uint16_t chan10 : 10;
            uint16_t chan11 : 10;
            uint16_t chan12 : 10;
         } __attribute__((packed));
         struct
         {
            ch9_12switch_t ch9_12switch;
            bind_info_t bind_info;
            uint16_t syncword;
         } __attribute__((packed));
         uint8_t ch9_12[5];
      };
   };
   uint8_t rcdata[16];
} frame_struct_t;

#define BIND_CHANNEL 96
#define FRAME_SIZE 16 // sizeof(bind_status)

#endif //__SX1280_RX_H__

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

typedef union
{
   struct
   {
      uint8_t telemetry : 1; // frame_mode in bind frame
      uint8_t ch16 : 1;      // ARM 
      uint8_t ch13 : 2;      // Aux
      uint8_t ch14 : 2;      // Aux
      uint8_t ch15 : 2;      // Aux
   };
   uint8_t val;
} frameheader_t;

typedef union
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

typedef union
{
   struct
   {
      uint8_t failsafe : 2; //failsafe mode
      uint8_t rx_num : 6;
      uint8_t sbus : 1;
      uint8_t sbus_inv : 1;
      uint8_t reserved : 6;
   };
   uint16_t val;
} bind_info_t;

typedef union
{
   struct
   {
      uint8_t sync_h;
      uint8_t sync_l;
   };
   uint16_t val;
} syncword_t;

typedef union
{
   struct
   {
      ch9_12switch_t ch9_12switch;
      bind_info_t bind_info;
      syncword_t syncword;
   };
   uint8_t ch9_12[5];
} last5ch_t;

typedef struct
{
   frameheader_t frameheader;
   uint8_t ch1_8[10]; // channel 1-8
   last5ch_t last5ch;
} frame_struct_t;

#define BIND_CHANNEL 96
#define FRAME_SIZE 16 //sizeof(bind_status)

#endif //__SX1280_RX_H__

static void setbind();

static void procces_bind_frame();

void radio_setparam();

static void frameproc();

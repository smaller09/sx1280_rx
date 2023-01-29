#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp8266/spi_struct.h"
#include "esp8266/pin_mux_register.h"
typedef union
{
   struct
   {
      uint32_t send_buff_32[8];
      uint32_t recv_buff_32[8];
   };
   struct
   {
      uint16_t send_buff_16[16];
      uint16_t recv_buff_16[16];
   };
   struct
   {
      uint8_t send_buff_8[32];
      uint8_t recv_buff_8[32];
   };
   /* data */
} sx1280_buff_t;
extern sx1280_buff_t data_buff;

void hspi_init();
void IRAM_ATTR hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);
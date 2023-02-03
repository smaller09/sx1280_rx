/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "sx1280_rx.h"
#include "sx1280.h"
#include "serial.h"
#include "driver/uart.h"
#include "ota.h"

WORD_ALIGNED_ATTR RTC_DATA_ATTR sx1280_buff_t spi_buf;

static void IRAM_ATTR mainloop(void *arg)
{
   for (;;)
   {
      WDT_FEED();
      vTaskDelay(10);
   }
   vTaskDelete(NULL);
}

void app_main()
{
   // UART0_Init();
   ota_init();
   
   SX1280_Init();

   vTaskDelay(5);

   SX1280SetStandby(STDBY_XOSC);

   vTaskDelay(5);

   xTaskCreate(mainloop, "mainloop", 1024, NULL, 10, NULL);

//   esp_task_wdt_init();

}

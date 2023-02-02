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

WORD_ALIGNED_ATTR RTC_DATA_ATTR sx1280_buff_t spi_buf;

static void IRAM_ATTR mainloop(void *arg)
{
   const char start[] = "WDT\n";

   for (;;)
   {
      WDT_FEED();
      vTaskDelay(10);
   }
   vTaskDelete(NULL);
}

void app_main()
{
   UART0_Init();
   SX1280_Init();

   RadioStatus_t status;
   status = SX1280GetStatus();
   uint16_t ver;
   ver = SX1280GetFirmwareVersion();
   xTaskCreate(mainloop, "mainloop", 1024, NULL, 10, NULL);
   esp_task_wdt_init();
}

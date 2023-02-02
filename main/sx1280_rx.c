/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "sx1280_rx.h"
#include "sx1280.h"
#include "serial.h"

WORD_ALIGNED_ATTR RTC_DATA_ATTR sx1280_buff_t spi_buf;

void app_main()
{
   
   SX1280_Init();
   uart0_init();
   
}

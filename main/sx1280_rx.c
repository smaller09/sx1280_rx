/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "sx1280_rx.h"

sx1280_buff_t spi_buf;
uint8_t package_size;


void gpio_config()
{
}

void app_main()
{
   gpio_config();
   hspi_init();
}

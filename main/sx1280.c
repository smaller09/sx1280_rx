/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
//#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


#include "esp8266/spi_struct.h"
#include "esp8266/gpio_struct.h"
//#include "esp_system.h"

#include "time.h"
#include "hspi.h"

void gpio_config()
{

}

void app_main()
{
   gpio_config();
   hspi_init();
}

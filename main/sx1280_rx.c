/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "sx1280_rx.h"
#include "sx1280.h"
#include "serial.h"
#include "ota.h"
#include "nvs.h"
#include "driver/gpio.h"
#define BIND_CHANNEL 96

sx1280_buff_t DRAM_ATTR spi_buf;

static uint8_t lora_channel;

static const char *RC_TABLE = "RC_TABLE";
// struct BIND_STATUS
static const char *BIND_STATUS = "BIND_STATUS";

static const char *TAG = "sx1280_rx";

static uint8_t isbinded =0;

static void rx_loop(void *arg)
{
   


   while (1)
   {
      WDT_FEED();
      vTaskDelay(10);
   }
   vTaskDelete(NULL);
}

void radio_init()
{
   nvs_handle_t status_handle;
   bind_status_t bind_status;
   bind_status.val = 0;
   isbinded = BIND_CHANNEL;
   if (nvs_open(RC_TABLE, NVS_READONLY, &status_handle) == ESP_OK)
   {
      if (nvs_get_u32(status_handle, BIND_STATUS, &bind_status.val) == ESP_OK)
      {
         isbinded = 0;
         SX1280SetLoraSyncWord(bind_status.syncword);
      }
   }
   nvs_close(status_handle);
   ModulationParams_t modulationParams;
   PacketParams_t packetParams;
   modulationParams.PacketType = PACKET_TYPE_LORA;
   modulationParams.Params.LoRa.Bandwidth = LORA_BW_0800;

   packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
   packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
   packetParams.Params.LoRa.HeaderType = LORA_PACKET_IMPLICIT;
   packetParams.PacketType = PACKET_TYPE_LORA;
   packetParams.Params.LoRa.PreambleLength = 12;
   modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_7;
   if ((isbinded == BIND_CHANNEL) || (bind_status.rc_mode == LLMODE))
   {
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF7;
      packetParams.Params.LoRa.PayloadLength = 14;
   }
   else
   {
      
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF8;
      packetParams.Params.LoRa.PayloadLength = 18;
   }

   SX1280SetPacketType(modulationParams.PacketType);

   SX1280SetModulationParams(&modulationParams);

   SX1280SetPacketParams(&packetParams);

   SX1280SetLoraMagicNum(LORA_MAGICNUMBER);

   SX1280SetRfFrequency(isbinded);
   
}

void dio_isr_handler(void* arg)
{
    //need to complete
}

static void gpio_init()
{
   // GPIO2:Reset, GPIO4:DIO1, GPIO5:BUSY, GPIO0: SW, GPIO16:LED
   gpio_config_t sx1280_gpio;

   sx1280_gpio.pin_bit_mask = (GPIO_Pin_2 | GPIO_Pin_16);
   sx1280_gpio.mode = GPIO_MODE_OUTPUT;
   sx1280_gpio.intr_type = GPIO_INTR_DISABLE;
   sx1280_gpio.pull_down_en = 0;
   sx1280_gpio.pull_up_en = 0;
   gpio_config(&sx1280_gpio);
   sx1280_gpio.pin_bit_mask = (GPIO_Pin_0 | GPIO_Pin_5);
   sx1280_gpio.mode = GPIO_MODE_INPUT;
   sx1280_gpio.intr_type = GPIO_INTR_DISABLE;
   sx1280_gpio.pull_down_en = 0;
   sx1280_gpio.pull_up_en = 1;
   gpio_config(&sx1280_gpio);

   sx1280_gpio.pin_bit_mask = (GPIO_Pin_4);
   sx1280_gpio.mode = GPIO_MODE_INPUT;
   sx1280_gpio.intr_type = GPIO_INTR_HIGH_LEVEL;
   sx1280_gpio.pull_down_en = 0;
   sx1280_gpio.pull_up_en = 0;
   gpio_config(&sx1280_gpio);

   gpio_install_isr_service(0);
   gpio_isr_handler_add(GPIO_NUM_4, dio_isr_handler, (void *) GPIO_NUM_4);

   gpio_set_level(GPIO_NUM_16, 0);
   gpio_set_level(GPIO_NUM_2, 1);
}


void app_main()
{

   ota_init();

   gpio_init();

   radio_init();

   SX1280_Init();

   vTaskDelay(5);

   esp_task_wdt_init();

   xTaskCreate(rx_loop, "mainloop", 1024, NULL, 10, NULL);
}

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
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#define BIND_CHANNEL 96
#define BIND_INFO_SIZE 16 //sizeof(bind_status)

sx1280_buff_t DRAM_ATTR WORD_ALIGNED_ATTR spi_buf;

static const char *RC_TABLE = "RC_TABLE";
// struct BIND_STATUS
static const char *BIND_STATUS = "BIND_STATUS";

static const char *TAG = "sx1280_rx";

static TickTime_t rxTimeout;
static TickTime_t txTimeout;

static uint8_t fhss_step;

static uint8_t fhss_ch = 96;

static bool islinked=false;

static bool unbinded = true;

static bool led=true;

frame_struct_t DRAM_ATTR frame_struct;

static void rx_loop(void *arg)
{
   while (1)
   {
      WDT_FEED();
      vTaskDelay(1000);
   }
   vTaskDelete(NULL);
}

void radio_init()
{
   nvs_handle_t status_handle;
   ESP_ERROR_CHECK(nvs_flash_init());
   if (nvs_open(RC_TABLE, NVS_READONLY, &status_handle) == ESP_OK)
   {
      ESP_LOGI(TAG, "NVS Opened!");
      size_t nvs_bind_size = BIND_INFO_SIZE;
      if (nvs_get_blob(status_handle, BIND_STATUS, &frame_struct, &nvs_bind_size) == ESP_OK)
      {
         unbinded = false;
         SX1280SetLoraSyncWord(frame_struct.last5byte.bind_info.sync_h, frame_struct.last5byte.bind_info.sync_l);
         fhss_step = frame_struct.last5byte.bind_info.rx_num;
         fhss_ch = 0;
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
   packetParams.Params.LoRa.PreambleLength = 0x23; //preamble length preamble length = LORA_PBLE_LEN_MANT*2^(LORA_PBLE_LEN_EXP)
   modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_7;
   if ((unbinded) || (frame_struct.frame_header.telemetry == LLMODE))
   {
      ESP_LOGI(TAG, "BIND_STATUS no set!");
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF7;
      packetParams.Params.LoRa.PayloadLength = 12;
   }
   else
   {
      
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF8; //LRmode
      packetParams.Params.LoRa.PayloadLength = 16;
   }



   SX1280SetPacketType(modulationParams.PacketType);
      
   SX1280SetRfFrequency(fhss_ch);

   SX1280SetModulationParams(&modulationParams);

   SX1280SetPacketParams(&packetParams);

   SX1280SetLoraMagicNum(LORA_MAGICNUMBER);
}

void radio_start()
{
   
}

void dio_isr_handler(void* arg)
{
    //need to complete
    SX1280ClearIrqStatus(IRQ_RADIO_ALL);
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

void led_loop()
{
   while (1)
   {
      if (unbinded)
      {
         vTaskDelay(500);
         led = !led;
      }
      else
      {
         vTaskDelay(10);
         led = islinked;
      }
      gpio_set_level(GPIO_NUM_16, led);
   }
}

void app_main()
{

   ota_init();

   gpio_init();

   SX1280_Init();

   radio_init();

   radio_start();
   
   esp_task_wdt_init();

   xTaskCreate(led_loop, "ledloop", configMINIMAL_STACK_SIZE, NULL, 6, NULL);

   xTaskCreate(rx_loop, "mainloop", 1024, NULL, 10, NULL);
}

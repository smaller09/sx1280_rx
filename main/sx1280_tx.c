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
#include "driver/hw_timer.h"
#include "esp_system.h"
#include "esp8266/timer_struct.h"
#define IRQERR (IRQ_CRC_ERROR | IRQ_SYNCWORD_ERROR)

sx1280_buff_t DRAM_ATTR WORD_ALIGNED_ATTR spi_buf;

static const char *RC_TABLE = "RC_TABLE";

static const char *BIND_STATUS = "BIND_STATUS";

static const char *TAG = "sx1280_tx";

static volatile uint8_t fhss_ch = BIND_CHANNEL;

static volatile bool islinked = false;

static volatile bool unbinded = true;

static volatile bool frame_ok = false;

static volatile bool telemetry;

static volatile uint8_t frame_mode = 0;

static uint8_t err_count_max;

static TaskHandle_t txloop_h = NULL;

frame_struct_t WORD_ALIGNED_ATTR DRAM_ATTR rc_frame;

frame_struct_t WORD_ALIGNED_ATTR DRAM_ATTR telemetry_frame;

static uint16_t time_interval;

static uint16_t frame_interval;

ModulationParams_t modulationParams;
PacketParams_t packetParams;

uint8_t rx_num = 1;
static BaseType_t content_switch = pdFALSE;
#define DEBUG
// #undef DEBUG

void dio_isr_handler(void *arg)
{

   xTaskNotifyFromISR(txloop_h, 1, eSetValueWithOverwrite, &content_switch);
   if (content_switch)
      portYIELD_FROM_ISR();
}

void timercb()
{
   xTaskNotifyFromISR(txloop_h, 2, eSetValueWithOverwrite, &content_switch);
   if (content_switch)
      portYIELD_FROM_ISR();
}

void radio_setparam()
{

   modulationParams.PacketType = PACKET_TYPE_LORA;
   modulationParams.Params.LoRa.Bandwidth = LORA_BW_0800;

   packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
   packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
   packetParams.Params.LoRa.HeaderType = LORA_PACKET_EXPLICIT;
   packetParams.PacketType = PACKET_TYPE_LORA;
   packetParams.Params.LoRa.PreambleLength = 6; // preamble length preamble length = LORA_PBLE_LEN_MANT*2^(LORA_PBLE_LEN_EXP)
   packetParams.Params.LoRa.PayloadLength = FRAME_SIZE;
   if ((unbinded) || (frame_mode == 0)) // unbinded or LLmode;
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF7;
   else
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF8; // LRmode
   modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_7;
   SX1280SetPacketType(modulationParams.PacketType);

   SX1280SetRfFrequency(fhss_ch);

   SX1280SetBufferBaseAddresses(txBaseAddress, rxBaseAddress);

   SX1280SetModulationParams(&modulationParams);

   SX1280SetPacketParams(&packetParams);

   SX1280SetLoraMagicNum(LORA_MAGICNUMBER);

   SX1280SetTxParams(28, RADIO_RAMP_02_US);
   SX1280SetDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR), IRQ_RADIO_NONE, IRQ_RADIO_NONE);

   SX1280ClearIrqStatus(IRQ_RADIO_ALL);

   switch (frame_mode) // only 2 mode support now.
   {
   case 0:
      err_count_max = 200;
      time_interval = 410;
      frame_interval = 10000;
      break;
   case 1:
      err_count_max = 100;
      time_interval = 2810;
      frame_interval = 20000;
      break;
   }

   ESP_LOGI(TAG, "unbinded: %d", unbinded);
   ESP_LOGI(TAG, "frame_mode: %d", frame_mode);
   ESP_LOGI(TAG, "fhss_ch: %d", fhss_ch);
   ESP_LOGI(TAG, "syncword %x", rc_frame.syncword);
}

static void procces_bind_frame()
{
   unbinded = true;
   SX1280SetStandby(STDBY_XOSC);

   fhss_ch = 96;

   rx_num = 15;
   rx_num &= 0x3f;
   frame_mode = 0; // frame mode

   rc_frame.frameheader.val = 0x00;
   rc_frame.chan01 = 511;
   rc_frame.chan02 = 511;
   rc_frame.chan03 = 511;
   rc_frame.chan04 = 511;
   rc_frame.chan05 = 511;
   rc_frame.chan06 = 511;
   rc_frame.chan07 = 511;
   rc_frame.chan08 = 511;

   rc_frame.ch9_12switch.val = 0;
   rc_frame.bind_info.failsafe = 0;
   rc_frame.bind_info.rx_num = rx_num;
   rc_frame.bind_info.sbus = 1;
   rc_frame.bind_info.sbus_inv = 0;
   rc_frame.bind_info.frame_mode = frame_mode;
   rc_frame.syncword = 0x5678;

   radio_setparam();
}

static void txloop(void *arg)
{
   uint32_t sx1280_notify;
   uint16_t irqstatus;
   uint8_t telemetry = 3;

   procces_bind_frame();

   SX1280SetLoraSyncWord(0x1424);
   
   for (uint8_t i = 0; i < 5; i++)
   {
      memcpy(&spi_buf.send_buf_8[1], rc_frame.rcdata, FRAME_SIZE); // frame
      SX1280SendPayload(FRAME_SIZE, RX_TX_SINGLE);

      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      SX1280ClearIrqStatus(IRQ_RADIO_ALL);
      vTaskDelay(10);
   }
   bool syncword_iq = rc_frame.syncword % 2;

   ESP_LOGI(TAG, "Main Loop");
   fhss_ch = rx_num;
   SX1280SetLoraSyncWord(rc_frame.syncword);

   SX1280SetRfFrequency(fhss_ch);
   packetParams.Params.LoRa.InvertIQ = ((fhss_ch % 2) ^ syncword_iq) ? LORA_IQ_NORMAL : LORA_IQ_INVERTED;
   SX1280SetPacketParams(&packetParams);

   vTaskDelay(100);
   while (1)
   {
      hw_timer_alarm_us(frame_interval, 0);
      telemetry++;
      telemetry = telemetry % 32;
      switch (telemetry)
      {
      case 1:
         rc_frame.frameheader.telemetry = 0;
         break;
      case 0:
         rc_frame.frameheader.telemetry = 1;
      default:
         memcpy(&spi_buf.send_buf_8[1], rc_frame.rcdata, FRAME_SIZE);
         SX1280SendPayload(FRAME_SIZE, RX_TX_SINGLE);
         break;
      }

      if ((telemetry == 2) && frame_ok)
      {
         SX1280GetPayload(FRAME_SIZE);
         memcpy(telemetry_frame.rcdata, spi_buf.recv_buf_8, FRAME_SIZE);
         /*   for (uint8_t i = 0; i < FRAME_SIZE; i++)
               printf(" %x", telemetry_frame.rcdata[i]);
            printf("\n");
            ESP_LOGI(TAG, "Telemetry"); */
      }

      fhss_ch = (fhss_ch * 2 + 5) % 101;

      sx1280_notify = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      irqstatus = SX1280GetIrqStatus();
      SX1280ClearIrqStatus(IRQ_RADIO_ALL);
      switch (sx1280_notify)
      {
      case 2:
         ESP_LOGI(TAG, "TX RX Timeout");
         frame_ok = false;
         SX1280SetStandby(STDBY_XOSC);

         break;

      case 1:

         if (irqstatus & IRQERR)
         {
            frame_ok = false;
            ESP_LOGI(TAG, "crc error");
         }
         else
         {
            islinked = true;
            frame_ok = true;
         }

         if (telemetry == 0)
            SX1280SetRx(RX_TX_SINGLE);
         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
         break;
      }
      SX1280SetRfFrequency(fhss_ch);
      packetParams.Params.LoRa.InvertIQ = ((fhss_ch % 2) ^ syncword_iq) ? LORA_IQ_NORMAL : LORA_IQ_INVERTED;
      SX1280SetPacketParams(&packetParams);
   }
   vTaskDelete(NULL);
}

void led_loop()
{
   bool led = 0;

   while (1)
   {
      if (islinked)
      {
         vTaskDelay(10);
         led = frame_ok;
      }
      else
      {
         if (unbinded)
            vTaskDelay(50);
         else
            vTaskDelay(500);
         led = !led;
      }
      gpio_set_level(GPIO_NUM_16, led);
   }
}

void app_main()
{
   esp_set_cpu_freq(ESP_CPU_FREQ_160M);
   ota_init();

   esp_task_wdt_init();

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

   SX1280_Init();

   sx1280_gpio.pin_bit_mask = (GPIO_Pin_4);
   sx1280_gpio.mode = GPIO_MODE_INPUT;
   sx1280_gpio.intr_type = GPIO_INTR_POSEDGE;
   sx1280_gpio.pull_up_en = 1;
   gpio_config(&sx1280_gpio);
   gpio_install_isr_service(0);
   gpio_isr_handler_add(GPIO_NUM_4, dio_isr_handler, (void *)GPIO_NUM_4);

   hw_timer_init(timercb, NULL);

   xTaskCreate(txloop, "mainloop", 2048, NULL, 12, &txloop_h);

   xTaskCreate(led_loop, "ledloop", configMINIMAL_STACK_SIZE / 2, NULL, 6, NULL);
}

void test()
{
   uint a = sizeof(rc_frame);
}

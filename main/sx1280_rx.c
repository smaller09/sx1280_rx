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

#define IRQERR (IRQ_CRC_ERROR | IRQ_SYNCWORD_ERROR | IRQ_RX_TX_TIMEOUT)

sx1280_buff_t DRAM_ATTR WORD_ALIGNED_ATTR spi_buf;

static const char *RC_TABLE = "RC_TABLE";

static const char *BIND_STATUS = "BIND_STATUS";

static const char *TAG = "sx1280_rx";

static volatile uint8_t fhss_ch = BIND_CHANNEL;

static volatile bool islinked = false;

static volatile bool unbinded = true;

static volatile bool frame_ok = false;

static volatile bool telemetry;

static volatile uint8_t frame_mode = 0;

static uint8_t err_count_max;

static TaskHandle_t rxloop_h = NULL;

frame_struct_t WORD_ALIGNED_ATTR DRAM_ATTR rc_frame;

frame_struct_t WORD_ALIGNED_ATTR DRAM_ATTR failsafe_frame;

frame_struct_t WORD_ALIGNED_ATTR DRAM_ATTR term_frame;

static uint16_t time_interval;

static uint16_t frame_interval;

static volatile uint32_t Tcount;

#define DEBUG
// #undef DEBUG

void dio_isr_handler(void *arg)
{
   BaseType_t content_switch = pdFALSE;
   // Tcount = frc1.count.data;
   xTaskNotifyFromISR(rxloop_h, 1, eSetValueWithOverwrite, &content_switch);
   if (content_switch)
      portYIELD_FROM_ISR();
}

void timercb()
{
   BaseType_t content_switch = pdFALSE;
   xTaskNotifyFromISR(rxloop_h, 2, eSetValueWithOverwrite, &content_switch);
   if (content_switch)
      portYIELD_FROM_ISR();
}

void radio_setparam()
{

   ModulationParams_t modulationParams;
   PacketParams_t packetParams;
   modulationParams.PacketType = PACKET_TYPE_LORA;
   modulationParams.Params.LoRa.Bandwidth = LORA_BW_0800;

   packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
   packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
   packetParams.Params.LoRa.HeaderType = LORA_PACKET_IMPLICIT;
   packetParams.PacketType = PACKET_TYPE_LORA;
   packetParams.Params.LoRa.PreambleLength = 0x0c; // preamble length preamble length = LORA_PBLE_LEN_MANT*2^(LORA_PBLE_LEN_EXP)
   modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_7;
   packetParams.Params.LoRa.PayloadLength = 16;
   if ((unbinded) || (frame_mode == 0)) // unbinded or LLmode;
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF7;
   else
      modulationParams.Params.LoRa.SpreadingFactor = LORA_SF8; // LRmode

   SX1280SetPacketType(modulationParams.PacketType);

   SX1280SetRfFrequency(fhss_ch);

   SX1280SetBufferBaseAddresses(txBaseAddress, rxBaseAddress);

   SX1280SetModulationParams(&modulationParams);

   SX1280SetPacketParams(&packetParams);

   SX1280SetLoraMagicNum(LORA_MAGICNUMBER);

   SX1280SetTxParams(18, RADIO_RAMP_02_US);
   SX1280SetDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_SYNCWORD_ERROR | IRQ_RX_TX_TIMEOUT), IRQ_RADIO_NONE, IRQ_RADIO_NONE);

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
   ESP_LOGI(TAG, "syncword %x %x", rc_frame.last4ch.syncword.sync_h, rc_frame.last4ch.syncword.sync_l);
}

static void setbind()
{
   SX1280SetStandby(STDBY_XOSC);
   SX1280SetLoraSyncWord(0x14, 0x24); // reset to default syncword;
   unbinded = true;
   ESP_LOGI(TAG, "Enter bind mode");
   fhss_ch = BIND_CHANNEL;
   vTaskDelay(1);
   radio_setparam();
}

static void procces_bind_frame()
{
   unbinded = false;
   SX1280SetStandby(STDBY_XOSC);
   SX1280SetLoraSyncWord(rc_frame.last4ch.syncword.sync_h, rc_frame.last4ch.syncword.sync_l);
   fhss_ch = rc_frame.last4ch.bind_info.rx_num;
   frame_mode = rc_frame.last4ch.bind_info.frame_mode; // frame mode in the telemetry bit.
   frame_mode &= 1;

   if (rc_frame.last4ch.bind_info.failsafe == 3)
   {

      failsafe_frame.frameheader.val = 0x00;

      failsafe_frame.ch1_8[0] = 0x7F; ////7F DF F0 01 FF
      failsafe_frame.ch1_8[1] = 0xDF;
      failsafe_frame.ch1_8[2] = 0xF0; //
      failsafe_frame.ch1_8[3] = 0x00; // channel3 (thr) set to 0
      failsafe_frame.ch1_8[4] = 0xFF;

      failsafe_frame.ch1_8[5] = 0x7F; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.ch1_8[6] = 0xDF; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.ch1_8[7] = 0xF7; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.ch1_8[8] = 0xFD; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.ch1_8[9] = 0xFF; //{0x7F,0xDF,0xF7,0xFD,0xFF}

      failsafe_frame.last4ch.ch9_12[0] = 0x7F; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.last4ch.ch9_12[1] = 0xDF; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.last4ch.ch9_12[2] = 0xF7; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.last4ch.ch9_12[3] = 0xFD; //{0x7F,0xDF,0xF7,0xFD,0xFF}
      failsafe_frame.last4ch.ch9_12[4] = 0xFF; //{0x7F,0xDF,0xF7,0xFD,0xFF}
   }
   if (rc_frame.last4ch.bind_info.failsafe == 2)
   {
      memcpy(failsafe_frame.rcdata, rc_frame.rcdata, 11); // frame
      uint16_t tmp;
      tmp = failsafe_frame.last4ch.ch9_12switch.ch9 * 511;
      failsafe_frame.last4ch.ch9_12[0] = (uint8_t)((tmp >> 2) & 0xff);
      failsafe_frame.last4ch.ch9_12[1] = ((uint8_t)(tmp & 0x03)) << 6;
      tmp = failsafe_frame.last4ch.ch9_12switch.ch10 * 511;
      failsafe_frame.last4ch.ch9_12[1] |= (uint8_t)((tmp >> 4) & 0x3f);
      failsafe_frame.last4ch.ch9_12[2] = ((uint8_t)(tmp & 0x0f)) << 4;
      tmp = failsafe_frame.last4ch.ch9_12switch.ch11 * 511;
      failsafe_frame.last4ch.ch9_12[2] |= (uint8_t)((tmp >> 6) & 0x0f);
      failsafe_frame.last4ch.ch9_12[3] = ((uint8_t)(tmp & 0x3f)) << 2;
      tmp = failsafe_frame.last4ch.ch9_12switch.ch11 * 511;
      failsafe_frame.last4ch.ch9_12[3] |= (uint8_t)((tmp >> 8) & 0x03);
      failsafe_frame.last4ch.ch9_12[4] = (uint8_t)(tmp & 0xff);
   }
   ESP_LOGI(TAG, "bind status read!");

   radio_setparam();
}

static void rxloop(void *arg)
{
   uint32_t sx1280_notify;
   uint16_t bind_count = 0;
   uint8_t err_count = 0;
   uint8_t frame_count = 0;
   uint8_t frame_err_count = 0;
   uint16_t irqstatus;
   SX1280SetRx(RX_TX_SINGLE);
   ESP_LOGI(TAG, "Main Loop");

   while (1)
   {

      sx1280_notify = ulTaskNotifyTake(pdTRUE, 10);
      if (sx1280_notify)
      {
         irqstatus = SX1280GetIrqStatus();
         SX1280ClearIrqStatus(IRQ_RADIO_ALL);
         if (irqstatus & IRQERR)
         {
            SX1280SetRx(RX_TX_SINGLE);
            ESP_LOGI(TAG, "freq %d", fhss_ch);
            continue;
         }

         if (unbinded)
         {
            SX1280GetPayload(FRAME_SIZE);
            memcpy(rc_frame.rcdata, spi_buf.recv_buf_8, FRAME_SIZE); // frame

            nvs_handle_t nvs_handle;
            nvs_open(RC_TABLE, NVS_READWRITE, &nvs_handle);
            ESP_LOGI(TAG, "NVS Opened!");
            nvs_set_blob(nvs_handle, BIND_STATUS, rc_frame.rcdata, FRAME_SIZE);
            ESP_LOGI(TAG, "Bind Status saved!");
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
            procces_bind_frame();
            SX1280SetRx(RX_TX_SINGLE);
            continue;
         }
         else
         {
            islinked = true;
            frame_ok = true;
            SX1280GetPayload(FRAME_SIZE);
            memcpy(rc_frame.rcdata, spi_buf.recv_buf_8, FRAME_SIZE); // frame
            hw_timer_alarm_us(time_interval, 0);
            fhss_ch = (fhss_ch * 2 + 5) % 101;
            SX1280SetRfFrequency(fhss_ch);

            ESP_LOGI(TAG, "First Frame!");
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            SX1280SetRx(RX_TX_SINGLE);
            break;
         }
      }
      if (gpio_get_level(GPIO_NUM_0) == 0)
         bind_count++;
      else
         bind_count = 0;

      if (bind_count > 30)
      {
         bind_count = 0;
         setbind();
      }
   }

   while (1)
   {

      hw_timer_alarm_us(frame_interval, 0);
      if (rc_frame.frameheader.telemetry)
      {
         memcpy(&spi_buf.send_buf_8[1], term_frame.rcdata, FRAME_SIZE); // frame
         SX1280SendPayload(FRAME_SIZE, RX_TX_SINGLE);
         rc_frame.frameheader.telemetry = 0;
      }

      frame_count++;
      if (frame_count >= 200)
      {
         if (err_count > 0)
            ESP_LOGI(TAG, "Error Count %d", err_count);
         frame_count = 0;
         err_count = 0;
      }

      sx1280_notify = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      switch (sx1280_notify)
      {
      case 1: // rxtxdone
         irqstatus = SX1280GetIrqStatus();
         SX1280ClearIrqStatus(IRQ_RADIO_ALL);
         if (irqstatus & IRQERR)
         {
            frame_ok = false;
            err_count++;
            frame_err_count++;
            ESP_LOGI(TAG, "crc error");
         }
         else
         {
            frame_ok = true;
            frame_err_count = 0;
            islinked = true;
            if (irqstatus & IRQ_RX_DONE)
            {
               SX1280GetPayload(FRAME_SIZE);
               memcpy(rc_frame.rcdata, spi_buf.recv_buf_8, FRAME_SIZE);
            } // frame
            if (rc_frame.frameheader.telemetry == 0)
               hw_timer_alarm_us(time_interval, 0);
            // ESP_LOGI(TAG, "Tcount :%d", Tcount);
         }
         fhss_ch = (fhss_ch * 2 + 5) % 101;
         SX1280SetRfFrequency(fhss_ch);
         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
         if (rc_frame.frameheader.telemetry == 0)
            SX1280SetRx(RX_TX_SINGLE);
         break;
      case 2: // rxtimeout
         frame_ok = false;
         err_count++;
         frame_err_count++;
         if (islinked)
         {
            SX1280SetFs();
            fhss_ch = (fhss_ch * 2 + 5) % 101;
            SX1280SetRfFrequency(fhss_ch);
            SX1280SetRx(RX_TX_SINGLE);
         }

         break;
      }
      if (frame_err_count >= err_count_max)
      {
         islinked = false;

         ESP_LOGI(TAG, "failsafe");
         ESP_LOGI(TAG, "CH: %d", fhss_ch);
         ESP_LOGI(TAG, "Tele: %d", rc_frame.frameheader.telemetry);
         SX1280SetRx(RX_TX_SINGLE);
         frame_err_count = 0;
      }
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
         vTaskDelay(1);
         led = frame_ok;
      }
      else
      {
         if (unbinded)
            vTaskDelay(5);
         else
            vTaskDelay(50);
         led = !led;
      }
      gpio_set_level(GPIO_NUM_16, led);
   }
}

static void radio_init()
{
   // GPIO2:Reset, GPIO4:DIO1, GPIO5:BUSY, GPIO0: SW, GPIO16:LED
   nvs_handle_t nvs_handle;
   ESP_ERROR_CHECK(nvs_flash_init());
   if (nvs_open(RC_TABLE, NVS_READONLY, &nvs_handle) == ESP_OK)
   {
      ESP_LOGI(TAG, "NVS Opened!");
      size_t nvs_bind_size = FRAME_SIZE;
      if (nvs_get_blob(nvs_handle, BIND_STATUS, rc_frame.rcdata, &nvs_bind_size) == ESP_OK)
         procces_bind_frame();
   }
   else
      radio_setparam();
   nvs_close(nvs_handle);
   ESP_LOGI(TAG, "radio init!");
   for (uint8_t i = 0; i < FRAME_SIZE; i++)
      term_frame.rcdata[i] = i;
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

   radio_init();

   sx1280_gpio.pin_bit_mask = (GPIO_Pin_4);
   sx1280_gpio.mode = GPIO_MODE_INPUT;
   sx1280_gpio.intr_type = GPIO_INTR_POSEDGE;
   sx1280_gpio.pull_up_en = 1;
   gpio_config(&sx1280_gpio);
   gpio_install_isr_service(0);
   gpio_isr_handler_add(GPIO_NUM_4, dio_isr_handler, (void *)GPIO_NUM_4);

   hw_timer_init(timercb, NULL);

   xTaskCreate(rxloop, "mainloop", 2048, NULL, 12, &rxloop_h);

   xTaskCreate(led_loop, "ledloop", configMINIMAL_STACK_SIZE / 2, NULL, 6, NULL);
}

void test()
{
   uint a = sizeof(rc_frame);
}

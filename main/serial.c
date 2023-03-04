#include "sx1280_rx.h"
#include "serial.h"
#define BUF_SIZE (128)
static QueueHandle_t uart0_queue;

static WORD_ALIGNED_ATTR DRAM_ATTR uint8_t crc8table[256];

static crsfrcPacket_t WORD_ALIGNED_ATTR DRAM_ATTR crsfrcframe;

static crsflinkstatusPacket_t WORD_ALIGNED_ATTR DRAM_ATTR crsflinkstatusframe;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    uint8_t *dtmp = (uint8_t *)malloc(BUF_SIZE);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, BUF_SIZE);
            switch (event.type)
            {
            // Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than other types of events.
            If we take too much time on data event, the queue might be full.*/
            case UART_DATA:
                // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
                // ESP_LOGI(TAG, "[DATA EVT]:");
                // uart_write_bytes(UART_NUM_0, (const char*)dtmp, event.size);

                // 串口数据处理

                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                // ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,UART_RXFIFO_TOUT_INT_ENA
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(uart0_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                // ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(uart0_queue);
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                // ESP_LOGI(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                // ESP_LOGI(TAG, "uart frame error");
                break;
            // Others
            default:
                // ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
                esp_task_wdt_reset();
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void UART0_Init()
{
    uart_config_t uart_config;

    uart_config.baud_rate = CRSF_BAUDRATE,
    uart_config.data_bits = UART_DATA_8_BITS,
    uart_config.parity = UART_PARITY_DISABLE,
    uart_config.stop_bits = UART_STOP_BITS_1,
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    crc8init(CRSF_CRC_POLY); // current crsf using d5 for poly

    memset(&crsflinkstatusframe, 0, sizeof(crsflinkstatusframe));
    crsflinkstatusframe.linkstatus.rf_Mode = 1; // for test
    memset(&crsfrcframe, 0, sizeof(crsfrcframe));
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE, BUF_SIZE, 16, &uart0_queue, 0);
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 9, NULL);
}

void crsfsendrc(frame_struct_t *frame)
{
    crsfrcframe.channels.ch1 = (frame->chan01 * 1.602150538 + 172);
    crsfrcframe.channels.ch2 = (frame->chan02 * 1.602150538 + 172);
    crsfrcframe.channels.ch3 = (frame->chan03 * 1.602150538 + 172);
    crsfrcframe.channels.ch4 = (frame->chan04 * 1.602150538 + 172);
    crsfrcframe.channels.ch5 = (frame->chan05 * 1.602150538 + 172);
    crsfrcframe.channels.ch6 = (frame->chan06 * 1.602150538 + 172);
    crsfrcframe.channels.ch7 = (frame->chan07 * 1.602150538 + 172);
    crsfrcframe.channels.ch8 = (frame->chan08 * 1.602150538 + 172);
    crsfrcframe.channels.ch9 = (frame->chan09 * 1.602150538 + 172);
    crsfrcframe.channels.ch10 = (frame->chan10 * 1.602150538 + 172);
    crsfrcframe.channels.ch11 = (frame->chan11 * 1.602150538 + 172);
    crsfrcframe.channels.ch12 = (frame->chan12 * 1.602150538 + 172);
    crsfrcframe.channels.ch13 = frame->frameheader.ch13 * 546 + 172;
    crsfrcframe.channels.ch14 = frame->frameheader.ch14 * 546 + 172;
    crsfrcframe.channels.ch15 = frame->frameheader.ch15 * 546 + 172;
    crsfrcframe.channels.ch16 = frame->frameheader.ch16 * 1639 + 172;
    crsfrcframe.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    crsfrcframe.header.frame_size = CRSF_FRAME_LENGTH_TYPE_CRC + CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE;
    crsfrcframe.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    crsfrcframe.crc8 = crc8calc((uint8_t *)&crsfrcframe.header.type, CRSF_FRAME_LENGTH_TYPE + CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);
    uart_write_bytes(UART_NUM_0, (char *)&crsfrcframe, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);
}

void crsfsendlinkstatus(PacketStatus_t *status)
{
    crsflinkstatusframe.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    crsflinkstatusframe.header.frame_size = CRSF_FRAME_LENGTH_TYPE_CRC + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE;
    crsflinkstatusframe.header.type = CRSF_FRAMETYPE_LINK_STATISTICS;
    crsflinkstatusframe.linkstatus.uplink_RSSI_1 = status->RssiPkt; // current only 1 antenna support;
    crsflinkstatusframe.linkstatus.uplink_RSSI_2 = 0;
    crsflinkstatusframe.linkstatus.uplink_Link_quality = status->LQI;
    crsflinkstatusframe.linkstatus.uplink_SNR = status->SnrPkt;
    crsflinkstatusframe.crc8 = crc8calc((uint8_t *)&crsflinkstatusframe.header.type, CRSF_FRAME_LENGTH_TYPE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE);
    uart_write_bytes(UART_NUM_0, (char *)&crsflinkstatusframe, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE);
}

void crc8init(uint8_t poly)
{
    uint8_t idx = 0;
    do
    {
        uint8_t crc = idx;
        for (uint8_t shift = 0; shift < 8; ++shift)
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        crc8table[idx] = crc;
        idx++;
    } while (idx);
}
uint8_t crc8calc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
        crc = crc8table[crc ^ *data++];
    return crc;
}

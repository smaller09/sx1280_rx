#include "serial.h"
#include "driver/uart.h"

#define BUF_SIZE (128)
static QueueHandle_t uart0_queue;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, BUF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_0);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than other types of events. 
                If we take too much time on data event, the queue might be full.*/
                case UART_DATA:
                    // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
                    //ESP_LOGI(TAG, "[DATA EVT]:");
                    //uart_write_bytes(UART_NUM_0, (const char*)dtmp, event.size);

                    //串口数据处理
                        
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    // ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    // ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    // ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    // ESP_LOGI(TAG, "uart frame error");
                    break;                
                //Others
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
    
        uart_config.baud_rate = 115200,
        uart_config.data_bits = UART_DATA_8_BITS,
        uart_config.parity = UART_PARITY_DISABLE,
        uart_config.stop_bits = UART_STOP_BITS_1,
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 100, &uart0_queue, 0);
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
/* 
    test
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp8266/eagle_soc.h"
#include "esp8266/spi_struct.h"
#include "esp8266/pin_mux_register.h"
#include "esp_libc.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "rom/ets_sys.h"
#include "hspi.h"

#define ENTER_CRITICAL() portENTER_CRITICAL()
#define EXIT_CRITICAL() portEXIT_CRITICAL()

#define spi_intr_enable() _xt_isr_unmask(1 << ETS_SPI_INUM)
#define spi_intr_disable() _xt_isr_mask(1 << ETS_SPI_INUM)
#define spi_intr_register(a, b) _xt_isr_attach(ETS_SPI_INUM, (a), (b))

#define DPORT_SPI_INT_STATUS_REG 0x3ff00020
#define DPORT_SPI_INT_STATUS_SPI0 BIT4
#define DPORT_SPI_INT_STATUS_SPI1 BIT7




void hspi_init()
{


}

// Copyright 2018-2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>

/* SPI bus CPOL and CPHA definition */
#define SPI_CPOL_LOW  0
#define SPI_CPOL_HIGH 1
#define SPI_CPHA_LOW  0
#define SPI_CPHA_HIGH 1

/* SPI bus data sequence definition */
#define SPI_BIT_ORDER_MSB_FIRST  1
#define SPI_BIT_ORDER_LSB_FIRST  0
#define SPI_BYTE_ORDER_MSB_FIRST 1
#define SPI_BYTE_ORDER_LSB_FIRST 0

/* SPI default bus interface parameter definition */
#define SPI_DEFAULT_INTERFACE   0x1C0    /* CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:0, BYTE_TX_ORDER:0, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0 */

/* SPI master default interrupt enable definition */
#define SPI_MASTER_DEFAULT_INTR_ENABLE 0x10    /* TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false */

/* SPI slave default interrupt enable definition */
#define SPI_SLAVE_DEFAULT_INTR_ENABLE 0x0F    /* TRANS_DONE: false, WRITE_STATUS: true, READ_STATUS: true, WRITE_BUFFER: true, READ_BUFFER: ture */

/* SPI event definition */
#define SPI_INIT_EVENT        0
#define SPI_TRANS_START_EVENT 1
#define SPI_TRANS_DONE_EVENT  2
#define SPI_DEINIT_EVENT      3

/* SPI slave transfer done interrupt status definition */
#define SPI_SLV_RD_BUF_DONE (BIT(0))
#define SPI_SLV_WR_BUF_DONE (BIT(1))
#define SPI_SLV_RD_STA_DONE (BIT(2))
#define SPI_SLV_WR_STA_DONE (BIT(3))
#define SPI_TRANS_DONE      (BIT(4))

typedef void (*spi_event_callback_t)(int event, void *arg);

/**
 * @brief SPI clock division factor enumeration
 */
typedef enum {
    SPI_2MHz_DIV  = 40,
    SPI_4MHz_DIV  = 20,
    SPI_5MHz_DIV  = 16,
    SPI_8MHz_DIV  = 10,
    SPI_10MHz_DIV = 8,
    SPI_16MHz_DIV = 5,
    SPI_20MHz_DIV = 4,
    SPI_40MHz_DIV = 2,
    SPI_80MHz_DIV = 1,
} spi_clk_div_t;

/**
 * @brief SPI working mode enumeration
 */
typedef enum {
    SPI_MASTER_MODE,
    SPI_SLAVE_MODE
} spi_mode_t;

/**
 * @brief SPI interrupt enable union type definition
 */
typedef union {
    struct {
        uint32_t read_buffer:  1;    /*!< configurate intterrupt to enable reading */ 
        uint32_t write_buffer: 1;    /*!< configurate intterrupt to enable writing */ 
        uint32_t read_status:  1;    /*!< configurate intterrupt to enable reading status */ 
        uint32_t write_status: 1;    /*!< configurate intterrupt to enable writing status */ 
        uint32_t trans_done:   1;    /*!< configurate intterrupt to enable transmission done */ 
        uint32_t reserved5:    27;   /*!< reserved */
    };                               /*!< not filled */
    uint32_t val;                    /*!< union fill */ 
} spi_intr_enable_t;

/**
 * @brief SPI bus interface parameter union type definition
 */
typedef union {
    struct {
        uint32_t cpol:          1;   /*!< Clock Polarity */
        uint32_t cpha:          1;   /*!< Clock Phase */
        uint32_t bit_tx_order:  1;   /*!< Tx bit order */
        uint32_t bit_rx_order:  1;   /*!< Rx bit order */
        uint32_t byte_tx_order: 1;   /*!< Tx byte order */
        uint32_t byte_rx_order: 1;   /*!< Rx byte order */
        uint32_t mosi_en:       1;   /*!< MOSI line enable */
        uint32_t miso_en:       1;   /*!< MISO line enable */
        uint32_t cs_en:         1;   /*!< CS line enable */
        uint32_t reserved9:    23;   /*!< resserved */
    };                               /*!< not filled */
    uint32_t val;                    /*!< union fill */ 
} spi_interface_t;

/**
 * @brief SPI transmission parameter structure type definition
 */
typedef struct {
    uint16_t *cmd;                  /*!< SPI transmission command */  
    uint32_t *addr;                 /*!< SPI transmission address */  
    uint32_t *mosi;                 /*!< SPI transmission MOSI buffer, in order to improve the transmission efficiency, it is recommended that the external incoming data is (uint32_t *) type data, do not use other type data. */  
    uint32_t *miso;                 /*!< SPI transmission MISO buffer, in order to improve the transmission efficiency, it is recommended that the external incoming data is (uint32_t *) type data, do not use other type data. */  
    union {
        struct {
            uint32_t cmd:   5;      /*!< SPI transmission command bits */  
            uint32_t addr:  7;      /*!< SPI transmission address bits */ 
            uint32_t mosi: 10;      /*!< SPI transmission MOSI buffer bits */  
            uint32_t miso: 10;      /*!< SPI transmission MISO buffer bits */ 
        };                          /*!< not filled */
        uint32_t val;               /*!< union fill */ 
    } bits;                         /*!< SPI transmission packet members' bits */  
} spi_trans_t;

void spi_set_clk_div(spi_clk_div_t *clk_div);

void spi_set_intr_enable(spi_intr_enable_t *intr_enable);

void spi_set_mode(spi_mode_t *mode);

void spi_set_dummy(spi_host_t host, uint16_t *bitlen);

void spi_set_interface(spi_interface_t *interface);

void hspi_set_event_callback(spi_event_callback_t *event_cb);

void hspi_slave_get_status(uint32_t *status);

void hspi_slave_set_status(uint32_t *status);

void hspi_trans(spi_trans_t *trans);

void hspi_init(spi_config_t *config);

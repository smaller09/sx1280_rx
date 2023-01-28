#include "esp8266/spi_struct.h"
#include "esp8266/pin_mux_register.h"
#include "hspi.h"
#define bool	_Bool
#define true	1
#define false	0

void hspi_setmode()
{
    SPI1.user.flash_mode = false;
    SPI1.pin.slave_mode = false;
    SPI1.slave.slave_mode = false;
    // Master uses the entire hardware buffer to improve transmission speed
    SPI1.user.usr_mosi_highpart = false;
    SPI1.user.usr_miso_highpart = false;
    SPI1.user.usr_mosi = true;
    // Create hardware cs in advance
    SPI1.user.cs_setup = true;
    // Hysteresis to keep hardware cs
    SPI1.user.cs_hold = true;
    SPI1.user.duplex = true;
    SPI1.user.ck_i_edge = true;
    SPI1.ctrl2.mosi_delay_num = 0;
    SPI1.ctrl2.miso_delay_num = 1;
    
    SPI1.user.fwrite_dual = false;
    SPI1.user.fwrite_quad = false;
    SPI1.user.fwrite_dio  = false;
    SPI1.user.fwrite_qio  = false;
    SPI1.ctrl.fread_dual  = false;
    SPI1.ctrl.fread_quad  = false;
    SPI1.ctrl.fread_dio   = false;
    SPI1.ctrl.fread_qio   = false;
    SPI1.ctrl.fastrd_mode = true;
    SPI1.slave.sync_reset = 1;
}

void hspi_set_interface()
{
    // Initialize HSPI IO
    PIN_PULLUP_EN(PERIPHS_IO_MUX_MTMS_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_HSPI_CLK); //GPIO14 is SPI CLK pin (Clock)
    
    PIN_PULLUP_EN(PERIPHS_IO_MUX_MTCK_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_HSPID_MOSI); //GPIO13 is SPI MOSI pin (Master Data Out)

    PIN_PULLUP_EN(PERIPHS_IO_MUX_MTDI_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_HSPIQ_MISO); //GPIO12 is SPI MISO pin (Master Data In)

    PIN_PULLUP_EN(PERIPHS_IO_MUX_MTDO_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_HSPI_CS0);
    
    // Set the clock polarity and phase
    SPI1.pin.ck_idle_edge = false;
    SPI1.user.ck_out_edge = false;

    // Set data bit order
    SPI1.ctrl.wr_bit_order = 0;
    SPI1.ctrl.rd_bit_order = 0;

    // Set data byte order
    SPI1.user.wr_byte_order = 1;
    SPI1.user.rd_byte_order = 1;
    
}

void hspi_set_clk_div()
{
    // Configure the IO_MUX clock (required, otherwise the clock output will be confusing)
    CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, SPI1_CLK_EQU_SYS_CLK);
    SPI1.clock.clk_equ_sysclk = false;
    SPI1.clock.clkdiv_pre = 0;
    SPI1.clock.clkcnt_n = 4;
    // In the master mode clkcnt_h = floor((clkcnt_n+1)/2-1). In the slave mode it must be 0
    SPI1.clock.clkcnt_h = 2;
    // In the master mode clkcnt_l = clkcnt_n. In the slave mode it must be 0
    SPI1.clock.clkcnt_l = 3;
}

void hspi_init()
{
    hspi_setmode();
    hspi_set_interface();
    hspi_set_clk_div();

}

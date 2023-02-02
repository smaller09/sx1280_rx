#include "sx1280.h"
#include "sx1280_rx.h"
#include "driver/gpio.h"
#include "driver/spi.h"

/**
  * @brief SPI data transfer function
  *
  * @note If the bit of the corresponding phase in the transmission parameter is 0, its data will not work.
  *       For example: trans.bits.cmd = 0, cmd will not be transmitted
  *
  * @param host SPI peripheral number
  *     - CSPI_HOST SPI0
  *     - HSPI_HOST SPI1
  *
  * @param trans Pointer to transmission parameter structure
  *
  * @return
  *     - ESP_OK Success
  *     - ESP_ERR_INVALID_ARG Parameter error
  *     - ESP_FAIL spi has not been initialized yet
  */
// esp_err_t spi_trans(spi_host_t host, spi_trans_t *trans);

extern  sx1280_buff_t DRAM_ATTR spi_buf;

const uint8_t txBaseAddress = 0x00, rxBaseAddress = 0x80;

RTC_DATA_ATTR spi_trans_t trans;

static void IRAM_ATTR hspi_trans(uint16_t cmd, uint8_t dout_bits, uint8_t din_bits)
{
    trans.cmd=&cmd;
    trans.bits.mosi = dout_bits;
    trans.bits.miso = din_bits;
    spi_trans(HSPI_HOST, &trans);
}

uint8_t IRAM_ATTR SX1280GetPayload(uint8_t size)
{

    hspi_trans(RADIO_GET_RXBUFFERSTATUS, 0, 24);

    if (spi_buf.recv_buf_8[1] != size)
        return 1;
    spi_buf.send_buf_8[0] = txBaseAddress;
    hspi_trans(RADIO_READ_BUFFER, 8, (size + 1) * 8);
    return 0;
}
// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);
void IRAM_ATTR SX1280SetRx(TickTime_t timeout)
{

    spi_buf.send_buf_8[0] = timeout.Step;
    spi_buf.send_buf_8[1] = (uint8_t)((timeout.NbSteps >> 8) & 0x00FF);
    spi_buf.send_buf_8[2] = (uint8_t)(timeout.NbSteps & 0x00FF);

    SX1280ClearIrqStatus(IRQ_RADIO_ALL);

    hspi_trans(RADIO_SET_RX, 24, 0);
}

void IRAM_ATTR SX1280ClearIrqStatus(uint16_t irq)
{

    spi_buf.send_buf_8[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
    spi_buf.send_buf_8[1] = (uint8_t)((uint16_t)irq & 0x00FF);

    hspi_trans(RADIO_CLR_IRQSTATUS, 16, 0);
}

// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);
void IRAM_ATTR SX1280SetTx(TickTime_t timeout)
{

    spi_buf.send_buf_8[0] = timeout.Step;
    spi_buf.send_buf_8[1] = (uint8_t)((timeout.NbSteps >> 8) & 0x00FF);
    spi_buf.send_buf_8[2] = (uint8_t)(timeout.NbSteps & 0x00FF);

    SX1280ClearIrqStatus(IRQ_RADIO_ALL);

    hspi_trans(RADIO_SET_TX, 24, 0);
}

void IRAM_ATTR SX1280SendPayload(uint8_t size, TickTime_t timeout)
{
    spi_buf.send_buf_8[0] = txBaseAddress;
    hspi_trans(RADIO_WRITE_BUFFER, (size + 1) * 8, 0);

    SX1280SetTx(timeout);
}

// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);

void IRAM_ATTR SX1280SetStandby(RadioStandbyModes_t standbyConfig)
{
    spi_buf.send_buf_8[0] = standbyConfig;
    hspi_trans(RADIO_SET_STANDBY, 8, 0);
}

void IRAM_ATTR SX1280SetRegulatorMode(RadioRegulatorModes_t mode)
{
    spi_buf.send_buf_8[0] = mode;
    hspi_trans(RADIO_SET_REGULATORMODE, 8, 0);
}

void IRAM_ATTR SX1280SetPacketType(RadioPacketTypes_t packetType)
{
    spi_buf.send_buf_8[0] = packetType;
    hspi_trans(RADIO_SET_PACKETTYPE, 8, 0);
}

void IRAM_ATTR SX1280SetModulationParams(ModulationParams_t *modulationParams)
{

    spi_buf.send_buf_8[0] = modulationParams->Params.LoRa.SpreadingFactor;
    spi_buf.send_buf_8[1] = modulationParams->Params.LoRa.Bandwidth;
    spi_buf.send_buf_8[2] = modulationParams->Params.LoRa.CodingRate;
    hspi_trans(RADIO_SET_MODULATIONPARAMS, 24, 0);
}

void IRAM_ATTR SX1280SetPacketParams(PacketParams_t *packetParams)
{

    spi_buf.send_buf_8[0] = packetParams->Params.LoRa.PreambleLength;
    spi_buf.send_buf_8[1] = packetParams->Params.LoRa.HeaderType;
    spi_buf.send_buf_8[2] = packetParams->Params.LoRa.PayloadLength;
    spi_buf.send_buf_8[3] = packetParams->Params.LoRa.CrcMode;
    spi_buf.send_buf_8[4] = packetParams->Params.LoRa.InvertIQ;
    spi_buf.send_buf_8[5] = NULL;
    spi_buf.send_buf_8[6] = NULL;

    hspi_trans(RADIO_SET_PACKETPARAMS, 48, 0);
}

void IRAM_ATTR SX1280SetBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{

    spi_buf.send_buf_8[0] = txBaseAddress;
    spi_buf.send_buf_8[1] = rxBaseAddress;
    hspi_trans(RADIO_SET_BUFFERBASEADDRESS, 16, 0);
}

void IRAM_ATTR SX1280SetTxParams(int8_t power, RadioRampTimes_t rampTime)
{

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    spi_buf.send_buf_8[0] = power + 18;
    spi_buf.send_buf_8[1] = (uint8_t)rampTime;
    hspi_trans(RADIO_SET_TXPARAMS, 16, 0);
}

void IRAM_ATTR SX1280SetAutoFS(uint8_t enable)
{
    spi_buf.send_buf_8[0] = enable;
    hspi_trans(RADIO_SET_AUTOFS, 8, 0);
}

void IRAM_ATTR SX1280SetRfFrequency(uint8_t channel)
{
    double frequency;
    frequency = 2400000000 + 1000000 * channel;
    uint32_t freq;

    freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
    spi_buf.send_buf_8[0] = (uint8_t)((freq >> 16) & 0xFF);
    spi_buf.send_buf_8[1] = (uint8_t)((freq >> 8) & 0xFF);
    spi_buf.send_buf_8[2] = (uint8_t)(freq & 0xFF);
    hspi_trans(RADIO_SET_RFFREQUENCY, 24, 0);
}

void SX1280Reset(void)
{
    gpio_set_level(GPIO_NUM_2,0);
    vTaskDelay(5);
    gpio_set_level(GPIO_NUM_2,1);
}

RadioStatus_t IRAM_ATTR SX1280GetStatus(void)
{
    RadioStatus_t status;
    hspi_trans(RADIO_GET_STATUS, 0, 8);
    status.Value = spi_buf.recv_buf_8[0];
    return status;
}

uint16_t IRAM_ATTR SX1280GetIrqStatus(void)
{
    // SX1280HalReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    hspi_trans(RADIO_GET_IRQSTATUS, 0, 24);
    return (spi_buf.recv_buf_8[1] << 8) | spi_buf.recv_buf_8[2];
}

void IRAM_ATTR SX1280SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{

    spi_buf.send_buf_8[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    spi_buf.recv_buf_8[1] = (uint8_t)(irqMask & 0x00FF);
    spi_buf.recv_buf_8[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    spi_buf.recv_buf_8[3] = (uint8_t)(dio1Mask & 0x00FF);
    spi_buf.recv_buf_8[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    spi_buf.recv_buf_8[5] = (uint8_t)(dio2Mask & 0x00FF);
    spi_buf.recv_buf_8[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    spi_buf.recv_buf_8[7] = (uint8_t)(dio3Mask & 0x00FF);

    hspi_trans(RADIO_SET_DIOIRQPARAMS, 64, 0);
}


void gpio_init()
{
    // GPIO2:Reset, GPIO4:DIO1, GPIO5:BUSY, GPIO0: SW, GPIO16:LED
    gpio_config_t sx1280_gpio;
    
    sx1280_gpio.pin_bit_mask=(GPIO_Pin_2 | GPIO_Pin_16);
    sx1280_gpio.mode = GPIO_MODE_OUTPUT;
    sx1280_gpio.intr_type = GPIO_INTR_DISABLE;
    sx1280_gpio.pull_down_en = 0;
    sx1280_gpio.pull_up_en = 0;
    gpio_config(&sx1280_gpio);
    sx1280_gpio.pin_bit_mask=(GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_5);
    sx1280_gpio.mode = GPIO_MODE_INPUT;
    sx1280_gpio.intr_type = GPIO_INTR_DISABLE;
    sx1280_gpio.pull_down_en = 0;
    sx1280_gpio.pull_up_en = 1;
    gpio_config(&sx1280_gpio);
    gpio_set_level(GPIO_NUM_16, 0);
    gpio_set_level(GPIO_NUM_2, 1);
}
void hspi_init()
{
    spi_config_t hspi_config;
    // Load default interface parameters
    // CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
    hspi_config.interface.val = SPI_DEFAULT_INTERFACE;

    // Load default interrupt enable
    // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false
    hspi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    // Set SPI to master mode
    // ESP8266 Only support half-duplex
    hspi_config.mode = SPI_MASTER_MODE;
    // Set the SPI clock frequency division factor
    hspi_config.clk_div = SPI_16MHz_DIV;
    // Register SPI event callback function
    hspi_config.event_cb = NULL;
    spi_init(HSPI_HOST, &hspi_config);

    trans.bits.addr = 0;
    trans.bits.cmd = 8;
    trans.addr = NULL;
    trans.miso = (spi_buf.recv_buf_32);
    trans.mosi = (spi_buf.send_buf_32);
}

// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);

uint16_t SX1280GetFirmwareVersion( void )
{
    spi_buf.send_buf_8[0] = (uint8_t)(((uint16_t)REG_LR_FIRMWARE_VERSION_MSB >> 8) & 0x00FF);
    spi_buf.send_buf_8[1] = (uint8_t)((uint16_t)REG_LR_FIRMWARE_VERSION_MSB & 0x00FF);

    hspi_trans(RADIO_READ_REGISTER, 16, 24);
    uint16_t ver;
    ver=spi_buf.recv_buf_8[1];
    return ((ver << 8 ) | spi_buf.recv_buf_8[2]);
}

uint8_t SX1280GetLNARegime()
{
    spi_buf.send_buf_8[0] = (uint8_t)(((uint16_t)REG_LNA_REGIME >> 8) & 0x00FF);
    spi_buf.send_buf_8[1] = (uint8_t)((uint16_t)REG_LNA_REGIME & 0x00FF);
    hspi_trans(RADIO_READ_REGISTER, 16, 16);
    return spi_buf.recv_buf_8[1];
}
void SX1280_Init()
{
    gpio_init();

    hspi_init();

    SX1280Reset();

}
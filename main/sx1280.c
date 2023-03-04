#include "sx1280_rx.h"
#include "sx1280.h"
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

extern sx1280_buff_t spi_buf;

/*      frequency = 2400000000 + 1000000 * i;
        freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
        freq_h = (uint8_t)((freq >> 16) & 0xFF);
        freq_m = (uint8_t)((freq >> 8) & 0xFF);
        freq_l = (uint8_t)(freq & 0xFF);
*/
static DRAM_ATTR int32_t fhss_freq[101] = {0};

static spi_trans_t trans;

static void hspi_trans(uint16_t cmd, uint8_t dout_bits, uint8_t din_bits)
{
    trans.cmd = &cmd;
    trans.bits.mosi = dout_bits;
    trans.bits.miso = din_bits;
    spi_trans(HSPI_HOST, &trans);
    while (gpio_get_level(GPIO_NUM_5))
        ;
}

void SX1280GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBuffer)
{
    hspi_trans(RADIO_GET_RXBUFFERSTATUS, 0, 24);
    *payloadLength = spi_buf.send_buf_8[1];
    *rxStartBuffer = spi_buf.send_buf_8[2];
}

void SX1280GetPayload(uint8_t size)
{
    spi_buf.send_buf_8[0] = rxBaseAddress;
    spi_buf.send_buf_8[1] = 0;
    hspi_trans(RADIO_READ_BUFFER, 16, (size)*8);
}
// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);
void SX1280SetRx(TickTime_t timeout)
{

    spi_buf.send_buf_8[0] = timeout.Step;
    spi_buf.send_buf_8[1] = (uint8_t)(timeout.NbSteps >> 8);
    spi_buf.send_buf_8[2] = (uint8_t)(timeout.NbSteps & 0xFF);

    // SX1280ClearIrqStatus(IRQ_RADIO_ALL);

    hspi_trans(RADIO_SET_RX, 24, 0);
}

void SX1280ClearIrqStatus(uint16_t irq)
{

    spi_buf.send_buf_8[0] = (uint8_t)((uint16_t)irq >> 8);
    spi_buf.send_buf_8[1] = (uint8_t)((uint16_t)irq & 0xFF);

    hspi_trans(RADIO_CLR_IRQSTATUS, 16, 0);
}

// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);
void SX1280SetTx(TickTime_t timeout)
{

    spi_buf.send_buf_8[0] = timeout.Step;
    spi_buf.send_buf_8[1] = (uint8_t)(timeout.NbSteps >> 8);
    spi_buf.send_buf_8[2] = (uint8_t)(timeout.NbSteps & 0xFF);

    // SX1280ClearIrqStatus(IRQ_RADIO_ALL);

    hspi_trans(RADIO_SET_TX, 24, 0);
}

void SX1280SendPayload(uint8_t size, TickTime_t timeout)
{
    spi_buf.send_buf_8[0] = txBaseAddress;
    hspi_trans(RADIO_WRITE_BUFFER, (size + 1) * 8, 0);

    SX1280SetTx(timeout);
}

// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);

void SX1280SetStandby(RadioStandbyModes_t standbyConfig)
{
    spi_buf.send_buf_8[0] = standbyConfig;
    hspi_trans(RADIO_SET_STANDBY, 8, 0);
}

void SX1280SetRegulatorMode(RadioRegulatorModes_t mode)
{
    spi_buf.send_buf_8[0] = mode;
    hspi_trans(RADIO_SET_REGULATORMODE, 8, 0);
}

void SX1280SetPacketType(RadioPacketTypes_t packetType)
{
    spi_buf.send_buf_8[0] = packetType;
    hspi_trans(RADIO_SET_PACKETTYPE, 8, 0);
}

void SX1280SetModulationParams(ModulationParams_t *modulationParams)
{

    spi_buf.send_buf_8[0] = modulationParams->Params.LoRa.SpreadingFactor;
    spi_buf.send_buf_8[1] = modulationParams->Params.LoRa.Bandwidth;
    spi_buf.send_buf_8[2] = modulationParams->Params.LoRa.CodingRate;
    hspi_trans(RADIO_SET_MODULATIONPARAMS, 24, 0);
}

void SX1280SetPacketParams(PacketParams_t *packetParams)
{

    spi_buf.send_buf_8[0] = packetParams->Params.LoRa.PreambleLength;
    spi_buf.send_buf_8[1] = packetParams->Params.LoRa.HeaderType;
    spi_buf.send_buf_8[2] = packetParams->Params.LoRa.PayloadLength;
    spi_buf.send_buf_8[3] = packetParams->Params.LoRa.CrcMode;
    spi_buf.send_buf_8[4] = packetParams->Params.LoRa.InvertIQ;
    spi_buf.send_buf_8[5] = 0;
    spi_buf.send_buf_8[6] = 0;

    hspi_trans(RADIO_SET_PACKETPARAMS, 56, 0);
}

void SX1280SetBufferBaseAddresses(uint8_t txBase, uint8_t rxBase)
{

    spi_buf.send_buf_8[0] = txBase;
    spi_buf.send_buf_8[1] = rxBase;
    hspi_trans(RADIO_SET_BUFFERBASEADDRESS, 16, 0);
}

void SX1280SetTxParams(uint8_t power, RadioRampTimes_t rampTime)
{

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    spi_buf.send_buf_8[0] = power;
    spi_buf.send_buf_8[1] = (uint8_t)rampTime;
    hspi_trans(RADIO_SET_TXPARAMS, 16, 0);
}

void SX1280SetAutoFS(uint8_t enable)
{
    spi_buf.send_buf_8[0] = enable;
    hspi_trans(RADIO_SET_AUTOFS, 8, 0);
}

void SX1280SetRfFrequency(uint8_t channel)
{
    spi_buf.send_buf_8[2] = fhss_freq[channel] & 0xFF;
    spi_buf.send_buf_8[1] = (fhss_freq[channel] >> 8) & 0xFF;
    spi_buf.send_buf_8[0] = (fhss_freq[channel] >> 16) & 0xFF;
    hspi_trans(RADIO_SET_RFFREQUENCY, 24, 0);
}

void SX1280Reset(void)
{
    gpio_set_level(GPIO_NUM_2, 0);
    vTaskDelay(1);
    gpio_set_level(GPIO_NUM_2, 1);
}

RadioStatus_t SX1280GetStatus(void)
{
    RadioStatus_t status;
    hspi_trans(RADIO_GET_STATUS, 0, 8);
    status.Value = spi_buf.recv_buf_8[0];
    return status;
}

uint16_t SX1280GetIrqStatus(void)
{
    // SX1280HalReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    hspi_trans(RADIO_GET_IRQSTATUS, 0, 24);
    uint16_t val = 0;
    val = spi_buf.recv_buf_8[1];
    val = (val << 8) | spi_buf.recv_buf_8[2];
    return val;
}

void SX1280SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{

    spi_buf.send_buf_8[0] = (uint8_t)(irqMask >> 8);
    spi_buf.send_buf_8[1] = (uint8_t)(irqMask & 0xFF);
    spi_buf.send_buf_8[2] = (uint8_t)(dio1Mask >> 8);
    spi_buf.send_buf_8[3] = (uint8_t)(dio1Mask & 0xFF);
    spi_buf.send_buf_8[4] = (uint8_t)(dio2Mask >> 8);
    spi_buf.send_buf_8[5] = (uint8_t)(dio2Mask & 0xFF);
    spi_buf.send_buf_8[6] = (uint8_t)(dio3Mask >> 8);
    spi_buf.send_buf_8[7] = (uint8_t)(dio3Mask & 0xFF);

    hspi_trans(RADIO_SET_DIOIRQPARAMS, 64, 0);
}

static void hspi_init()
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

void SX1280SetLoraSyncWord(uint16_t syncword)
{
    spi_buf.send_buf_8[0] = (uint8_t)(REG_LORASYNCWORD >> 8);
    spi_buf.send_buf_8[1] = (uint8_t)(REG_LORASYNCWORD & 0xff);
    spi_buf.send_buf_16[1] = syncword;

    //    spi_buf.send_buf_16[1]=Syncword;
    hspi_trans(RADIO_WRITE_REGISTER, 32, 0);
}

void SX1280SetLoraMagicNum(uint8_t MagicNum)
{
    spi_buf.send_buf_8[0] = (uint8_t)(REG_LORAMAGICNUM >> 8);
    spi_buf.send_buf_8[1] = (uint8_t)(REG_LORAMAGICNUM & 0xff);
    spi_buf.send_buf_8[2] = MagicNum;
    hspi_trans(RADIO_WRITE_REGISTER, 24, 0);
    spi_buf.send_buf_8[0] = (uint8_t)(REG_CrystalFEC >> 8);
    spi_buf.send_buf_8[1] = (uint8_t)(REG_CrystalFEC & 0xff);
    spi_buf.send_buf_8[2] = 0x01;
    hspi_trans(RADIO_WRITE_REGISTER, 24, 0); // In all cases 0x1 must be written to the Frequency Error Compensation mode register 0x093C
}

int32_t SX1280complement2(const uint32_t num, const uint8_t bitCnt)
{
    int32_t retVal = (int32_t)num;
    if (num >= 2 << (bitCnt - 2))
    {
        retVal -= 2 << (bitCnt - 1);
    }
    return retVal;
}

int32_t SX1280GetFrequencyError(uint8_t ch, bool syncword_iq)
{
    int32_t efe = 0;

    spi_buf.send_buf_8[0] = (REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB >> 8) & 0xFF;
    spi_buf.send_buf_8[1] = REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB & 0xFF;
    hspi_trans(RADIO_READ_REGISTER, 16, 32);

    efe = (spi_buf.recv_buf_8[1] << 16) | (spi_buf.recv_buf_8[2] << 8) | spi_buf.recv_buf_8[3];
    efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

    efe = (int32_t)(((double)SX1280complement2(efe, 20)) * 0.003968); // / (1600.0 / (double)812500.0 * 1000.0)
    if (syncword_iq ^ (ch % 2))
        fhss_freq[ch] += efe;
    else
        fhss_freq[ch] -= efe;
    return efe;
}

uint16_t SX1280GetFirmwareVersion(void)
{
    spi_buf.send_buf_8[0] = (REG_LR_FIRMWARE_VERSION_MSB >> 8) & 0xFF;
    spi_buf.send_buf_8[1] = REG_LR_FIRMWARE_VERSION_MSB & 0xFF;

    hspi_trans(RADIO_READ_REGISTER, 16, 24);
    uint16_t ver;
    ver = spi_buf.recv_buf_8[1];
    return ((ver << 8) | spi_buf.recv_buf_8[2]);
}

uint8_t SX1280GetLNARegime()
{
    spi_buf.send_buf_8[0] = (uint8_t)(((uint16_t)REG_LNA_REGIME >> 8) & 0x00FF);
    spi_buf.send_buf_8[1] = (uint8_t)((uint16_t)REG_LNA_REGIME & 0x00FF);
    hspi_trans(RADIO_READ_REGISTER, 16, 16);
    return spi_buf.recv_buf_8[1];
}

void SX1280GetPacketStatus(PacketStatus_t *pktStatus)
{
    hspi_trans(RADIO_GET_PACKETSTATUS, 0, 48);
    memcpy(pktStatus, &spi_buf.recv_buf_8[1], 5);
    pktStatus->RssiPkt = -(pktStatus->RssiPkt / 2);
    (pktStatus->SnrPkt < 128) ? (pktStatus->SnrPkt = pktStatus->SnrPkt / 4) : (pktStatus->SnrPkt = (pktStatus->SnrPkt - 256) / 4);
}

void SX1280SetFs(void)
{
    hspi_trans(RADIO_SET_FS, 0, 0);
}

void SX1280_Init()
{
    for (uint8_t i = 0; i < 101; i++)
        fhss_freq[i] = ((double)(2400000000 + i * 1000000)) / FREQ_STEP;

    hspi_init();

    SX1280Reset();

    SX1280SetStandby(STDBY_XOSC);

    SX1280SetRegulatorMode(USE_DCDC);

    SX1280SetAutoFS(true);

    SX1280SetStandby(STDBY_XOSC);
}
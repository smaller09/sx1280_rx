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

extern sx1280_buff_t spi_buf;

/*      frequency = 2400000000 + 1000000 * i;
        freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
        freq_h = (uint8_t)((freq >> 16) & 0xFF);
        freq_m = (uint8_t)((freq >> 8) & 0xFF);
        freq_l = (uint8_t)(freq & 0xFF);
*/
static const DRAM_ATTR uint8_t fhss_freq[101][3] = {
    {0xb8, 0x9d, 0x89},
    {0xb8, 0xb1, 0x3b},
    {0xb8, 0xc4, 0xec},
    {0xb8, 0xd8, 0x9d},
    {0xb8, 0xec, 0x4e},
    {0xb9, 0x0, 0x0},
    {0xb9, 0x13, 0xb1},
    {0xb9, 0x27, 0x62},
    {0xb9, 0x3b, 0x13},
    {0xb9, 0x4e, 0xc4},
    {0xb9, 0x62, 0x76},
    {0xb9, 0x76, 0x27},
    {0xb9, 0x89, 0xd8},
    {0xb9, 0x9d, 0x89},
    {0xb9, 0xb1, 0x3b},
    {0xb9, 0xc4, 0xec},
    {0xb9, 0xd8, 0x9d},
    {0xb9, 0xec, 0x4e},
    {0xba, 0x0, 0x0},
    {0xba, 0x13, 0xb1},
    {0xba, 0x27, 0x62},
    {0xba, 0x3b, 0x13},
    {0xba, 0x4e, 0xc4},
    {0xba, 0x62, 0x76},
    {0xba, 0x76, 0x27},
    {0xba, 0x89, 0xd8},
    {0xba, 0x9d, 0x89},
    {0xba, 0xb1, 0x3b},
    {0xba, 0xc4, 0xec},
    {0xba, 0xd8, 0x9d},
    {0xba, 0xec, 0x4e},
    {0xbb, 0x0, 0x0},
    {0xbb, 0x13, 0xb1},
    {0xbb, 0x27, 0x62},
    {0xbb, 0x3b, 0x13},
    {0xbb, 0x4e, 0xc4},
    {0xbb, 0x62, 0x76},
    {0xbb, 0x76, 0x27},
    {0xbb, 0x89, 0xd8},
    {0xbb, 0x9d, 0x89},
    {0xbb, 0xb1, 0x3b},
    {0xbb, 0xc4, 0xec},
    {0xbb, 0xd8, 0x9d},
    {0xbb, 0xec, 0x4e},
    {0xbc, 0x0, 0x0},
    {0xbc, 0x13, 0xb1},
    {0xbc, 0x27, 0x62},
    {0xbc, 0x3b, 0x13},
    {0xbc, 0x4e, 0xc4},
    {0xbc, 0x62, 0x76},
    {0xbc, 0x76, 0x27},
    {0xbc, 0x89, 0xd8},
    {0xbc, 0x9d, 0x89},
    {0xbc, 0xb1, 0x3b},
    {0xbc, 0xc4, 0xec},
    {0xbc, 0xd8, 0x9d},
    {0xbc, 0xec, 0x4e},
    {0xbd, 0x0, 0x0},
    {0xbd, 0x13, 0xb1},
    {0xbd, 0x27, 0x62},
    {0xbd, 0x3b, 0x13},
    {0xbd, 0x4e, 0xc4},
    {0xbd, 0x62, 0x76},
    {0xbd, 0x76, 0x27},
    {0xbd, 0x89, 0xd8},
    {0xbd, 0x9d, 0x89},
    {0xbd, 0xb1, 0x3b},
    {0xbd, 0xc4, 0xec},
    {0xbd, 0xd8, 0x9d},
    {0xbd, 0xec, 0x4e},
    {0xbe, 0x0, 0x0},
    {0xbe, 0x13, 0xb1},
    {0xbe, 0x27, 0x62},
    {0xbe, 0x3b, 0x13},
    {0xbe, 0x4e, 0xc4},
    {0xbe, 0x62, 0x76},
    {0xbe, 0x76, 0x27},
    {0xbe, 0x89, 0xd8},
    {0xbe, 0x9d, 0x89},
    {0xbe, 0xb1, 0x3b},
    {0xbe, 0xc4, 0xec},
    {0xbe, 0xd8, 0x9d},
    {0xbe, 0xec, 0x4e},
    {0xbf, 0x0, 0x0},
    {0xbf, 0x13, 0xb1},
    {0xbf, 0x27, 0x62},
    {0xbf, 0x3b, 0x13},
    {0xbf, 0x4e, 0xc4},
    {0xbf, 0x62, 0x76},
    {0xbf, 0x76, 0x27},
    {0xbf, 0x89, 0xd8},
    {0xbf, 0x9d, 0x89},
    {0xbf, 0xb1, 0x3b},
    {0xbf, 0xc4, 0xec},
    {0xbf, 0xd8, 0x9d},
    {0xbf, 0xec, 0x4e},
    {0xc0, 0x0, 0x0},
    {0xc0, 0x13, 0xb1},
    {0xc0, 0x27, 0x62},
    {0xc0, 0x3b, 0x13},
    {0xc0, 0x4e, 0xc4},
};

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

void SX1280GetPacketStatus(PacketStatus_t *pktStatus)
{
    hspi_trans(RADIO_GET_PACKETSTATUS, 0, 48);

    pktStatus->packetType = PACKET_TYPE_LORA;

    pktStatus->Params.LoRa.RssiPkt = -spi_buf.recv_buf_8[1] / 2;
    (spi_buf.recv_buf_8[2] < 128) ? (pktStatus->Params.LoRa.SnrPkt = spi_buf.recv_buf_8[2] / 4) : (pktStatus->Params.LoRa.SnrPkt = ((spi_buf.recv_buf_8[2] - 256) / 4));

    pktStatus->Params.LoRa.ErrorStatus.SyncError = (spi_buf.recv_buf_8[3] >> 6) & 0x01;
    pktStatus->Params.LoRa.ErrorStatus.LengthError = (spi_buf.recv_buf_8[3] >> 5) & 0x01;
    pktStatus->Params.LoRa.ErrorStatus.CrcError = (spi_buf.recv_buf_8[3] >> 4) & 0x01;
    pktStatus->Params.LoRa.ErrorStatus.AbortError = (spi_buf.recv_buf_8[3] >> 3) & 0x01;
    pktStatus->Params.LoRa.ErrorStatus.HeaderReceived = (spi_buf.recv_buf_8[3] >> 2) & 0x01;
    pktStatus->Params.LoRa.ErrorStatus.PacketReceived = (spi_buf.recv_buf_8[3] >> 1) & 0x01;
    pktStatus->Params.LoRa.ErrorStatus.PacketControlerBusy = spi_buf.recv_buf_8[3] & 0x01;

    pktStatus->Params.LoRa.TxRxStatus.RxNoAck = (spi_buf.recv_buf_8[4] >> 5) & 0x01;
    pktStatus->Params.LoRa.TxRxStatus.PacketSent = spi_buf.recv_buf_8[4] & 0x01;

    pktStatus->Params.LoRa.SyncAddrStatus = spi_buf.recv_buf_8[5] & 0x07;
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
    spi_buf.send_buf_8[0] = fhss_freq[channel][0];
    spi_buf.send_buf_8[1] = fhss_freq[channel][1];
    spi_buf.send_buf_8[2] = fhss_freq[channel][2];
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

void SX1280SetLoraSyncWord(uint8_t sync_h, uint8_t sync_l)
{
    spi_buf.send_buf_8[0] = (uint8_t)(REG_LORASYNCWORD >> 8);
    spi_buf.send_buf_8[1] = (uint8_t)(REG_LORASYNCWORD & 0xff);
    spi_buf.send_buf_8[2] = sync_h;
    spi_buf.send_buf_8[3] = sync_l;
    //    spi_buf.send_buf_16[1]=Syncword;
    hspi_trans(RADIO_WRITE_REGISTER, 32, 0);
}

void SX1280SetLoraMagicNum(uint8_t MagicNum)
{
    spi_buf.send_buf_8[0] = (uint8_t)(REG_LORAMAGICNUM >> 8);
    spi_buf.send_buf_8[1] = (uint8_t)(REG_LORAMAGICNUM & 0xff);
    spi_buf.send_buf_8[2] = MagicNum;
    hspi_trans(RADIO_WRITE_REGISTER, 24, 0);
}

/*
void SX1280Calibrate( CalibrationParams_t calibParam )
{
    uint8_t cal = ( calibParam.ADCBulkPEnable << 5 ) |
                  ( calibParam.ADCBulkNEnable << 4 ) |
                  ( calibParam.ADCPulseEnable << 3 ) |
                  ( calibParam.PLLEnable << 2 ) |
                  ( calibParam.RC13MEnable << 1 ) |
                  ( calibParam.RC64KEnable );

    SX1280HalWriteCommand( RADIO_CALIBRATE, &cal, 1 );
}

double SX1280GetFrequencyError( )
{
    uint8_t efeRaw[3] = {0};
    uint32_t efe = 0;
    double efeHz = 0.0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            efeRaw[0] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
            efeRaw[1] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
            efeRaw[2] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
            efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
            efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

            efeHz = 1.55 * ( double )SX1280complement2( efe, 20 ) / ( 1600.0 / ( double )SX1280GetLoRaBandwidth( ) * 1000.0 );
            break;

        case PACKET_TYPE_NONE:
        case PACKET_TYPE_BLE:
        case PACKET_TYPE_FLRC:
        case PACKET_TYPE_GFSK:
            break;
    }

    return efeHz;
}
*/

/*
uint16_t SX1280GetFirmwareVersion(void)
{
    spi_buf.send_buf_8[0] = (uint8_t)(((uint16_t)REG_LR_FIRMWARE_VERSION_MSB >> 8) & 0x00FF);
    spi_buf.send_buf_8[1] = (uint8_t)((uint16_t)REG_LR_FIRMWARE_VERSION_MSB & 0x00FF);

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
*/

void SX1280_Init()
{

    hspi_init();

    SX1280Reset();

    SX1280SetStandby(STDBY_XOSC);

    SX1280SetRegulatorMode(USE_DCDC);

    SX1280SetAutoFS(true);

    SX1280SetStandby(STDBY_XOSC);
}
#include "sx1280.h"
#include "sx1280_rx.h"
#include "hspi.h"

// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits)

extern sx1280_buff_t spi_buf;

const uint8_t txBaseAddress = 0x00, rxBaseAddress = 0x80;

uint8_t SX1280GetPayload(uint8_t size)
{
    hspi_trans(RADIO_GET_RXBUFFERSTATUS, 0, 24);

    if (spi_buf.recv_buf_8[1] != size)
        return 1;
    spi_buf.send_buf_8[0] = txBaseAddress;
    hspi_trans(RADIO_READ_BUFFER, 8, (size + 1) * 8);

    return 0;
}
// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);
void SX1280SetRx(TickTime_t timeout)
{

    spi_buf.send_buf_8[0] = timeout.Step;
    spi_buf.send_buf_8[1] = (uint8_t)((timeout.NbSteps >> 8) & 0x00FF);
    spi_buf.send_buf_8[2] = (uint8_t)(timeout.NbSteps & 0x00FF);

    SX1280ClearIrqStatus(IRQ_RADIO_ALL);

    hspi_trans(RADIO_SET_RX, 24, 0);
}

void SX1280ClearIrqStatus(uint16_t irq)
{

    spi_buf.send_buf_8[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
    spi_buf.send_buf_8[1] = (uint8_t)((uint16_t)irq & 0x00FF);

    hspi_trans(RADIO_CLR_IRQSTATUS, 16, 0);
}

// hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);
void SX1280SetTx(TickTime_t timeout)
{

    spi_buf.send_buf_8[0] = timeout.Step;
    spi_buf.send_buf_8[1] = (uint8_t)((timeout.NbSteps >> 8) & 0x00FF);
    spi_buf.send_buf_8[2] = (uint8_t)(timeout.NbSteps & 0x00FF);

    SX1280ClearIrqStatus(IRQ_RADIO_ALL);

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
    spi_buf.send_buf_8[5] = NULL;
    spi_buf.send_buf_8[6] = NULL;

    hspi_trans(RADIO_SET_PACKETPARAMS, 48, 0);
}

void SX1280SetBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{

    spi_buf.send_buf_8[0] = txBaseAddress;
    spi_buf.send_buf_8[1] = rxBaseAddress;
    hspi_trans(RADIO_SET_BUFFERBASEADDRESS, 16, 0);
}

void SX1280SetTxParams(int8_t power, RadioRampTimes_t rampTime)
{

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    spi_buf.send_buf_8[0] = power + 18;
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
}

RadioStatus_t SX1280GetStatus(void)
{
    uint8_t stat = 0;
    RadioStatus_t status;

    hspi_trans(RADIO_GET_STATUS, 0, 8);
    status.Value = spi_buf.recv_buf_8[0];
    return status;
}

uint16_t SX1280GetIrqStatus(void)
{
    // SX1280HalReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    hspi_trans(RADIO_GET_IRQSTATUS, 0, 24);
    return (spi_buf.recv_buf_8[1] << 8) | spi_buf.recv_buf_8[2];
}

void SX1280SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
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

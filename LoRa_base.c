#include "LoRa_base.h"

/* ── Register map ──────────────────────────────────────────────────────────── */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

/* ── Operating modes ───────────────────────────────────────────────────────── */
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

/* ── PA config ─────────────────────────────────────────────────────────────── */
#define PA_BOOST                 0x80

/* ── IRQ masks ─────────────────────────────────────────────────────────────── */
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH             255

/* ── Driver state (single definition) ─────────────────────────────────────── */
int _packetIndex      = 0;
int _implicitHeaderMode = 0;

/* ── Public API ────────────────────────────────────────────────────────────── */

int LoRa_begin(long frequency)
{
    /* Unmask all interrupts */
    LoRa_writeRegister(REG_IRQ_FLAGS_MASK, 0x00);
    /* Map DIO0 to RxDone */
    LoRa_writeRegister(REG_DIO_MAPPING_1, 0x00);

    LoRa_sleep();
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE);

    for (uint32_t k = 0; k < SystemCoreClock / 8; k++) { __NOP(); }

    LoRa_setFrequency(frequency);

    LoRa_writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    LoRa_writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    /* LNA boost */
    LoRa_writeRegister(REG_LNA, LoRa_readRegister(REG_LNA) | 0x03);

    /* Auto AGC */
    LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x04);

    LoRa_setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
    LoRa_setSignalBandwidth(50e3);
    LoRa_setPreambleLength(8);
    LoRa_setSpreadingFactor(7);

    LoRa_idle();

    return 1;
}

int LoRa_beginPacket(int implicitHeader)
{
    LoRa_idle();

    if (implicitHeader) {
        LoRa_implicitHeaderMode();
    } else {
        LoRa_explicitHeaderMode();
    }

    LoRa_writeRegister(REG_FIFO_ADDR_PTR,   0);
    LoRa_writeRegister(REG_PAYLOAD_LENGTH,  0);

    return 1;
}

int LoRa_endPacket(void)
{
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    while ((LoRa_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {}

    LoRa_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

    return 1;
}

size_t LoRa_write(uint8_t *buffer, size_t size)
{
    int currentLength = LoRa_readRegister(REG_PAYLOAD_LENGTH);

    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }

    for (size_t i = 0; i < size; i++) {
        LoRa_writeRegister(REG_FIFO, buffer[i]);
    }

    LoRa_writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

int LoRa_available(void)
{
    return (LoRa_readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa_read(void)
{
    if (!LoRa_available()) {
        return -1;
    }
    _packetIndex++;
    return LoRa_readRegister(REG_FIFO);
}

void LoRa_receive(int size)
{
    if (size > 0) {
        LoRa_implicitHeaderMode();
        LoRa_writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        LoRa_explicitHeaderMode();
    }
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa_idle(void)
{
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa_sleep(void)
{
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa_setTxPower(int level, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin) {
        if (level < 0)       { level = 0;  }
        else if (level > 14) { level = 14; }
        LoRa_writeRegister(REG_PA_CONFIG, 0x70 | level);
    } else {
        if (level < 2)       { level = 2;  }
        else if (level > 17) { level = 17; }
        LoRa_writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRa_setFrequency(long frequency)
{
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    LoRa_writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_writeRegister(REG_FRF_MID, (uint8_t)(frf >>  8));
    LoRa_writeRegister(REG_FRF_LSB, (uint8_t)(frf >>  0));
}

void LoRa_setSpreadingFactor(int sf)
{
    if (sf < 6)       { sf = 6;  }
    else if (sf > 12) { sf = 12; }

    if (sf == 6) {
        LoRa_writeRegister(REG_DETECTION_OPTIMIZE,  0xc5);
        LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        LoRa_writeRegister(REG_DETECTION_OPTIMIZE,  0xc3);
        LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    LoRa_writeRegister(REG_MODEM_CONFIG_2,
        (LoRa_readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void LoRa_setSignalBandwidth(long sbw)
{
    int bw;

    if      (sbw <= 7.8E3)  { bw = 0; }
    else if (sbw <= 10.4E3) { bw = 1; }
    else if (sbw <= 15.6E3) { bw = 2; }
    else if (sbw <= 20.8E3) { bw = 3; }
    else if (sbw <= 31.25E3){ bw = 4; }
    else if (sbw <= 41.7E3) { bw = 5; }
    else if (sbw <= 62.5E3) { bw = 6; }
    else if (sbw <= 125E3)  { bw = 7; }
    else if (sbw <= 250E3)  { bw = 8; }
    else                    { bw = 9; }

    LoRa_writeRegister(REG_MODEM_CONFIG_1,
        (LoRa_readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void LoRa_setCodingRate4(int denominator)
{
    if (denominator < 5)     { denominator = 5; }
    else if (denominator > 8){ denominator = 8; }

    int cr = denominator - 4;
    LoRa_writeRegister(REG_MODEM_CONFIG_1,
        (LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRa_setPreambleLength(long length)
{
    LoRa_writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    LoRa_writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRa_enableCrc(void)
{
    LoRa_writeRegister(REG_MODEM_CONFIG_2,
        LoRa_readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRa_disableCrc(void)
{
    LoRa_writeRegister(REG_MODEM_CONFIG_2,
        LoRa_readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRa_explicitHeaderMode(void)
{
    _implicitHeaderMode = 0;
    LoRa_writeRegister(REG_MODEM_CONFIG_1,
        LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRa_implicitHeaderMode(void)
{
    _implicitHeaderMode = 1;
    LoRa_writeRegister(REG_MODEM_CONFIG_1,
        LoRa_readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

uint8_t LoRa_handleDio0Rise(void)
{
    uint8_t packetLength = 0;
    int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);

    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        _packetIndex = 0;
        LoRa_writeRegister(REG_FIFO_ADDR_PTR,
            LoRa_readRegister(REG_FIFO_RX_CURRENT_ADDR));
        packetLength = _implicitHeaderMode
            ? LoRa_readRegister(REG_PAYLOAD_LENGTH)
            : LoRa_readRegister(REG_RX_NB_BYTES);
    }

    return packetLength;
}

void LoRa_clearDIOrise(void)
{
    int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);
    LoRa_writeRegister(REG_IRQ_FLAGS, irqFlags);
}

void LoRa_writeRegister(uint8_t address, uint8_t value)
{
    LoRa_singleTransfer(address | 0x80, value);
}

uint8_t LoRa_readRegister(uint8_t address)
{
    return LoRa_singleTransfer(address & 0x7f, 0x00);
}
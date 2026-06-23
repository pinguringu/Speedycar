// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#include "stdint.h"
#include "stddef.h"
#include "cybsp.h"
#include "cy_utils.h"

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

int     LoRa_begin(long frequency);

int     LoRa_beginPacket(int implicitHeader);
int     LoRa_endPacket(void);

size_t  LoRa_write(uint8_t *buffer, size_t size);

int     LoRa_available(void);
int     LoRa_read(void);

void    LoRa_receive(int size);
void    LoRa_idle(void);
void    LoRa_sleep(void);

void    LoRa_setTxPower(int level, int outputPin);
void    LoRa_setFrequency(long frequency);
void    LoRa_setSpreadingFactor(int sf);
void    LoRa_setSignalBandwidth(long sbw);
void    LoRa_setCodingRate4(int denominator);
void    LoRa_setPreambleLength(long length);
void    LoRa_enableCrc(void);
void    LoRa_disableCrc(void);

void    LoRa_explicitHeaderMode(void);
void    LoRa_implicitHeaderMode(void);

uint8_t LoRa_handleDio0Rise(void);
void    LoRa_clearDIOrise(void);

uint8_t LoRa_readRegister(uint8_t address);
void    LoRa_writeRegister(uint8_t address, uint8_t value);

/* SPI transfer — defined in main.c (platform layer) */
uint8_t LoRa_singleTransfer(uint8_t address, uint8_t value);

/* Driver-internal state — single definition in LoRa_base.c */
extern int _packetIndex;
extern int _implicitHeaderMode;

#endif /* LORA_H */
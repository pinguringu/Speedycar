// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "stdint.h"
#include "stddef.h"
#include "cybsp.h"
#include "cy_utils.h"


#ifndef LORA_H
#define LORA_H

#define LORA_DEFAULT_SS_PIN    PA4
#define LORA_DEFAULT_RESET_PIN PC13
#define LORA_DEFAULT_DIO0_PIN  PA1

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1



  int LoRa_begin(long frequency);
  void LoRa_end();

  int LoRa_beginPacket(int implicitHeader);
  int LoRa_endPacket();

  int LoRa_parsePacket(int size);
  int LoRa_packetRssi();
  float LoRa_packetSnr();

  // from Print

  size_t LoRa_write(uint8_t *buffer, size_t size);

  // from Stream
  int LoRa_available();
  int LoRa_read();
  int LoRa_peek();
  void LoRa_flush();

  void LoRa_onReceive();

  void LoRa_receive(int size);
  void LoRa_idle();
  void LoRa_sleep();

  void LoRa_setTxPower(int level, int outputPin);
  void LoRa_setFrequency(long frequency);
  void LoRa_setSpreadingFactor(int sf);
  void LoRa_setSignalBandwidth(long sbw);
  void LoRa_setCodingRate4(int denominator);
  void LoRa_setPreambleLength(long length);
  void LoRa_setSyncWord(int sw);
  void LoRa_enableCrc();
  void LoRa_disableCrc();





  void LoRa_setPins(int reset, int dio0);



  void LoRa_explicitHeaderMode();
  void LoRa_implicitHeaderMode();

  uint8_t LoRa_handleDio0Rise();

  uint8_t LoRa_readRegister(uint8_t address);
  void LoRa_writeRegister(uint8_t address, uint8_t value);


  void LoRa_onDio0Rise();

  static int _packetIndex;
  static int _implicitHeaderMode;

  void LoRa_receiveflow(int);



#endif

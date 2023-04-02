/**
  ******************************************************************************
  * @file    ST25DVSensor.h
  * @author  MCD Application Team
  * @brief   Header file of NFC ST25SV sensor module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#ifndef _ST25DVSENSOR_H_
#define _ST25DVSENSOR_H_
#include "Arduino.h"
#include <Wire.h>
#include <Stream.h>
#include "ST25DV_IO/st25dv_io.h"
#include "libNDEF/NDEF_class.h"

#if defined(ARDUINO_SAM_DUE)
  #define WIRE Wire1
#else
  #define WIRE Wire
#endif

class ST25DV {
  public:
    ST25DV(int32_t gpo, int32_t lpd, TwoWire *i2c, Stream *serial = NULL) : st25dv_io(gpo, lpd, i2c, serial), ndef(&st25dv_io) {}

    int begin();
    int writeURI(String protocol, String uri, String info);
    int readURI(String *s);
    NDEF *getNDEF();

  protected:
    NFCTAG_StatusTypeDef ST25DV_Init(void);
    void ST25DV_GPO_Init(void);
    void ST25DV_GPO_DeInit(void);
    uint8_t ST25DV_GPO_ReadPin(void);
    void ST25DV_LPD_Init(void);
    void ST25DV_LPD_DeInit(void);
    uint8_t ST25DV_LPD_ReadPin(void);
    void ST25DV_LPD_WritePin(uint8_t LpdPinState);
    void ST25DV_I2C_Init(void);
    void ST25DV_SelectI2cSpeed(uint8_t i2cspeedchoice);

    ST25DV_IO st25dv_io;
    NDEF ndef;
    uint8_t NfctagInitialized = 0;
};

#endif

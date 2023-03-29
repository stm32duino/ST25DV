
/**
  ******************************************************************************
  * @file    ST25DVSensor.c
  * @author  MCD Application Team
  * @brief   Source file of NFC ST25SV sensor module.
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

#include "st25dv_io.h"


ST25DV_IO::ST25DV_IO(int32_t gpo, int32_t lpd, TwoWire *i2c, Stream *serial)
{
  _gpo = gpo;
  _lpd = lpd;
  _pwire = i2c;
  _serial = serial;
}

/******************************** LINK EEPROM COMPONENT *****************************/

/**
  * @brief  Write data, at specific address, through i2c to the ST25DV
  * @param  pData: pointer to the data to write
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to write
  * @param  Size : Size in bytes of the value to be written
  * @retval NFCTAG enum status
  */

NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_IO_MemWrite(const uint8_t *const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size)
{
  NFCTAG_StatusTypeDef pollstatus;
  byte ret;
  uint32_t tickstart;
  uint8_t Addr = DevAddr >> 1;
  char tmp[4];

  if (_pwire == NULL) {
    return NFCTAG_ERROR;
  }

  if (_serial != NULL) {
    //  _serial->println(SP);
    _serial->print("  w ");
    sprintf(tmp, "%02X", Addr);
    _serial->print(tmp);
    _serial->print("@");
    sprintf(tmp, "%02X", TarAddr >> 8);
    _serial->print(tmp);
    sprintf(tmp, "%02X", TarAddr & 0xFF);
    _serial->print(tmp);
    _serial->print(":");
    _serial->println(Size);
    _serial->print("  ");
    for (uint16_t d = 0; d < Size; d++) {
      sprintf(tmp, "%02X", pData[d]);
      _serial->print(tmp);
    }
    _serial->println("");
  }

  _pwire->beginTransmission(Addr); // transmit to device
  _pwire->write(TarAddr >> 8);   // send memory address MSB
  _pwire->write(TarAddr & 0xFF); // send memory address LSB
  _pwire->write(pData, Size);        // sends  bytes
  ret = _pwire->endTransmission(true);    // stop transmitting
  if (_serial != NULL) {
    _serial->print("  =");
    _serial->println(ret);
  }

  if (ret == 0) {
    /* Poll until EEPROM is available */
    tickstart = millis();
    /* Wait until ST25DV is ready or timeout occurs */
    do {
      pollstatus = ST25DV_IO_IsDeviceReady(DevAddr, 1);
    } while (((millis() - tickstart) < ST25DV_I2C_TIMEOUT) && (pollstatus != NFCTAG_OK));

    if (pollstatus != NFCTAG_OK) {
      return NFCTAG_TIMEOUT;
    }
  }
#if defined(ARDUINO_ARCH_ARC) || defined(ARDUINO_ARCH_ARC32)
  // Arduino 101 i2c seems buggy after an address NACK: restart the i2c
  else if (ret == 2) {
    if (_serial != NULL) {
      _serial->print("  -\n");
    }
    _pwire->begin();
  }
#endif
  return NFCTAG_ConvertStatus(ret);
}

/**
  * @brief  Reads data at a specific address from the NFCTAG.
  * @param  pData: pointer to store read data
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to read
  * @param  Size : Size in bytes of the value to be read
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_IO_MemRead(uint8_t *const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size)
{
  int i = 0;
  uint8_t ret = 4;
  uint8_t Addr = DevAddr >> 1;
  char tmp[4];

  if (_pwire == NULL) {
    return NFCTAG_ERROR;
  }

  if (_serial != NULL) {
    //  _serial->println(SP);
    _serial->print("  r ");
    sprintf(tmp, "%02X", Addr);
    _serial->print(tmp);
    _serial->print("@");
    sprintf(tmp, "%02X", TarAddr >> 8);
    _serial->print(tmp);
    sprintf(tmp, "%02X", TarAddr & 0xFF);
    _serial->print(tmp);
    _serial->print(":");
    _serial->println(Size);
  }
  _pwire->beginTransmission(Addr);    // Get the slave's attention, tell it we're sending a command byte
  _pwire->write(TarAddr >> 8);           //  The command byte, sets pointer to register with address of 0x32
  _pwire->write(TarAddr & 0xFF);         //  The command byte, sets pointer to register with address of 0x32
  ret = _pwire->endTransmission(true);
  // Address is not OK
  if (ret != 0) {
    return NFCTAG_ConvertStatus(ret);
  }

  // be carefully with the (int)Size, the size parameter us no more on 16 bits but only 15 bits (the cast is required for arduino UNO)
  _pwire->requestFrom((int)Addr, (int)Size); // Tell slave we need to read 1byte from the current register


  while (_pwire->available()) {
    pData[i++] = _pwire->read();      // read that byte into 'slaveByte2' variable
  }
  if (_serial != NULL) {
    _serial->print("  ");
    for (int d = 0; d < i; d++) {
      sprintf(tmp, "%02X", pData[d]);
      _serial->print(tmp);
    }
    _serial->println("");
  }

  /*
  It doesn't seem like Arduino wants you to call `endTransmission`
  after `requestFrom`. On the ESP32 the ret value is 8 because it
  is an `endTransmission` without a `startTransmission` which is
  considered an error by the library.
  */
  // ret = _pwire->endTransmission();
  // if (_serial != NULL) {
  //   //  _serial->println(pData[0]);
  //   _serial->print("  =");
  //   _serial->println(ret);
  //   //  _serial->print("  ");
  //   //  _serial->println(pData[0]);
  //   //  _serial->println((uint32_t)pData);
  // }
  return NFCTAG_ConvertStatus(ret);
}

NFCTAG_StatusTypeDef ST25DV_IO::NFCTAG_ConvertStatus(uint8_t ret)
{
  if (ret == 0) {
    return NFCTAG_OK;
  } else if ((ret == 2) || (ret == 3)) {
    return NFCTAG_NACK;
  } else {
    return NFCTAG_ERROR;
  }
}


/**
  * @brief  Reads data at current address from the NFCTAG.
  * @param  pData: pointer to store read data
  * @param  DevAddr : Target device address
  * @param  Size : Size in bytes of the value to be read
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_IO_Read(uint8_t *const pData, const uint8_t DevAddr, const uint16_t Size)
{
  int i = 0;
  uint8_t Addr = DevAddr >> 1;
  char tmp[4];

  if (_pwire == NULL) {
    return NFCTAG_ERROR;
  }
  if (_serial != NULL) {
    _serial->print("  r");
    _serial->print(":");
    _serial->println(Size);
  }
  // be carefully with the (int)Size, the size parameter us no more on 16 bits but only 15 bits (the cast is required for arduino UNO)
  byte ret = _pwire->requestFrom((int)Addr, (int)Size); // Tell slave we need to read 1byte from the current register
  while (_pwire->available()) {
    pData[i++] = _pwire->read();      // read that byte into 'slaveByte2' variable
  }

  if (_serial != NULL) {
    _serial->print("  ");
    for (int d = 0; d < i; d++) {
      sprintf(tmp, "%02X", pData[d]);
      _serial->print(tmp);
    }
    _serial->println("");
  }
  /*
  It doesn't seem like Arduino wants you to call `endTransmission`
  after `requestFrom`. On the ESP32 the ret value is 8 because it
  is an `endTransmission` without a `startTransmission` which is
  considered an error by the library.
  */
  // ret = _pwire->endTransmission();

  // if (_serial != NULL) {
  //   //  _serial->println(pData[0]);
  //   _serial->print("  =");
  //   _serial->println(ret);
  // }
  return NFCTAG_ConvertStatus(ret);
}


/**
* @brief  Checks if target device is ready for communication
* @note   This function is used with Memory devices
* @param  DevAddr : Target device address
* @retval NFCTAG enum status
*/
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_IO_IsDeviceReady(const uint8_t DevAddr, const uint32_t Trials)
{
  int ret = 4;
  uint32_t count = 0;
  if (_pwire == NULL) {
    return NFCTAG_ERROR;
  }
  if (_serial != NULL) {
    _serial->println("  ?");
  }

  while ((count++ < Trials) && ret) {
    _pwire->beginTransmission(DevAddr >> 1);
    ret = _pwire->endTransmission();
    if (_serial != NULL) {
      _serial->print("  =");
      _serial->println(ret);
    }
  }
  return NFCTAG_ConvertStatus(ret);
}

/**
  * @brief  ST25DV nfctag Initialization.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_Init(void)
{
  /* Configure the low level interface */
  return ST25DV_IO_Init();
}

/**
  * @brief  Reads the ST25DV ID.
  * @param  pICRef Pointeron a uint8_t used to return the ST25DV ID.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadID(uint8_t *const pICRef)
{
  /* Read ICRef on device */
  return ST25DV_i2c_ReadRegister(pICRef, ST25DV_ICREF_REG, 1);
}

/**
  * @brief  Reads N bytes of Data, starting from the specified I2C address.
  * @param  pData   Pointer used to return the read data.
  * @param  TarAddr I2C data memory address to read.
  * @param  NbByte  Number of bytes to be read.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadData(uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte)
{
  /* Read Data in user memory */
  return ST25DV_IO_MemRead(pData, ST25DV_ADDR_DATA_I2C, TarAddr, NbByte);
}

/**
  * @brief  Writes N bytes of Data starting from the specified I2C Address.
  * @param  pData   Pointer on the data to be written.
  * @param  TarAddr I2C data memory address to be written.
  * @param  NbByte  Number of bytes to be written.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteData(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte)
{
  NFCTAG_StatusTypeDef ret;
  uint16_t split_data_nb;
  const uint8_t *pdata_index = (const uint8_t *)pData;
  uint16_t bytes_to_write = NbByte;
  uint16_t mem_addr = TarAddr;

  /* ST25DV can write a maximum of 256 bytes in EEPROM per i2c communication */
  do {
    /* Split write if data to write is superior of max write bytes for ST25DV */
    if (bytes_to_write > ST25DV_MAX_WRITE_BYTE) {
      /* DataSize higher than max page write, copy data by page */
      split_data_nb = (uint16_t)ST25DV_MAX_WRITE_BYTE;
    } else {
      /* DataSize lower or equal to max page write, copy only last bytes */
      split_data_nb = bytes_to_write;
    }
    /* Write split_data_nb bytes in memory */
    ret = ST25DV_IO_MemWrite(pdata_index, ST25DV_ADDR_DATA_I2C, mem_addr, split_data_nb);

    /* update index, dest address, size for next write */
    pdata_index += split_data_nb;
    mem_addr += split_data_nb;
    bytes_to_write -= split_data_nb;
  } while ((bytes_to_write > 0) && (ret == NFCTAG_OK));

  return ret;
}

/**
  * @brief  Reads N bytes from Registers, starting at the specified I2C address.
  * @param  pData   Pointer used to return the read data.
  * @param  TarAddr I2C memory address to be read.
  * @param  NbByte  Number of bytes to be read.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadRegister(uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte)
{
  /* Read Data in system memory */
  return ST25DV_IO_MemRead(pData, ST25DV_ADDR_SYST_I2C, TarAddr, NbByte);
}

/**
  * @brief    Writes N bytes to the specified register.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    pData   Pointer on the data to be written.
  * @param    TarAddr I2C register address to written.
  * @param    NbByte  Number of bytes to be written.
  * @return   NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteRegister(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte)
{
  NFCTAG_StatusTypeDef ret;
  uint8_t split_data_nb;
  uint16_t bytes_to_write = NbByte;
  uint16_t mem_addr = TarAddr;
  const uint8_t *pdata_index = (const uint8_t *)pData;

  /* ST25DV can write a maximum of 256 bytes in EEPROM per i2c communication */
  do {
    /* Split write if data to write is superior of max write bytes for ST25DV */
    if (bytes_to_write > ST25DV_MAX_WRITE_BYTE) {
      /* DataSize higher than max page write, copy data by page */
      split_data_nb = (uint8_t)ST25DV_MAX_WRITE_BYTE;
    } else {
      /* DataSize lower or equal to max page write, copy only last bytes */
      split_data_nb = bytes_to_write;
    }
    /* Write split_data_nb bytes in register */
    ret = ST25DV_IO_MemWrite(pdata_index, ST25DV_ADDR_SYST_I2C, mem_addr, split_data_nb);

    /* update index, dest address, size for next write */
    pdata_index += split_data_nb;
    mem_addr += split_data_nb;
    bytes_to_write -= split_data_nb;
  } while ((bytes_to_write > 0) && (ret == NFCTAG_OK));

  return ret;
}

/**
  * @brief  Reads the ST25DV IC Revision.
  * @param  pICRev Pointer on the uint8_t used to return the ST25DV IC Revision number.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadICRev(uint8_t *const pICRev)
{
  /* Read ICRev on device */
  return ST25DV_i2c_ReadRegister(pICRev, ST25DV_ICREV_REG, 1);
}

/** TODO adjust comment
  * @brief  Reads the ST25DV GPO configuration.
  * @param  pGPOStatus  Pointer on a uint16_t used to return the current GPO consiguration, as:
  *                     - RFUSERSTATE = 0x02
  *                     - RFBUSY = 0x04
  *                     - RFINTERRUPT = 0x08
  *                     - FIELDFALLING = 0x08
  *                     - FIELDRISING = 0x10
  *                     - RFPUTMSG = 0x20
  *                     - RFGETMSG = 0x40
  *                     - RFWRITE = 0x80
  *
  * @retval   NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetGPOStatus(uint16_t *const pGPOStatus)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;
  /* Read value of GPO register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DVXXKC_GPO2_REG, 1);
  if (status == NFCTAG_OK) {
    // Extract GPO configuration
    *pGPOStatus = ((uint16_t)0x0003 & (uint16_t)reg_value) << 8;
    // Read value of GPO register
    status = ST25DV_i2c_ReadRegister(&reg_value, ST25DVXXKC_GPO1_REG, 1);
    if (status == NFCTAG_OK) {
      // Extract GPO configuration
      *pGPOStatus |= ((uint16_t)0x00FF & (uint16_t)reg_value);
    }
  }
  return status;
}

/**
  * @brief    Configures the ST25DV GPO.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    ITConf Provides the GPO configuration to apply:
  *           - RFUSERSTATE = 0x01
  *           - RFBUSY = 0x02
  *           - RFINTERRUPT = 0x04
  *           - FIELDFALLING = 0x08
  *           - FIELDRISING = 0x10
  *           - RFPUTMSG = 0x20
  *           - RFGETMSG = 0x40
  *           - RFWRITE = 0x80
  *
  * @retval   NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ConfigureGPO(const uint16_t ITConf)
{
  /* Write GPO configuration to register */
  return ST25DV_i2c_WriteRegister((uint8_t *)&ITConf, ST25DVXXKC_GPO1_REG, 1);
}


/**
  * @brief  Reads the ST25DV ITtime duration for the GPO pulses.
  * @param  pITtime Pointer used to return the coefficient for the GPO Pulse duration (Pulse duration = 302,06 us - ITtime * 512 / fc).
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadITPulse(ST25DV_PULSE_DURATION *const pITtime)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read ITtime register value */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DVXXKC_GPO2_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract delay coefficient value */
  *pITtime = (ST25DV_PULSE_DURATION)reg_value;

  return NFCTAG_OK;
}

/**
  * @brief    Configures the ST25DV ITtime duration for the GPO pulse.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    ITtime Coefficient for the Pulse duration to be written (Pulse duration = 302,06 us - ITtime * 512 / fc)
  * @retval   NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteITPulse(const ST25DV_PULSE_DURATION ITtime)
{
  uint8_t reg_value;

  /* prepare data to write */
  reg_value = (uint8_t)ITtime;

  /* Write value for ITtime register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DVXXKC_GPO2_REG, 1);
}

/**
  * @brief  Reads N bytes of Data, starting at current address.
  * @param  pData   Pointer used to return the read data.
  * @param  NbByte  Number of bytes to be read.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadDataCurrentAddr(uint8_t *const pData, const uint16_t NbByte)
{
  /* Read Data in user memory */
  return ST25DV_IO_Read(pData, ST25DV_ADDR_DATA_I2C, NbByte);
}

/**
  * @brief  Reads the ST25DV UID.
  * @param  pUid Pointer used to return the ST25DV UID value.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadUID(ST25DV_UID *const pUid)
{
  uint8_t reg_value[8];
  uint8_t i;
  NFCTAG_StatusTypeDef status;

  /* Read value of UID registers */
  status = ST25DV_i2c_ReadRegister(reg_value, ST25DV_UID_REG, 8);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Store information in 2 WORD */
  pUid->MsbUid = 0;

  for (i = 0; i < 4; i++) {
    pUid->MsbUid = (pUid->MsbUid << 8) | reg_value[7 - i];
  }

  pUid->LsbUid = 0;

  for (i = 0; i < 4; i++) {
    pUid->LsbUid = (pUid->LsbUid << 8) | reg_value[3 - i];
  }

  return NFCTAG_OK;
}

/**
  * @brief  Reads the ST25DV DSFID.
  * @param  pDsfid Pointer used to return the ST25DV DSFID value.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadDSFID(uint8_t *const pDsfid)
{
  /* Read DSFID register */
  return ST25DV_i2c_ReadRegister(pDsfid, ST25DV_DSFID_REG, 1);
}

/**
  * @brief  Reads the ST25DV DSFID RF Lock state.
  * @param  pLockDsfid Pointer on a ST25DV_LOCK_STATUS used to return the DSFID lock state.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadDsfidRFProtection(ST25DV_LOCK_STATUS *const pLockDsfid)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_LOCKDSFID_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Lock Status */
  if (reg_value == 0) {
    *pLockDsfid = ST25DV_UNLOCKED;
  } else {
    *pLockDsfid = ST25DV_LOCKED;
  }
  return NFCTAG_OK;
}

/**
  * @brief  Reads the ST25DV AFI.
  * @param  pAfi Pointer used to return the ST25DV AFI value.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadAFI(uint8_t *const pAfi)
{
  /* Read AFI register */
  return ST25DV_i2c_ReadRegister(pAfi, ST25DV_AFI_REG, 1);
}

/**
  * @brief  Reads the AFI RF Lock state.
  * @param  pLockAfi Pointer on a ST25DV_LOCK_STATUS used to return the ASFID lock state.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadAfiRFProtection(ST25DV_LOCK_STATUS *const pLockAfi)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_LOCKAFI_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Lock Status */
  if (reg_value == 0) {
    *pLockAfi = ST25DV_UNLOCKED;
  } else {
    *pLockAfi = ST25DV_LOCKED;
  }
  return NFCTAG_OK;
}

/**
  * @brief  Reads the I2C Protected Area state.
  * @param  pProtZone Pointer on a ST25DV_I2C_PROT_ZONE structure used to return the Protected Area state.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadI2CProtectZone(ST25DV_I2C_PROT_ZONE *const pProtZone)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read value of I2c Protected Zone register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_I2CZSS_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Dispatch information to corresponding struct member */
  pProtZone->ProtectZone1 = (ST25DV_PROTECTION_CONF)((reg_value & ST25DV_I2CZSS_PZ1_MASK) >> ST25DV_I2CZSS_PZ1_SHIFT);
  pProtZone->ProtectZone2 = (ST25DV_PROTECTION_CONF)((reg_value & ST25DV_I2CZSS_PZ2_MASK) >> ST25DV_I2CZSS_PZ2_SHIFT);
  pProtZone->ProtectZone3 = (ST25DV_PROTECTION_CONF)((reg_value & ST25DV_I2CZSS_PZ3_MASK) >> ST25DV_I2CZSS_PZ3_SHIFT);
  pProtZone->ProtectZone4 = (ST25DV_PROTECTION_CONF)((reg_value & ST25DV_I2CZSS_PZ4_MASK) >> ST25DV_I2CZSS_PZ4_SHIFT);

  return NFCTAG_OK;
}

/**
  * @brief  Lock/Unlock the Cfg registers, to prevent any RF write access.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  LockCfg ST25DV_LOCK_STATUS value corresponding to the lock state to be written.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteLockCFG(const ST25DV_LOCK_STATUS LockCfg)
{
  uint8_t reg_value;

  /* Configure value to write on register */
  reg_value = (uint8_t)LockCfg;

  /* Write LOCKCFG register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_LOCKCFG_REG, 1);
}

/**
  * @brief  Presents I2C password, to authorize the I2C writes to protected areas.
  * @param  PassWord Password value on 32bits
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_PresentI2CPassword(const ST25DV_PASSWD PassWord)
{
  uint8_t ai2c_message[17] = {0};
  uint8_t i;

  /* Build I2C Message with Password + Validation code 0x09 + Password */
  ai2c_message[8] = 0x09;
  for (i = 0; i < 4; i++) {
    ai2c_message[i] = (PassWord.MsbPasswd >> ((3 - i) * 8)) & 0xFF;
    ai2c_message[i + 4] = (PassWord.LsbPasswd >> ((3 - i) * 8)) & 0xFF;
    ai2c_message[i + 9] = ai2c_message[i];
    ai2c_message[i + 13] = ai2c_message[i + 4];
  };

  /* Present password to ST25DV */
  return ST25DV_i2c_WriteRegister(ai2c_message, ST25DV_I2CPASSWD_REG, 17);
}

/**
  * @brief  Writes a new I2C password.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  PassWord New I2C PassWord value on 32bits.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteI2CPassword(const ST25DV_PASSWD PassWord)
{
  uint8_t ai2c_message[17] = {0};
  uint8_t i;

  /* Build I2C Message with Password + Validation code 0x07 + Password */
  ai2c_message[8] = 0x07;

  for (i = 0; i < 4; i++) {
    ai2c_message[i] = (PassWord.MsbPasswd >> ((3 - i) * 8)) & 0xFF;
    ai2c_message[i + 4] = (PassWord.LsbPasswd >> ((3 - i) * 8)) & 0xFF;
    ai2c_message[i + 9] = ai2c_message[i];
    ai2c_message[i + 13] = ai2c_message[i + 4];
  };

  /* Write new password in I2CPASSWD register */
  return ST25DV_i2c_WriteRegister(ai2c_message, ST25DV_I2CPASSWD_REG, 17);
}

/**
  * @brief  Reads the RF Zone Security Status (defining the allowed RF accesses).
  * @param  Zone        ST25DV_PROTECTION_ZONE value corresponding to the protected area.
  * @param  pRfprotZone Pointer on a ST25DV_RF_PROT_ZONE value corresponding to the area protection state.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadRFZxSS(const ST25DV_PROTECTION_ZONE Zone, ST25DV_RF_PROT_ZONE *const pRfprotZone)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;
  uint16_t sector_security_addr;

  /* Select Sector Security register address */
  switch (Zone) {
    case ST25DV_PROT_ZONE1:
      sector_security_addr = ST25DV_RFZ1SS_REG;
      break;
    case ST25DV_PROT_ZONE2:
      sector_security_addr = ST25DV_RFZ2SS_REG;
      break;
    case ST25DV_PROT_ZONE3:
      sector_security_addr = ST25DV_RFZ3SS_REG;
      break;
    case ST25DV_PROT_ZONE4:
      sector_security_addr = ST25DV_RFZ4SS_REG;
      break;

    default:
      return NFCTAG_ERROR;
  }

  /* Read actual value of Sector Security Status register */
  status = ST25DV_i2c_ReadRegister(&reg_value, sector_security_addr, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Sector Security Status configuration */
  pRfprotZone->PasswdCtrl = (ST25DV_PASSWD_PROT_STATUS)((reg_value & ST25DV_RFZSS_PWDCTRL_MASK) >> ST25DV_RFZSS_PWDCTRL_SHIFT);
  pRfprotZone->RWprotection = (ST25DV_PROTECTION_CONF)((reg_value & ST25DV_RFZSS_RWPROT_MASK) >> ST25DV_RFZSS_RWPROT_SHIFT);

  return NFCTAG_OK;
}

/**
  * @brief  Writes the RF Zone Security Status (defining the allowed RF accesses)
  * @details  Needs the I2C Password presentation to be effective.
  * @param  Zone        ST25DV_PROTECTION_ZONE value corresponding to the area on which to set the RF protection.
  * @param  RfProtZone  Pointer on a ST25DV_RF_PROT_ZONE value defininf the protection to be set on the area.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteRFZxSS(const ST25DV_PROTECTION_ZONE Zone, const ST25DV_RF_PROT_ZONE RfProtZone)
{
  uint8_t reg_value;
  uint16_t sector_security_addr;

  /* Select Sector Security register address */
  switch (Zone) {
    case ST25DV_PROT_ZONE1:
      sector_security_addr = ST25DV_RFZ1SS_REG;
      break;
    case ST25DV_PROT_ZONE2:
      sector_security_addr = ST25DV_RFZ2SS_REG;
      break;
    case ST25DV_PROT_ZONE3:
      sector_security_addr = ST25DV_RFZ3SS_REG;
      break;
    case ST25DV_PROT_ZONE4:
      sector_security_addr = ST25DV_RFZ4SS_REG;
      break;

    default:
      return NFCTAG_ERROR;
  }

  /* Update Sector Security Status */
  reg_value = (RfProtZone.RWprotection << ST25DV_RFZSS_RWPROT_SHIFT) & ST25DV_RFZSS_RWPROT_MASK;
  reg_value |= ((RfProtZone.PasswdCtrl << ST25DV_RFZSS_PWDCTRL_SHIFT) & ST25DV_RFZSS_PWDCTRL_MASK);

  /* Write Sector Security register */
  return ST25DV_i2c_WriteRegister(&reg_value, sector_security_addr, 1);
}

/**
  * @brief  Reads the value of the an area end address.
  * @param  EndZone ST25DV_END_ZONE value corresponding to an area end address.
  * @param  pEndZ   Pointer used to return the end address of the area.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadEndZonex(const ST25DV_END_ZONE EndZone, uint8_t *const pEndZ)
{
  uint16_t mem_addr;

  /* End zone register address to read */
  switch (EndZone) {
    case ST25DV_ZONE_END1:
      mem_addr = ST25DV_END1_REG;
      break;
    case ST25DV_ZONE_END2:
      mem_addr = ST25DV_END2_REG;
      break;
    case ST25DV_ZONE_END3:
      mem_addr = ST25DV_END3_REG;
      break;

    default:
      return NFCTAG_ERROR;
  }

  /* Read the corresponding End zone */
  return ST25DV_i2c_ReadRegister(pEndZ, mem_addr, 1);
}

/**
  * @brief    Sets the end address of an area.
  * @details  Needs the I2C Password presentation to be effective.
  * @note     The ST25DV answers a NACK when setting the EndZone2 & EndZone3 to same value than respectively EndZone1 & EndZone2.\n
  *           These NACKs are ok.
  * @param  EndZone ST25DV_END_ZONE value corresponding to an area.
  * @param  EndZ   End zone value to be written.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteEndZonex(const ST25DV_END_ZONE EndZone, const uint8_t EndZ)
{
  uint16_t mem_addr;
  NFCTAG_StatusTypeDef ret;

  /* End zone register address to write */
  switch (EndZone) {
    case ST25DV_ZONE_END1:
      mem_addr = ST25DV_END1_REG;
      break;
    case ST25DV_ZONE_END2:
      mem_addr = ST25DV_END2_REG;
      break;
    case ST25DV_ZONE_END3:
      mem_addr = ST25DV_END3_REG;
      break;

    default:
      return NFCTAG_ERROR;
  }

  /* Write the corresponding End zone value in register */
  ret = ST25DV_i2c_WriteRegister(&EndZ, mem_addr, 1);

  return ret;
}

/**
  * @brief  Initializes the end address of the ST25DV areas with their default values (end of memory).
  * @details  Needs the I2C Password presentation to be effective..
  *           The ST25DV answers a NACK when setting the EndZone2 & EndZone3 to same value than respectively EndZone1 & EndZone2.
  *           These NACKs are ok.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_InitEndZone(void)
{
  uint8_t endval = 0xFF;
  uint32_t maxmemlength;
  ST25DV_MEM_SIZE memsize;
  NFCTAG_StatusTypeDef ret;

  memsize.Mem_Size = 0;
  memsize.BlockSize = 0;

  /* Get EEPROM mem size */
  ST25DV_i2c_ReadMemSize(&memsize);
  maxmemlength = (memsize.Mem_Size + 1) * (memsize.BlockSize + 1);

  /* Compute Max value for endzone register */
  endval = (maxmemlength / 32) - 1;

  /* Write EndZone value to ST25DV registers */
  ret = ST25DV_i2c_WriteEndZonex(ST25DV_ZONE_END3, endval);
  if ((ret != NFCTAG_OK) && (ret != NFCTAG_NACK)) {
    return ret;
  }

  ret = ST25DV_i2c_WriteEndZonex(ST25DV_ZONE_END2, endval);
  if ((ret != NFCTAG_OK) && (ret != NFCTAG_NACK)) {
    return ret;
  }

  ret = ST25DV_i2c_WriteEndZonex(ST25DV_ZONE_END1, endval);
  if ((ret != NFCTAG_OK) && (ret != NFCTAG_NACK)) {
    return ret;
  }

  return ret;
}

/**
  * @brief  Creates user areas with defined lengths.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  Zone1Length Length of area1 in bytes (32 to 8192, 0x20 to 0x2000)
  * @param  Zone2Length Length of area2 in bytes (0 to 8128, 0x00 to 0x1FC0)
  * @param  Zone3Length Length of area3 in bytes (0 to 8064, 0x00 to 0x1F80)
  * @param  Zone4Length Length of area4 in bytes (0 to 8000, 0x00 to 0x1F40)
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_CreateUserZone(uint16_t Zone1Length, uint16_t Zone2Length, uint16_t Zone3Length, uint16_t Zone4Length)
{
  uint8_t EndVal;
  ST25DV_MEM_SIZE memsize;
  uint16_t maxmemlength = 0;
  NFCTAG_StatusTypeDef ret;

  memsize.Mem_Size = 0;
  memsize.BlockSize = 0;

  ST25DV_i2c_ReadMemSize(&memsize);

  maxmemlength = (memsize.Mem_Size + 1) * (memsize.BlockSize + 1);

  /* Checks that values of different zones are in bounds */
  if ((Zone1Length < 32) || (Zone1Length > maxmemlength) || (Zone2Length > (maxmemlength - 32))
      || (Zone3Length > (maxmemlength - 64)) || (Zone4Length > (maxmemlength - 96))) {
    return NFCTAG_ERROR;
  }

  /* Checks that the total is less than the authorised maximum */
  if ((Zone1Length + Zone2Length + Zone3Length + Zone4Length) > maxmemlength) {
    return NFCTAG_ERROR;
  }

  /* if The value for each Length is not a multiple of 64 correct it. */
  if ((Zone1Length % 32) != 0) {
    Zone1Length = Zone1Length - (Zone1Length % 32);
  }

  if ((Zone2Length % 32) != 0) {
    Zone2Length = Zone2Length - (Zone2Length % 32);
  }

  if ((Zone3Length % 32) != 0) {
    Zone3Length = Zone3Length - (Zone3Length % 32);
  }

  /* First right 0xFF in each Endx value */
  ret = ST25DV_i2c_InitEndZone();
  if ((ret != NFCTAG_OK) && (ret != NFCTAG_NACK)) {
    return ret;
  }

  /* Then Write corresponding value for each zone */
  EndVal = (uint8_t)((Zone1Length / 32) - 1);
  ret = ST25DV_i2c_WriteEndZonex(ST25DV_ZONE_END1, EndVal);
  if ((ret != NFCTAG_OK) && (ret != NFCTAG_NACK)) {
    return ret;
  }

  EndVal = (uint8_t)(((Zone1Length + Zone2Length) / 32) - 1);
  ret = ST25DV_i2c_WriteEndZonex(ST25DV_ZONE_END2, EndVal);
  if ((ret != NFCTAG_OK) && (ret != NFCTAG_NACK)) {
    return ret;
  }

  EndVal = (uint8_t)(((Zone1Length + Zone2Length + Zone3Length) / 32) - 1);
  ret = ST25DV_i2c_WriteEndZonex(ST25DV_ZONE_END3, EndVal);
  if ((ret != NFCTAG_OK) && (ret != NFCTAG_NACK)) {
    return ret;
  }

  return NFCTAG_OK;
}

/**
  * @brief  Reads the ST25DV Memory Size.
  * @param  pSizeInfo Pointer on a ST25DV_MEM_SIZE structure used to return the Memory size information.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadMemSize(ST25DV_MEM_SIZE *const pSizeInfo)
{
  uint8_t reg_value[3];
  NFCTAG_StatusTypeDef status;

  /* Read actual value of MEM_SIZE register */
  status = ST25DV_i2c_ReadRegister(reg_value, ST25DV_MEM_SIZE_REG, 3);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Memory information */
  pSizeInfo->BlockSize = reg_value[2];
  pSizeInfo->Mem_Size = reg_value[1];
  pSizeInfo->Mem_Size = (pSizeInfo->Mem_Size << 8) | reg_value[0];
  return NFCTAG_OK;
}

/**
  * @brief  Reads the Energy harvesting mode.
  * @param  pEH_mode Pointer on a ST25DV_EH_MODE_STATUS value corresponding to the Energy Harvesting state.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadEHMode(ST25DV_EH_MODE_STATUS *const pEH_mode)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of EH_MODE register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_EH_MODE_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract EH_mode configuration */
  if ((reg_value & ST25DV_EH_MODE_MASK) == ST25DV_EH_MODE_MASK) {
    *pEH_mode = ST25DV_EH_ON_DEMAND;
  } else {
    *pEH_mode = ST25DV_EH_ACTIVE_AFTER_BOOT;
  }

  return NFCTAG_OK;
}

/**
  * @brief  Sets the Energy harvesting mode.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  EH_mode ST25DV_EH_MODE_STATUS value for the Energy harvesting mode to be set.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteEHMode(const ST25DV_EH_MODE_STATUS EH_mode)
{
  uint8_t reg_value;

  /* Update EH_mode */
  reg_value = (uint8_t)EH_mode;

  /* Write EH_MODE register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_EH_MODE_REG, 1);
}

/**
  * @brief  Reads the RF Management configuration.
  * @param  pRF_Mngt Pointer on a ST25DV_RF_MNGT structure used to return the RF Management configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadRFMngt(ST25DV_RF_MNGT *const pRF_Mngt)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);

  if (status == NFCTAG_OK) {
    /* Extract RF Disable information */
    if ((reg_value & ST25DV_RF_MNGT_RFDIS_MASK) == ST25DV_RF_MNGT_RFDIS_MASK) {
      pRF_Mngt->RfDisable = ST25DV_ENABLE;
    } else {
      pRF_Mngt->RfDisable = ST25DV_DISABLE;
    }

    /* Extract RF Sleep information */
    if ((reg_value & ST25DV_RF_MNGT_RFSLEEP_MASK) == ST25DV_RF_MNGT_RFSLEEP_MASK) {
      pRF_Mngt->RfSleep = ST25DV_ENABLE;
    } else {
      pRF_Mngt->RfSleep = ST25DV_DISABLE;
    }
  }

  return status;
}

/**
  * @brief  Sets the RF Management configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  Rfmngt Value of the RF Management configuration to be written.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteRFMngt(const uint8_t Rfmngt)
{
  /* Write RF_MNGT register */
  return ST25DV_i2c_WriteRegister(&Rfmngt, ST25DV_RF_MNGT_REG, 1);
}

/**
  * @brief  Reads the RFDisable register information.
  * @param  pRFDisable Pointer on a ST25DV_EN_STATUS value corresponding to the RF Disable status.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetRFDisable(ST25DV_EN_STATUS *const pRFDisable)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);

  /* Extract RFDisable information */
  if (status == NFCTAG_OK) {
    if ((reg_value & ST25DV_RF_MNGT_RFDIS_MASK) == ST25DV_RF_MNGT_RFDIS_MASK) {
      *pRFDisable = ST25DV_ENABLE;
    } else {
      *pRFDisable = ST25DV_DISABLE;
    }
    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Sets the RF Disable configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_SetRFDisable(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_RMNGT register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update RF Disable field configuration */
  reg_value |= ST25DV_RF_MNGT_RFDIS_MASK;

  /* Write RF_MNGT register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
}

/**
  * @brief  Resets the RF Disable configuration
  * @details  Needs the I2C Password presentation to be effective.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ResetRFDisable(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_RMNGT register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update RF Disable field configuration */
  reg_value &= ST25DV_RF_MNGT_RFDIS_FIELD;

  /* Write RF_MNGT register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
}

/**
  * @brief  Reads the RFSleep register information.
  * @param  pRFSleep Pointer on a ST25DV_EN_STATUS value corresponding to the RF Sleep status.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetRFSleep(ST25DV_EN_STATUS *const pRFSleep)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);

  /* Extract RFSleep information */
  if (status == NFCTAG_OK) {
    if ((reg_value & ST25DV_RF_MNGT_RFDIS_MASK) == ST25DV_RF_MNGT_RFDIS_MASK) {
      *pRFSleep = ST25DV_ENABLE;
    } else {
      *pRFSleep = ST25DV_DISABLE;
    }
    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Sets the RF Sleep configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_SetRFSleep(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_RMNGT register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update RF Sleep field configuration */
  reg_value |= ST25DV_RF_MNGT_RFSLEEP_MASK;

  /* Write RF_MNGT register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
}

/**
  * @brief  Resets the RF Sleep configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ResetRFSleep(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_RMNGT register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update RF Sleep field configuration */
  reg_value &= ST25DV_RF_MNGT_RFSLEEP_FIELD;

  /* Write RF_MNGT register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_RF_MNGT_REG, 1);
}

/**
  * @brief  Reads the Mailbox mode.
  * @param  pMB_mode Pointer on a ST25DV_EH_MODE_STATUS value used to return the Mailbox mode.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadMBMode(ST25DV_EN_STATUS *const pMB_mode)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of MB_MODE register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_MB_MODE_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Mailbox mode status */
  if ((reg_value & ST25DV_MB_MODE_RW_MASK) == ST25DV_MB_MODE_RW_MASK) {
    *pMB_mode = ST25DV_ENABLE;
  } else {
    *pMB_mode = ST25DV_DISABLE;
  }

  return NFCTAG_OK;
}

/**
  * @brief  Sets the Mailbox mode.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  MB_mode ST25DV_EN_STATUS value corresponding to the Mailbox mode to be set.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteMBMode(const ST25DV_EN_STATUS MB_mode)
{
  uint8_t reg_value;

  /* Update Mailbox mode status */
  reg_value = (uint8_t)MB_mode;

  /* Write MB_MODE register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_MB_MODE_REG, 1);
}

/**
  * @brief  Reads the Mailbox watchdog duration coefficient.
  * @param  pWdgDelay Pointer on a uint8_t used to return the watchdog duration coefficient.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadMBWDG(uint8_t *const pWdgDelay)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of MB_WDG register */
  status = ST25DV_i2c_ReadRegister(&reg_value, ST25DV_MB_WDG_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract watchdog coefficient delay configuration */
  *pWdgDelay = (reg_value & ST25DV_MB_WDG_DELAY_MASK) >> ST25DV_MB_WDG_DELAY_RW_SHIFT;

  return NFCTAG_OK;
}

/**
  * @brief  Writes the Mailbox watchdog coefficient delay
  * @details  Needs the I2C Password presentation to be effective.
  * @param  WdgDelay Watchdog duration coefficient to be written (Watch dog duration = MB_WDG*30 ms +/- 6%).
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteMBWDG(const uint8_t WdgDelay)
{
  uint8_t reg_value;

  /* Set Watchdog coefficient delay */
  reg_value = WdgDelay & ST25DV_MB_WDG_DELAY_MASK;

  /* Write MB_MODE register */
  return ST25DV_i2c_WriteRegister(&reg_value, ST25DV_MB_WDG_REG, 1);
}

/**
  * @brief  Reads N bytes of data from the Mailbox, starting at the specified byte offset.
  * @param  pData   Pointer on the buffer used to return the read data.
  * @param  Offset  Offset in the Mailbox memory, byte number to start the read.
  * @param  NbByte  Number of bytes to be read.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadMailboxData(uint8_t *const pData, const uint16_t Offset, const uint16_t NbByte)
{
  if (Offset > ST25DV_MAX_MAILBOX_LENGTH) {
    return NFCTAG_ERROR;
  }

  /* Read Data in user memory */
  return ST25DV_IO_MemRead(pData, ST25DV_ADDR_DATA_I2C, ST25DV_MAILBOX_RAM_REG + Offset, NbByte);
}

/**
  * @brief  Writes N bytes of data in the Mailbox, starting from first Mailbox Address.
  * @param  pData   Pointer to the buffer containing the data to be written.
  * @param  NbByte  Number of bytes to be written.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteMailboxData(const uint8_t *const pData, const uint16_t NbByte)
{
  NFCTAG_StatusTypeDef status;

  /* ST25DV can write a maximum of 256 bytes in Mailbox */
  if (NbByte < ST25DV_MAX_MAILBOX_LENGTH) {
    /* Write NbByte data in memory */
    status = ST25DV_IO_MemWrite(pData, ST25DV_ADDR_DATA_I2C, ST25DV_MAILBOX_RAM_REG, NbByte);
  } else {
    status = NFCTAG_ERROR;
  }

  return status;
}

/**
  * @brief  Reads N bytes from the mailbox registers, starting at the specified I2C address.
  * @param  pData   Pointer on the buffer used to return the data.
  * @param  TarAddr I2C memory address to be read.
  * @param  NbByte  Number of bytes to be read.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadMailboxRegister(uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte)
{
  if ((TarAddr < ST25DV_GPO_DYN_REG) || (TarAddr > ST25DV_MB_LEN_DYN_REG)) {
    return NFCTAG_ERROR;
  }

  return ST25DV_IO_MemRead(pData, ST25DV_ADDR_DATA_I2C, TarAddr, NbByte);
}

/**
  * @brief  Writes N bytes to the specified mailbox register.
  * @param  pData   Pointer on the data to be written.
  * @param  TarAddr I2C register address to be written.
  * @param  NbByte  Number of bytes to be written.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteMailboxRegister(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte)
{
  NFCTAG_StatusTypeDef status;

  if ((TarAddr < ST25DV_GPO_DYN_REG) || (TarAddr > ST25DV_MB_LEN_DYN_REG)) {
    return NFCTAG_ERROR;
  }

  /* ST25DV can write a maximum of 256 bytes in Mailbox */
  if (NbByte < ST25DV_MAX_MAILBOX_LENGTH) {
    /* Write NbByte data in memory */
    status = ST25DV_IO_MemWrite(pData, ST25DV_ADDR_DATA_I2C, TarAddr, NbByte);
  } else {
    status = NFCTAG_ERROR;
  }

  return status;
}

/**
  * @brief  Reads the status of the security session open register.
  * @param  pSession Pointer on a ST25DV_I2CSSO_STATUS value used to return the session status.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadI2CSecuritySession_Dyn(ST25DV_I2CSSO_STATUS *const pSession)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of I2C_SSO_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_I2C_SSO_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Open session information */
  if ((reg_value & ST25DV_I2C_SSO_DYN_I2CSSO_MASK) == ST25DV_I2C_SSO_DYN_I2CSSO_MASK) {
    *pSession = ST25DV_SESSION_OPEN;
  } else {
    *pSession = ST25DV_SESSION_CLOSED;
  }

  return NFCTAG_OK;
}

/**
  * @brief  Reads the IT status register from the ST25DV.
  * @param  pITStatus Pointer on uint8_t, used to return the IT status, such as:
  *                       - RFUSERSTATE = 0x01
  *                       - RFBUSY = 0x02
  *                       - RFINTERRUPT = 0x04
  *                       - FIELDFALLING = 0x08
  *                       - FIELDRISING = 0x10
  *                       - RFPUTMSG = 0x20
  *                       - RFGETMSG = 0x40
  *                       - RFWRITE = 0x80
  *
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadITSTStatus_Dyn(uint8_t *const pITStatus)
{
  /* Read value of ITStatus register */
  return ST25DV_i2c_ReadMailboxRegister(pITStatus, ST25DV_ITSTS_DYN_REG, 1);
}

/**
  * @brief  Read value of dynamic GPO register configuration.
  * @param  pGPO ST25DV_GPO pointer of the dynamic GPO configuration to store.
  * @retval NFCTAG enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadGPO_Dyn(uint8_t *GPOConfig)
{
  /* Read actual value of ST25DV_GPO_DYN_REG register */
  return ST25DV_i2c_ReadMailboxRegister(GPOConfig, ST25DV_GPO_DYN_REG, 1);
}


/**
  * @brief  Get dynamique GPO enable status
  * @param  pGPO_en ST25DV_EN_STATUS pointer of the GPO enable status to store
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetGPO_en_Dyn(ST25DV_EN_STATUS *const pGPO_en)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;
  /* Read actual value of GPO_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_GPO_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract GPO enable status information */
  if ((reg_value & ST25DV_GPO_DYN_ENABLE_MASK) == ST25DV_GPO_DYN_ENABLE_MASK) {
    *pGPO_en = ST25DV_ENABLE;
  } else {
    *pGPO_en = ST25DV_DISABLE;
  }

  return NFCTAG_OK;
}

/**
  * @brief  Set dynamique GPO enable configuration.
  * @param  None No parameters.
  * @retval NFCTAG enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_SetGPO_en_Dyn(void)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of GPO_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_GPO_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update GPO enable configuration */
  reg_value |= ST25DV_GPO_DYN_ENABLE_MASK;

  /* Write GPO_DYN Register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_GPO_DYN_REG, 1);
}

/**
  * @brief  Reset dynamique GPO enable configuration.
  * @param  None No parameters.
  * @retval NFCTAG enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ResetGPO_en_Dyn(void)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of GPO_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_GPO_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update GPO enable configuration */
  reg_value &= ST25DV_GPO_DYN_ENABLE_FIELD;

  /* Write GPO_DYN Register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_GPO_DYN_REG, 1);
}

/**
  * @brief  Read value of dynamic EH Ctrl register configuration
  * @param  pEH_CTRL : ST25DV_EH_CTRL pointer of the dynamic EH Ctrl configuration to store
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadEHCtrl_Dyn(ST25DV_EH_CTRL *const pEH_CTRL)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of ST25DV_EH_CTRL_DYN_REG register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);

  if (status == NFCTAG_OK) {
    /* Extract EH EN Mode configuration */
    if ((reg_value & ST25DV_EH_CTRL_DYN_EH_EN_MASK) == ST25DV_EH_CTRL_DYN_EH_EN_MASK) {
      pEH_CTRL->EH_EN_Mode = ST25DV_ENABLE;
    } else {
      pEH_CTRL->EH_EN_Mode = ST25DV_DISABLE;
    }

    /* Extract EH_ON configuration */
    if ((reg_value & ST25DV_EH_CTRL_DYN_EH_ON_MASK) == ST25DV_EH_CTRL_DYN_EH_ON_MASK) {
      pEH_CTRL->EH_on = ST25DV_ENABLE;
    } else {
      pEH_CTRL->EH_on = ST25DV_DISABLE;
    }

    /* Extract FIELD_ON configuration */
    if ((reg_value & ST25DV_EH_CTRL_DYN_FIELD_ON_MASK) == ST25DV_EH_CTRL_DYN_FIELD_ON_MASK) {
      pEH_CTRL->Field_on = ST25DV_ENABLE;
    } else {
      pEH_CTRL->Field_on = ST25DV_DISABLE;
    }

    /* Extract VCC_ON configuration */
    if ((reg_value & ST25DV_EH_CTRL_DYN_VCC_ON_MASK) == ST25DV_EH_CTRL_DYN_VCC_ON_MASK) {
      pEH_CTRL->VCC_on = ST25DV_ENABLE;
    } else {
      pEH_CTRL->VCC_on = ST25DV_DISABLE;
    }

    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Reads the Energy Harvesting dynamic status.
  * @param  pEH_Val Pointer on a ST25DV_EN_STATUS value used to return the Energy Harvesting dynamic status.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetEHENMode_Dyn(ST25DV_EN_STATUS *const pEH_Val)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of EH_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Energy Harvesting status information */
  if ((reg_value & ST25DV_EH_CTRL_DYN_EH_EN_MASK) == ST25DV_EH_CTRL_DYN_EH_EN_MASK) {
    *pEH_Val = ST25DV_ENABLE;
  } else {
    *pEH_Val = ST25DV_DISABLE;
  }

  return NFCTAG_OK;
}

/**
  * @brief  Dynamically sets the Energy Harvesting mode.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_SetEHENMode_Dyn(void)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of EH_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update Energy Harvesting configuration */
  reg_value |= ST25DV_EH_CTRL_DYN_EH_EN_MASK;

  /* Write EH_CTRL_DYN Register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);
}

/**
  * @brief  Dynamically unsets the Energy Harvesting mode.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ResetEHENMode_Dyn(void)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read actual value of EH_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update Energy Harvesting configuration */
  reg_value &= ST25DV_EH_CTRL_DYN_EH_EN_FIELD;

  /* Write EH_CTRL_DYN Register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);
}

/**
  * @brief  Reads the EH_ON status from the EH_CTRL_DYN register.
  * @param  pEHON Pointer on a ST25DV_EN_STATUS value used to return the EHON status.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetEHON_Dyn(ST25DV_EN_STATUS *const pEHON)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of EH_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);

  /* Extract RF Field information */
  if (status == NFCTAG_OK) {
    if ((reg_value & ST25DV_EH_CTRL_DYN_EH_ON_MASK) == ST25DV_EH_CTRL_DYN_EH_ON_MASK) {
      *pEHON = ST25DV_ENABLE;
    } else {
      *pEHON = ST25DV_DISABLE;
    }
    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Checks if RF Field is present in front of the ST25DV.
  * @param  pRF_Field Pointer on a ST25DV_FIELD_STATUS value used to return the field presence.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetRFField_Dyn(ST25DV_FIELD_STATUS *const pRF_Field)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of EH_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);

  /* Extract RF Field information */
  if (status == NFCTAG_OK) {
    if ((reg_value & ST25DV_EH_CTRL_DYN_FIELD_ON_MASK) == ST25DV_EH_CTRL_DYN_FIELD_ON_MASK) {
      *pRF_Field = ST25DV_FIELD_ON;
    } else {
      *pRF_Field = ST25DV_FIELD_OFF;
    }
    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Check if VCC is supplying the ST25DV.
  * @param  pVCC ST25DV_VCC_STATUS pointer of the VCC status to store
  * @retval NFCTAG enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetVCC_Dyn(ST25DV_VCC_STATUS *const pVCC)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of EH_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_EH_CTRL_DYN_REG, 1);

  /* Extract VCC information */
  if (status == NFCTAG_OK) {
    if ((reg_value & ST25DV_EH_CTRL_DYN_VCC_ON_MASK) == ST25DV_EH_CTRL_DYN_VCC_ON_MASK) {
      *pVCC = ST25DV_VCC_ON;
    } else {
      *pVCC = ST25DV_VCC_OFF;
    }
    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Read value of dynamic RF Management configuration
  * @param  pRF_Mngt : ST25DV_RF_MNGT pointer of the dynamic RF Management configuration to store
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadRFMngt_Dyn(ST25DV_RF_MNGT *const pRF_Mngt)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);

  if (status == NFCTAG_OK) {
    /* Extract RF Disable configuration */
    if ((reg_value & ST25DV_RF_MNGT_DYN_RFDIS_MASK) == ST25DV_RF_MNGT_DYN_RFDIS_MASK) {
      pRF_Mngt->RfDisable = ST25DV_ENABLE;
    } else {
      pRF_Mngt->RfDisable = ST25DV_DISABLE;
    }

    /* Extract RF Sleep configuration */
    if ((reg_value & ST25DV_RF_MNGT_DYN_RFSLEEP_MASK) == ST25DV_RF_MNGT_DYN_RFSLEEP_MASK) {
      pRF_Mngt->RfSleep = ST25DV_ENABLE;
    } else {
      pRF_Mngt->RfSleep = ST25DV_DISABLE;
    }

    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Writes a value to the RF Management dynamic register.
  * @param  RF_Mngt Value to be written to the RF Management dynamic register.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_WriteRFMngt_Dyn(const uint8_t RF_Mngt)
{
  /* Write value to RF_MNGT_DYN register */
  return ST25DV_i2c_WriteMailboxRegister(&RF_Mngt, ST25DV_RF_MNGT_DYN_REG, 1);
}

/**
  * @brief  Reads the RFDisable dynamic register information.
  * @param  pRFDisable Pointer on a ST25DV_EN_STATUS value used to return the RF Disable state.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetRFDisable_Dyn(ST25DV_EN_STATUS *const pRFDisable)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);

  /* Extract RFDisable information */
  if (status == NFCTAG_OK) {
    if ((reg_value & ST25DV_RF_MNGT_DYN_RFDIS_MASK) == ST25DV_RF_MNGT_DYN_RFDIS_MASK) {
      *pRFDisable = ST25DV_ENABLE;
    } else {
      *pRFDisable = ST25DV_DISABLE;
    }
    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Sets the RF Disable dynamic configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_SetRFDisable_Dyn(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update dynamic RF Disable field */
  reg_value |= ST25DV_RF_MNGT_DYN_RFDIS_MASK;

  /* Write RF_MNGT_DYN register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
}

/**
  * @brief  Unsets the RF Disable dynamic configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ResetRFDisable_Dyn(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update dynamic RF Disable field configuration */
  reg_value &= ST25DV_RF_MNGT_DYN_RFDIS_FIELD;

  /* Write RF_MNGT_DYN register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
}

/**
  * @brief  Reads the RFSleep dynamic register information.
  * @param  pRFSleep Pointer on a ST25DV_EN_STATUS values used to return the RF Sleep state.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetRFSleep_Dyn(ST25DV_EN_STATUS *const pRFSleep)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);

  /* Extract RFSleep information */
  if (status == NFCTAG_OK) {
    if ((reg_value & ST25DV_RF_MNGT_DYN_RFDIS_MASK) == ST25DV_RF_MNGT_DYN_RFDIS_MASK) {
      *pRFSleep = ST25DV_ENABLE;
    } else {
      *pRFSleep = ST25DV_DISABLE;
    }
    return NFCTAG_OK;
  }

  return status;
}

/**
  * @brief  Sets the RF Sleep dynamic configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_SetRFSleep_Dyn(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update dynamic RF Disable field configuration */
  reg_value |= ST25DV_RF_MNGT_DYN_RFSLEEP_MASK;

  /* Write RF_MNGT_DYN register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
}

/**
  * @brief  Unsets the RF Sleep dynamic configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ResetRFSleep_Dyn(void)
{
  NFCTAG_StatusTypeDef status;
  uint8_t reg_value = 0;

  /* Read actual value of RF_MNGT_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Update dynamic RF Disable field configuration */
  reg_value &= ST25DV_RF_MNGT_DYN_RFSLEEP_FIELD;

  /* Write RF_MNGT_DYN register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_RF_MNGT_DYN_REG, 1);
}

/**
  * @brief  Reads the Mailbox ctrl dynamic register.
  * @param  pCtrlStatus Pointer on a ST25DV_MB_CTRL_DYN_STATUS structure used to return the dynamic Mailbox ctrl information.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadMBCtrl_Dyn(ST25DV_MB_CTRL_DYN_STATUS *const pCtrlStatus)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read MB_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_MB_CTRL_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  /* Extract Mailbox ctrl information */
  pCtrlStatus->MbEnable = (reg_value & ST25DV_MB_CTRL_DYN_MBEN_MASK) >> ST25DV_MB_CTRL_DYN_MBEN_SHIFT;
  pCtrlStatus->HostPutMsg = (reg_value & ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK) >> ST25DV_MB_CTRL_DYN_HOSTPUTMSG_SHIFT;
  pCtrlStatus->RfPutMsg = (reg_value & ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK) >> ST25DV_MB_CTRL_DYN_RFPUTMSG_SHIFT;
  pCtrlStatus->HostMissMsg = (reg_value & ST25DV_MB_CTRL_DYN_HOSTMISSMSG_MASK) >> ST25DV_MB_CTRL_DYN_HOSTMISSMSG_SHIFT;
  pCtrlStatus->RFMissMsg = (reg_value & ST25DV_MB_CTRL_DYN_RFMISSMSG_MASK) >> ST25DV_MB_CTRL_DYN_RFMISSMSG_SHIFT;
  pCtrlStatus->CurrentMsg = (ST25DV_CURRENT_MSG)((reg_value & ST25DV_MB_CTRL_DYN_CURRENTMSG_MASK) >> ST25DV_MB_CTRL_DYN_CURRENTMSG_SHIFT);

  return NFCTAG_OK;
}

/**
  * @brief  Reads the Mailbox Enable dynamic configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_GetMBEN_Dyn(ST25DV_EN_STATUS *const pMBEN)
{
  uint8_t reg_value;
  NFCTAG_StatusTypeDef status;

  /* Read MB_CTRL_DYN register */
  status = ST25DV_i2c_ReadMailboxRegister(&reg_value, ST25DV_MB_CTRL_DYN_REG, 1);
  if (status != NFCTAG_OK) {
    return status;
  }

  if ((reg_value & ST25DV_MB_MODE_RW_MASK) == ST25DV_MB_MODE_RW_MASK) {
    *pMBEN = ST25DV_ENABLE;
  } else {
    *pMBEN = ST25DV_DISABLE;
  }

  return NFCTAG_OK;
}

/**
  * @brief  Sets the Mailbox Enable dynamic configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_SetMBEN_Dyn(void)
{
  uint8_t reg_value;

  /* Set dynamic Mailbox enable */
  reg_value = ST25DV_MB_CTRL_DYN_MBEN_MASK;

  /* Write MB_CTRL_DYN register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_MB_CTRL_DYN_REG, 1);
}

/**
  * @brief  Unsets the Mailbox Enable dynamic configuration.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ResetMBEN_Dyn(void)
{
  uint8_t reg_value;

  /* Set dynamic Mailbox disable */
  reg_value = 0;

  /* Write MB_CTRL_DYN register */
  return ST25DV_i2c_WriteMailboxRegister(&reg_value, ST25DV_MB_CTRL_DYN_REG, 1);
}

/**
  * @brief  Reads the Mailbox message length dynamic register.
  * @param  pMBLength Pointer on a uint8_t used to return the Mailbox message length.
  * @return NFCTAG_StatusTypeDef enum status.
  */
NFCTAG_StatusTypeDef ST25DV_IO::ST25DV_i2c_ReadMBLength_Dyn(uint8_t *const pMBLength)
{
  /* Read actual value of MBLEN_DYN register */
  return ST25DV_i2c_ReadMailboxRegister(pMBLength, ST25DV_MB_LEN_DYN_REG, 1);
}

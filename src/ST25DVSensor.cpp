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

#include "ST25DVSensor.h"

int ST25DV::begin()
{
  return begin(NULL, 0);
}

int ST25DV::begin(uint8_t *buffer, uint16_t bufferLength)
{
  uint8_t nfctag_id = 0;

  if (!NfctagInitialized) {
    /* ST25DV Init */
    if (ST25DV_Init() != NFCTAG_OK) {
      return NFCTAG_ERROR;
    }

    /* Check ST25DV driver ID */
    st25dv_io.ST25DV_i2c_ReadID(&nfctag_id);

    if ((nfctag_id == I_AM_ST25DV04) || (nfctag_id == I_AM_ST25DV64) ||
        (nfctag_id == I_AM_ST25DV04KC) || (nfctag_id == I_AM_ST25DV64KC)) {
      NfctagInitialized = 1;
    } else {
      return NFCTAG_ERROR;
    }

    int ret = ndef.begin(buffer, bufferLength);
    if (ret != NDEF_OK) {
      return ret;
    }
  }
  return NFCTAG_OK;
};

int ST25DV::writeURI(String protocol, String uri, String info)
{
  sURI_Info _URI;

  // Unabridged protocols must be written using
  // `writeUnabridgedURI()`
  if (protocol.equals("")) {
    return NDEF_ERROR;
  }

  strcpy(_URI.protocol, protocol.c_str());
  strcpy(_URI.URI_Message, uri.c_str());
  strcpy(_URI.Information, info.c_str());

  return ndef.NDEF_WriteURI(&_URI);
}

int ST25DV::readURI(String *s)
{
  uint16_t ret;
  sURI_Info uri = {"", "", ""};
  sRecordInfo_t recordInfo;

  ret = ndef.NDEF_IdentifyNDEF(&recordInfo);
  if (ret) {
    return ret;
  }

  ret = ndef.NDEF_ReadURI(&recordInfo, &uri);
  if (ret) {
    return ret;
  }
  *s = String(uri.protocol) + String(uri.URI_Message);

  return 0;
}

/*
 * @brief Writes an unabbrieved URI
 *
 * The NFC NDEF format uses URI identifier code 0x00
 * to indicate a URI that is not abbreviated.
 *
 * @param uri the uri to write
 * @param info to write
 * @retval success or failure
 */
int ST25DV::writeUnabridgedURI(String uri, String info)
{
  sURI_Info _URI;

  strcpy(_URI.protocol, "");
  strcpy(_URI.URI_Message, uri.c_str());
  strcpy(_URI.Information, info.c_str());

  return ndef.NDEF_WriteURI(&_URI);
}

/*
 * @brief Reads an unabbrieved URI
 * @param s the uri read
 * @retval success or failure
 */
int ST25DV::readUnabridgedURI(String *s)
{
  uint16_t ret;
  sURI_Info uri = {"", "", ""};
  sRecordInfo_t recordInfo;

  ret = ndef.NDEF_IdentifyNDEF(&recordInfo);
  if (ret) {
    return ret;
  }

  ret = ndef.NDEF_ReadURI(&recordInfo, &uri);
  if (ret) {
    return ret;
  }

  // If the URI is abbreivated return error
  if (strncmp("", uri.protocol, 1) != 0) {
    return ret; //NDEF_ERROR;
  }

  *s = String(uri.URI_Message);

  return 0;
}

/*
 * @brief Writes an SMS record
 *
 * @param phoneNumber
 * @param message
 * @param info
 * @retval success or failure
 */
int ST25DV::writeSMS(String phoneNumber, String message, String info)
{
  sSMSInfo _SMS;

  strncpy(_SMS.PhoneNumber, phoneNumber.c_str(), 16);
  strncpy(_SMS.Message, message.c_str(), 400);
  strncpy(_SMS.Information, info.c_str(), 400);

  return ndef.NDEF_WriteSMS(&_SMS);
}

/*
 * @brief Reads an SMS record
 *
 * @param phoneNumber
 * @param message
 * @retval success or failure
 */
int ST25DV::readSMS(String *phoneNumber, String *message)
{
  uint16_t ret;
  sSMSInfo _SMS;
  sRecordInfo_t recordInfo;

  ret = ndef.NDEF_IdentifyNDEF(&recordInfo);
  if (ret) {
    return ret;
  }

  ret = ndef.NDEF_ReadSMS(&recordInfo, &_SMS);
  if (ret) {
    return ret;
  }

  *phoneNumber = String(_SMS.PhoneNumber);
  *message = String(_SMS.Message);

  return NDEF_OK;
}

/*
 * @brief Writes a GEO record
 *
 * @param latitude
 * @param longitude
 * @param info
 * @retval success or failure
 */
int ST25DV::writeGEO(String latitude, String longitude, String info)
{
  sGeoInfo _GEO;

  strncpy(_GEO.Latitude, latitude.c_str(), 20);
  strncpy(_GEO.Longitude, longitude.c_str(), 20);
  strncpy(_GEO.Information, info.c_str(), 100);

  return ndef.NDEF_WriteGeo(&_GEO);
}

/*
 * @brief Reads a GEO record
 *
 * @param latitude
 * @param longitude
 * @retval success or failure
 */
int ST25DV::readGEO(String *latitude, String *longitude)
{
  uint16_t ret;
  sGeoInfo _GEO;
  sRecordInfo_t recordInfo;

  ret = ndef.NDEF_IdentifyNDEF(&recordInfo);
  if (ret) {
    return ret;
  }

  ret = ndef.NDEF_ReadGeo(&recordInfo, &_GEO);
  if (ret) {
    return ret;
  }

  *latitude = String(_GEO.Latitude);
  *longitude = String(_GEO.Longitude);

  return NDEF_OK;
}


int ST25DV::writeEMail(String emailAdd, String subject, String message, String info)
{
  sEmailInfo _EMAIL;

  strncpy(_EMAIL.EmailAdd, emailAdd.c_str(), 64);
  strncpy(_EMAIL.Subject, subject.c_str(), 100);
  strncpy(_EMAIL.Message, message.c_str(), 2000);
  strncpy(_EMAIL.Information, info.c_str(), 400);

  return ndef.NDEF_WriteEmail(&_EMAIL);
}

int ST25DV::readEMail(String *emailAdd, String *subject, String *message)
{
  uint16_t ret;
  sEmailInfo _EMAIL;
  sRecordInfo_t recordInfo;

  ret = ndef.NDEF_IdentifyNDEF(&recordInfo);
  if (ret) {
    return ret;
  }

  ret = ndef.NDEF_ReadEmail(&recordInfo, &_EMAIL);
  if (ret) {
    return ret;
  }

  *emailAdd = String(_EMAIL.EmailAdd);
  *subject = String(_EMAIL.Subject);
  *message = String(_EMAIL.Message);

  return NDEF_OK;
}

/**
  * @brief  reads the type of NDEF on the tag
  * @param  None
  * @retval the type or UNKNOWN_TYPE if errors occur
  */
NDEF_TypeDef ST25DV::readNDEFType(void)
{
  uint16_t ret;
  sRecordInfo_t recordInfo;

  ret = ndef.NDEF_IdentifyNDEF(&recordInfo);
  if (ret) {
    return UNKNOWN_TYPE;
  }

  return recordInfo.NDEF_Type;
}

/**
  * @brief  Returns the NDEF class instance used by the component
  * @param  None
  * @retval success or failure
  */
NDEF *ST25DV::getNDEF(void)
{
  return &ndef;
}

/**
  * @brief  Initializes peripherals used by the I2C NFCTAG driver
  * @param  None
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV::ST25DV_Init(void)
{
  if (st25dv_io.get_pwire() == NULL) {
    return NFCTAG_ERROR;
  }

  ST25DV_GPO_Init();
  ST25DV_LPD_Init();

  ST25DV_I2C_Init();
  ST25DV_SelectI2cSpeed(3);

  return NFCTAG_OK;
}

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG GPO pin
  * @param  None
  * @retval None
  */
void ST25DV::ST25DV_GPO_Init(void)
{
  pinMode(st25dv_io.get_gpo(), INPUT);
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
uint8_t ST25DV::ST25DV_GPO_ReadPin(void)
{
  return digitalRead(st25dv_io.get_gpo());
}

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG LPD pin
  * @param  None
  * @retval None
  */
void ST25DV::ST25DV_LPD_Init(void)
{
  if (st25dv_io.get_lpd() > 0) {
    pinMode(st25dv_io.get_lpd(), OUTPUT);
    digitalWrite(st25dv_io.get_lpd(), LOW);
  }
}

/**
  * @brief  DeInit LPD.
  * @param  None.
  * @note LPD DeInit does not disable the GPIO clock nor disable the Mfx
  */
void ST25DV::ST25DV_LPD_DeInit(void)
{
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
uint8_t ST25DV::ST25DV_LPD_ReadPin(void)
{
  return digitalRead(st25dv_io.get_lpd());
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
void ST25DV::ST25DV_LPD_WritePin(uint8_t LpdPinState)
{
  digitalWrite(st25dv_io.get_lpd(), LpdPinState);
}

/**
  * @brief  This function select the I2C1 speed to communicate with NFCTAG
  * @param  i2cspeedchoice Number from 0 to 5 to select i2c speed
  * @retval HAL GPIO pin status
  */
void ST25DV::ST25DV_SelectI2cSpeed(uint8_t i2cspeedchoice)
{
  if (st25dv_io.get_pwire() == NULL) {
    return;
  }

#if !defined(ARDUINO_ARCH_ARC) && !defined(ARDUINO_ARCH_ARC32)
  switch (i2cspeedchoice) {
    case 0:

      st25dv_io.get_pwire()->setClock(10000);
      break;

    case 1:

      st25dv_io.get_pwire()->setClock(100000);
      break;

    case 2:

      st25dv_io.get_pwire()->setClock(200000);
      break;

    case 3:

      st25dv_io.get_pwire()->setClock(400000);
      break;

    case 4:

      st25dv_io.get_pwire()->setClock(800000);
      break;

    case 5:

      st25dv_io.get_pwire()->setClock(1000000);
      break;

    default:

      st25dv_io.get_pwire()->setClock(1000000);
      break;
  }
}

/**
  * @brief  This function initialize the I2C
  * @param  None
  * @retval None
  */
void ST25DV::ST25DV_I2C_Init(void)
{
  st25dv_io.get_pwire()->begin();
}

#endif

/**
  ******************************************************************************
  * @file    lib_NDEF_SMS.h
  * @author  MMY Application Team
  * @version $Revision: 2475 $
  * @date    $Date: 2016-06-24 12:11:59 +0200 (Fri, 24 Jun 2016) $
  * @brief   This file help to manage SMS NDEF file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_NDEF_SMS_H
#define __LIB_NDEF_SMS_H

/* Includes ------------------------------------------------------------------*/
#include "lib_NDEF.h"


typedef struct {
    char PhoneNumber[SMS_PHONE_MAX_SIZE];
    char Message[SMS_MESSAGE_MAX_SIZE];
    char Information[SMS_INFO_MAX_SIZE];
} sSMSInfo;

#endif /* __LIB_NDEF_SMS_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

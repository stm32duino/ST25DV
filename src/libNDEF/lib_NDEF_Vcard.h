/**
  ******************************************************************************
  * @file    lib_NDEF_Vcard.h
  * @author  MMY Application Team
  * @version $Revision: 2892 $
  * @date    $Date: 2016-09-19 11:03:26 +0200 (Mon, 19 Sep 2016) $
  * @brief   This file help to manage Vcard NDEF file.
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
#ifndef __LIB_NDEF_VCARD_H
#define __LIB_NDEF_VCARD_H

/* Includes ------------------------------------------------------------------*/
#include "lib_NDEF.h"


#define VCARD_VERSION_2_1             "2.1"
#define VCARD_VERSION_2_1_SIZE        3

#define VCARD_VERSION_3_0             "3.0"
#define VCARD_VERSION_3_0_SIZE        3


#define VCARD_BEGIN                   "BEGIN:"
#define VCARD                         "VCARD"
#define VERSION                       "VERSION:"
#define VCARD_NAME                     "N:"
// TODO: rename FIRSTNAME into FORMATTEDNAME
#define FIRSTNAME                     "FN:"
#define HOME_TEL                      "TEL;HOME:"
#define WORK_TEL                      "TEL;WORK:"
#define CELL_TEL                      "TEL;CELL:"
#define HOME_EMAIL                    "EMAIL;HOME:"
#define WORK_EMAIL                    "EMAIL;WORK:"
#define GEN_EMAIL                     "EMAIL:"
#define HOME_ADDRESS                  "ADR;HOME:"
#define WORK_ADDRESS                  "ADR;WORK:"
#define GEN_ADDRESS                   "ADR:"
#define TITLE                         "TITLE:"
#define ORG                           "ORG:"
#define URL                           "URL:"
#define VCARD_END                     "END:"
#define JPEG                          "JPEG"

#define LIMIT                         "\r\n"

#define VCARD_BEGIN_STRING_SIZE       6
#define VCARD_STRING_SIZE             5
#define VERSION_STRING_SIZE           8
#define VCARD_NAME_STRING_SIZE        2
// TODO: rename FIRSTNAME into FORMATTEDNAME
#define FIRSTNAME_STRING_SIZE         3
#define HOME_TEL_STRING_SIZE          9
#define WORK_TEL_STRING_SIZE          9
#define CELL_TEL_STRING_SIZE          9
#define HOME_EMAIL_STRING_SIZE        11
#define WORK_EMAIL_STRING_SIZE        11
#define HOME_ADDRESS_STRING_SIZE      9
#define WORK_ADDRESS_STRING_SIZE      9
#define TITLE_STRING_SIZE             6
#define ORG_STRING_SIZE               4
#define URL_STRING_SIZE               4
#define VCARD_END_STRING_SIZE         4
#define JPEG_STRING_SIZE              4

#define LIMIT_STRING_SIZE             2

typedef struct {
  char Version [10];
  char Name[80];
  // TODO: rename FirstName into FormattedName
  char FirstName[80];
  char Title[80];
  char Org[80];
  char HomeAddress[80];
  char WorkAddress[80];
  char Address[80];
  char HomeTel[40];
  char WorkTel[40];
  char CellTel[40];
  char HomeEmail[80];
  char WorkEmail[80];
  char Email[80];
  char Url[80];
} sVcardInfo;

#endif /* __LIB_NDEF_VCARD_H */


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

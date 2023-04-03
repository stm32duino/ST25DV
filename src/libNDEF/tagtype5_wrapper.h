/**
  ******************************************************************************
  * @file    tagtype5_wrapper.h
  * @author  MMY Application Team
  * @version $Revision: 3310 $
  * @date    $Date: 2017-01-13 11:22:18 +0100 (Fri, 13 Jan 2017) $
  * @brief   This file provides an abstraction layer to the libNDEF for the NFC Forum Type5 Tag.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
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
#ifndef __TAGTYPE5_WRAPPER_H
#define __TAGTYPE5_WRAPPER_H

/* Includes ------------------------------------------------------------------*/
#include "lib_NDEF.h"
#include "NDEF_class.h"

/** @addtogroup Tag_Type_5
  * @{
  */




/** @brief Type5 Tag Type-Length-Value structure as defined by the NFC Forum */
typedef struct {
  uint8_t   Type;     /**< NFC Forum message Type */
  uint8_t   Length;   /**< Message length if lesser than 255 bytes */
  uint16_t  Length16; /**< Message length if greater than or equal to 255 bytes */
} TT5_TLV_t;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @brief Memory size value indicating that this is a 8-bytes Capability Container */
#define NFCT5_EXTENDED_CCFILE             0x00
/** @brief Capability container version 1.0 */
#define NFCT5_VERSION_V1_0                0x40
/** @brief Read access condition mask for the Capability Container byte1 */
#define NFCT5_READ_ACCESS                 0x0C
/** @brief Write access condition mask for the Capability Container byte1 */
#define NFCT5_WRITE_ACCESS                0x03

/** @brief Type5 Tag NDEF message TLV-Type. */
#define NFCT5_NDEF_MSG_TLV                ((uint8_t) 0x03)
/** @brief Type5 Tag Proprietary message TLV-Type. */
#define NFCT5_PROPRIETARY_TLV             ((uint8_t) 0xFD)
/** @brief Type5 Tag Terminator TLV-Type. */
#define NFCT5_TERMINATOR_TLV              ((uint8_t) 0xFE)
/** @brief TLV-Length indicating a 4-bytes TLV (Length coded on 2 bytes). */
#define NFCT5_3_BYTES_L_TLV               ((uint8_t) 0xFF)

/* Exported macro ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
//extern sCCFileInfo CCFileStruct;


#endif /* __TAGTYPE5_WRAPPER_H */

/* @}
 */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

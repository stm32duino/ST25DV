/**
  ******************************************************************************
  * @file    st25dv_nfctag.h
  * @author  MMY Application Team
  * @version $Revision: 2983 $
  * @date    $Date: 2016-09-27 15:08:30 +0200 (Tue, 27 Sep 2016) $
  * @brief   This file contains definitions for the st25dv_nfctag.c
  *          specific functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
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
#ifndef __ST25DV_NFCTAG_H
#define __ST25DV_NFCTAG_H
/* Includes ------------------------------------------------------------------*/
#include "ST25DV/st25dv.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup BSP
  * @{
  */

/** @addtogroup ST25DV_NFCTAG
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define NFCTAG_4K_SIZE            ((uint32_t) 0x200)
#define NFCTAG_16K_SIZE           ((uint32_t) 0x800)
#define NFCTAG_64K_SIZE           ((uint32_t) 0x2000)
/* External variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported function  --------------------------------------------------------*/
/** @defgroup ST25DV_NFCTAG_Exported_Functions
  * @{
  */
NFCTAG_StatusTypeDef BSP_NFCTAG_Init(void);
void BSP_NFCTAG_DeInit(void);
uint8_t BSP_NFCTAG_isInitialized(void);
NFCTAG_StatusTypeDef BSP_NFCTAG_ReadID(uint8_t *const wai_id);
uint32_t BSP_NFCTAG_GetByteSize(void);
NFCTAG_StatusTypeDef BSP_NFCTAG_IsDeviceReady(const uint32_t Trials);
NFCTAG_StatusTypeDef BSP_NFCTAG_ConfigIT(const uint16_t ITConfig);
NFCTAG_StatusTypeDef BSP_NFCTAG_GetITStatus(uint16_t *const ITConfig);
NFCTAG_StatusTypeDef BSP_NFCTAG_ReadData(uint8_t *const pData, const uint16_t TarAddr, const uint16_t Size);
NFCTAG_StatusTypeDef BSP_NFCTAG_WriteData(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t Size);
NFCTAG_StatusTypeDef BSP_NFCTAG_ReadRegister(uint8_t *const pData, const uint16_t TarAddr, const uint16_t Size);
NFCTAG_StatusTypeDef BSP_NFCTAG_WriteRegister(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t Size);
NFCTAG_ExtDrvTypeDef *BSP_NFCTAG_GetExtended_Drv(void);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __ST25DV_NFCTAG_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

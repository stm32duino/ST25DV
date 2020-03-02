/**
  ******************************************************************************
  * @file    st25dv.h
  * @author  MMY Application Team
  * @version $Revision: 3308 $
  * @date    $Date: 2017-01-13 11:19:33 +0100 (Fri, 13 Jan 2017) $
  * @brief   This file provides set of driver functions to manage communication
  * @brief   between MCU and ST25DV chip
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
#ifndef __ST25DV_H
#define __ST25DV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */


/** @addtogroup ST25DV
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  NFCTAG status enumerator definition.
 */
typedef enum {
  NFCTAG_OK      = 0,
  NFCTAG_ERROR   = 1,
  NFCTAG_BUSY    = 2,
  NFCTAG_TIMEOUT = 3,
  NFCTAG_NACK    = 4
} NFCTAG_StatusTypeDef;

/**
 * @brief  ST25DV Enable Disable enumerator definition.
 */
typedef enum {
  ST25DV_DISABLE = 0,
  ST25DV_ENABLE
} ST25DV_EN_STATUS;

/**
 * @brief  ST25DV Energy Harvesting mode enumerator definition.
 */
typedef enum {
  ST25DV_EH_ACTIVE_AFTER_BOOT = 0,
  ST25DV_EH_ON_DEMAND
} ST25DV_EH_MODE_STATUS;

/**
 * @brief  ST25DV FIELD status enumerator definition.
 */
typedef enum {
  ST25DV_FIELD_OFF = 0,
  ST25DV_FIELD_ON
} ST25DV_FIELD_STATUS;

/**
 * @brief  ST25DV VCC status enumerator definition
 */
typedef enum {
  ST25DV_VCC_OFF = 0,
  ST25DV_VCC_ON
} ST25DV_VCC_STATUS;

/**
 * @brief  ST25DV protection status enumerator definition
 */
typedef enum {
  ST25DV_NO_PROT = 0,
  ST25DV_WRITE_PROT,
  ST25DV_READ_PROT,
  ST25DV_READWRITE_PROT
} ST25DV_PROTECTION_CONF;

/**
 * @brief  ST25DV area protection enumerator definition.
 */
typedef enum {
  ST25DV_PROT_ZONE1 = 0,
  ST25DV_PROT_ZONE2,
  ST25DV_PROT_ZONE3,
  ST25DV_PROT_ZONE4
} ST25DV_PROTECTION_ZONE;

/**
 * @brief  ST25DV password protection status enumerator definition.
 */
typedef enum {
  ST25DV_NOT_PROTECTED = 0,
  ST25DV_PROT_PASSWD1,
  ST25DV_PROT_PASSWD2,
  ST25DV_PROT_PASSWD3
} ST25DV_PASSWD_PROT_STATUS;

/**
 * @brief  ST25DV lock status enumerator definition.
 */
typedef enum {
  ST25DV_UNLOCKED = 0,
  ST25DV_LOCKED
} ST25DV_LOCK_STATUS;

/**
 * @brief  ST25DV Number of Blocks for the CCFile enumerator definition.
 */
typedef enum {
  ST25DV_CCFILE_1BLCK = 0,
  ST25DV_CCFILE_2BLCK
} ST25DV_CCFILE_BLOCK;

/**
 * @brief  ST25DV session status enumerator definition.
 */
typedef enum {
  ST25DV_SESSION_CLOSED = 0,
  ST25DV_SESSION_OPEN
} ST25DV_I2CSSO_STATUS;

/**
 * @brief  ST25DV area end address enumerator definition.
 */
typedef enum {
  ST25DV_ZONE_END1 = 0,
  ST25DV_ZONE_END2,
  ST25DV_ZONE_END3
} ST25DV_END_ZONE;

/**
 * @brief  ST25DV IT pulse duration enumerator definition.
 */
typedef enum {
  ST25DV_302_US = 0,
  ST25DV_264_US,
  ST25DV_226_US,
  ST25DV_188_US,
  ST25DV_151_US,
  ST25DV_113_US,
  ST25DV_75_US,
  ST25DV_37_US
} ST25DV_PULSE_DURATION;

/**
 * @brief  ST25DV Mailbox Current Message enumerator definition
 */
typedef enum {
  ST25DV_NO_MSG = 0,
  ST25DV_HOST_MSG,
  ST25DV_RF_MSG
} ST25DV_CURRENT_MSG;

/**
 * @brief  ST25DV EH Ctrl structure definition
 */
typedef struct {
  ST25DV_EN_STATUS EH_EN_Mode;
  ST25DV_EN_STATUS EH_on;
  ST25DV_EN_STATUS Field_on;
  ST25DV_EN_STATUS VCC_on;
} ST25DV_EH_CTRL;

/**
 * @brief  ST25DV GPO structure definition
 */
typedef struct {
  ST25DV_EN_STATUS GPO_RFUser_en;
  ST25DV_EN_STATUS GPO_RFActivity_en;
  ST25DV_EN_STATUS GPO_RFInterrupt_en;
  ST25DV_EN_STATUS GPO_FieldChange_en;
  ST25DV_EN_STATUS GPO_RFPutMsg_en;
  ST25DV_EN_STATUS GPO_RFGetMsg_en;
  ST25DV_EN_STATUS GPO_RFWrite_en;
  ST25DV_EN_STATUS GPO_Enable;
} ST25DV_GPO;

/**
 * @brief  ST25DV RF Management structure definition.
 */
typedef struct {
  ST25DV_EN_STATUS RfDisable;
  ST25DV_EN_STATUS RfSleep;
} ST25DV_RF_MNGT;

/**
 * @brief  ST25DV RF Area protection structure definition.
 */
typedef struct {
  ST25DV_PASSWD_PROT_STATUS PasswdCtrl;
  ST25DV_PROTECTION_CONF RWprotection;
} ST25DV_RF_PROT_ZONE;

/**
 * @brief  ST25DV I2C Area protection structure definition.
 */
typedef struct {
  ST25DV_PROTECTION_CONF ProtectZone1;
  ST25DV_PROTECTION_CONF ProtectZone2;
  ST25DV_PROTECTION_CONF ProtectZone3;
  ST25DV_PROTECTION_CONF ProtectZone4;
} ST25DV_I2C_PROT_ZONE;

/**
 * @brief  ST25DV MB_CTRL_DYN register structure definition.
 */
typedef struct {
  uint8_t MbEnable;
  uint8_t HostPutMsg;
  uint8_t RfPutMsg;
  uint8_t HostMissMsg;
  uint8_t RFMissMsg;
  ST25DV_CURRENT_MSG CurrentMsg;
} ST25DV_MB_CTRL_DYN_STATUS;

/**
 * @brief  ST25DV Lock CCFile structure definition.
 */
typedef struct {
  ST25DV_LOCK_STATUS LckBck0;
  ST25DV_LOCK_STATUS LckBck1;
} ST25DV_LOCK_CCFILE;

/**
 * @brief  ST25DV Memory size structure definition.
 */
typedef struct {
  uint8_t BlockSize;
  uint16_t Mem_Size;
} ST25DV_MEM_SIZE;

/**
 * @brief  ST25DV UID information structure definition.
 */
typedef struct {
  uint32_t MsbUid;
  uint32_t LsbUid;
} ST25DV_UID;

/**
 * @brief  ST25DV Password structure definition.
 */
typedef struct {
  uint32_t MsbPasswd;
  uint32_t LsbPasswd;
} ST25DV_PASSWD;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 * @brief  NFCTAG standard driver API structure definition.
 */
typedef struct {
  NFCTAG_StatusTypeDef(*Init)(void);
  NFCTAG_StatusTypeDef(*ReadID)(uint8_t *const);
  NFCTAG_StatusTypeDef(*IsReady)(const uint32_t);
  NFCTAG_StatusTypeDef(*GetITStatus)(uint16_t *const);
  NFCTAG_StatusTypeDef(*ConfigIT)(const uint16_t);
  NFCTAG_StatusTypeDef(*ReadData)(uint8_t *const, const uint16_t, const uint16_t);
  NFCTAG_StatusTypeDef(*WriteData)(const uint8_t *const, const uint16_t, const uint16_t);
  NFCTAG_StatusTypeDef(*ReadRegister)(uint8_t *const, const uint16_t, const uint16_t);
  NFCTAG_StatusTypeDef(*WriteRegister)(const uint8_t *const, const uint16_t, const uint16_t);
  void                       *pData;
} NFCTAG_DrvTypeDef;

/**
 * @brief  NFCTAG extended driver API structure definition.
 */
typedef struct {
  NFCTAG_StatusTypeDef(*ReadICRev)(uint8_t *const);
  NFCTAG_StatusTypeDef(*WriteITPulse)(const ST25DV_PULSE_DURATION);
  NFCTAG_StatusTypeDef(*ReadITPulse)(ST25DV_PULSE_DURATION *const);
  NFCTAG_StatusTypeDef(*ReadDataCurrentAddr)(uint8_t *const, const uint16_t);
  NFCTAG_StatusTypeDef(*ReadUID)(ST25DV_UID *const);
  NFCTAG_StatusTypeDef(*ReadDSFID)(uint8_t *const);
  NFCTAG_StatusTypeDef(*ReadDsfidRFProtection)(ST25DV_LOCK_STATUS *const);
  NFCTAG_StatusTypeDef(*ReadAFI)(uint8_t *const);
  NFCTAG_StatusTypeDef(*ReadAfiRFProtection)(ST25DV_LOCK_STATUS *const);
  NFCTAG_StatusTypeDef(*ReadI2CProtectZone)(ST25DV_I2C_PROT_ZONE *const);
  NFCTAG_StatusTypeDef(*WriteI2CProtectZonex)(const ST25DV_PROTECTION_ZONE, const ST25DV_PROTECTION_CONF);
  NFCTAG_StatusTypeDef(*ReadLockCCFile)(ST25DV_LOCK_CCFILE *const);
  NFCTAG_StatusTypeDef(*WriteLockCCFile)(const ST25DV_CCFILE_BLOCK, const ST25DV_LOCK_STATUS);
  NFCTAG_StatusTypeDef(*ReadLockCFG)(ST25DV_LOCK_STATUS *const);
  NFCTAG_StatusTypeDef(*WriteLockCFG)(const ST25DV_LOCK_STATUS);
  NFCTAG_StatusTypeDef(*PresentI2CPassword)(const ST25DV_PASSWD);
  NFCTAG_StatusTypeDef(*WriteI2CPassword)(const ST25DV_PASSWD);
  NFCTAG_StatusTypeDef(*ReadRFZxSS)(const ST25DV_PROTECTION_ZONE, ST25DV_RF_PROT_ZONE *const);
  NFCTAG_StatusTypeDef(*WriteRFZxSS)(const ST25DV_PROTECTION_ZONE, const ST25DV_RF_PROT_ZONE);
  NFCTAG_StatusTypeDef(*ReadEndZonex)(const ST25DV_END_ZONE, uint8_t *const);
  NFCTAG_StatusTypeDef(*WriteEndZonex)(const ST25DV_END_ZONE, const uint8_t);
  NFCTAG_StatusTypeDef(*InitEndZone)(void);
  NFCTAG_StatusTypeDef(*CreateUserZone)(uint16_t, uint16_t, uint16_t, uint16_t);
  NFCTAG_StatusTypeDef(*ReadMemSize)(ST25DV_MEM_SIZE *const);
  NFCTAG_StatusTypeDef(*ReadEHMode)(ST25DV_EH_MODE_STATUS *const);
  NFCTAG_StatusTypeDef(*WriteEHMode)(const ST25DV_EH_MODE_STATUS);
  NFCTAG_StatusTypeDef(*ReadRFMngt)(ST25DV_RF_MNGT *const);
  NFCTAG_StatusTypeDef(*WriteRFMngt)(const uint8_t);
  NFCTAG_StatusTypeDef(*GetRFDisable)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*SetRFDisable)(void);
  NFCTAG_StatusTypeDef(*ResetRFDisable)(void);
  NFCTAG_StatusTypeDef(*GetRFSleep)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*SetRFSleep)(void);
  NFCTAG_StatusTypeDef(*ResetRFSleep)(void);
  NFCTAG_StatusTypeDef(*ReadMBMode)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*WriteMBMode)(const ST25DV_EN_STATUS);
  NFCTAG_StatusTypeDef(*ReadMBWDG)(uint8_t *const);
  NFCTAG_StatusTypeDef(*WriteMBWDG)(const uint8_t);
  NFCTAG_StatusTypeDef(*ReadMailboxData)(uint8_t *const, const uint16_t, const uint16_t);
  NFCTAG_StatusTypeDef(*WriteMailboxData)(const uint8_t *const, const uint16_t);
  NFCTAG_StatusTypeDef(*ReadMailboxRegister)(uint8_t *const, const uint16_t, const uint16_t);
  NFCTAG_StatusTypeDef(*WriteMailboxRegister)(const uint8_t *const, const uint16_t, const uint16_t);
  NFCTAG_StatusTypeDef(*ReadI2CSecuritySession_Dyn)(ST25DV_I2CSSO_STATUS *const);
  NFCTAG_StatusTypeDef(*ReadITSTStatus_Dyn)(uint8_t *const);
  NFCTAG_StatusTypeDef(*ReadGPO_Dyn)(uint8_t *);
  NFCTAG_StatusTypeDef(*GetGPO_en_Dyn)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*SetGPO_en_Dyn)(void);
  NFCTAG_StatusTypeDef(*ResetGPO_en_Dyn)(void);
  NFCTAG_StatusTypeDef(*ReadEHCtrl_Dyn)(ST25DV_EH_CTRL *const);
  NFCTAG_StatusTypeDef(*GetEHENMode_Dyn)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*SetEHENMode_Dyn)(void);
  NFCTAG_StatusTypeDef(*ResetEHENMode_Dyn)(void);
  NFCTAG_StatusTypeDef(*GetEHON_Dyn)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*GetRFField_Dyn)(ST25DV_FIELD_STATUS *const);
  NFCTAG_StatusTypeDef(*GetVCC_Dyn)(ST25DV_VCC_STATUS *const);
  NFCTAG_StatusTypeDef(*ReadRFMngt_Dyn)(ST25DV_RF_MNGT *const);
  NFCTAG_StatusTypeDef(*WriteRFMngt_Dyn)(const uint8_t);
  NFCTAG_StatusTypeDef(*GetRFDisable_Dyn)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*SetRFDisable_Dyn)(void);
  NFCTAG_StatusTypeDef(*ResetRFDisable_Dyn)(void);
  NFCTAG_StatusTypeDef(*GetRFSleep_Dyn)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*SetRFSleep_Dyn)(void);
  NFCTAG_StatusTypeDef(*ResetRFSleep_Dyn)(void);
  NFCTAG_StatusTypeDef(*ReadMBctrl_Dyn)(ST25DV_MB_CTRL_DYN_STATUS *const);
  NFCTAG_StatusTypeDef(*GetMBEN_Dyn)(ST25DV_EN_STATUS *const);
  NFCTAG_StatusTypeDef(*SetMBEN_Dyn)(void);
  NFCTAG_StatusTypeDef(*ResetMBEN_Dyn)(void);
  NFCTAG_StatusTypeDef(*ReadMBLength_Dyn)(uint8_t *const);
} NFCTAG_ExtDrvTypeDef;
#endif

/* Exported constants --------------------------------------------------------*/
/** @brief ST25DV 4Kbits */
#define I_AM_ST25DV04                        0x24
/** @brief ST25DV 64Kbits */
#define I_AM_ST25DV64                        0x26

#ifndef NULL
#define NULL      (void *) 0
#endif

/** @brief I2C address to be used for ST25DV Data accesses. */
#define ST25DV_ADDR_DATA_I2C                 0xA6
/** @brief I2C address to be used for ST25DV System accesses. */
#define ST25DV_ADDR_SYST_I2C                 0xAE

/** @brief I2C Time out (ms), min value : (Max write bytes) / (Internal page write) * tw   (256/4)*5. */
#define ST25DV_I2C_TIMEOUT                   320

/** @brief Size of the ST25DV write buffer. */
#define ST25DV_MAX_WRITE_BYTE                256
/** @brief Size of the ST25DVMailbox memory. */
#define ST25DV_MAX_MAILBOX_LENGTH            256

/* Registers i2c address */
/** @brief ST25DV GPO register address. */
#define ST25DV_GPO_REG                       0x0000
/** @brief ST25DV IT duration register address. */
#define ST25DV_ITTIME_REG                    0x0001
/** @brief ST25DV Energy Harvesting register address. */
#define ST25DV_EH_MODE_REG                   0x0002
/** @brief ST25DV RF management register address. */
#define ST25DV_RF_MNGT_REG                   0x0003
/** @brief ST25DV Area 1 security register address. */
#define ST25DV_RFZ1SS_REG                    0x0004
/** @brief ST25DV Area 1 end address register address. */
#define ST25DV_END1_REG                      0x0005
/** @brief ST25DV Area 2 security register address. */
#define ST25DV_RFZ2SS_REG                    0x0006
/** @brief ST25DV Area 2 end address register address. */
#define ST25DV_END2_REG                      0x0007
/** @brief ST25DV Area 3 security register address. */
#define ST25DV_RFZ3SS_REG                    0x0008
/** @brief ST25DV Area 3 end address register address. */
#define ST25DV_END3_REG                      0x0009
/** @brief ST25DV Area 4 security register address. */
#define ST25DV_RFZ4SS_REG                    0x000A
/** @brief ST25DV I2C security register address. */
#define ST25DV_I2CZSS_REG                    0x000B
/** @brief ST25DV Capability Container lock register address. */
#define ST25DV_LOCKCCFILE_REG                0x000C
/** @brief ST25DV Mailbox mode register address. */
#define ST25DV_MB_MODE_REG                   0x000D
/** @brief ST25DV Mailbox Watchdog register address. */
#define ST25DV_MB_WDG_REG                    0x000E
/** @brief ST25DV Configuration lock register address. */
#define ST25DV_LOCKCFG_REG                   0x000F
/** @brief ST25DV DSFID lock register address. */
#define ST25DV_LOCKDSFID_REG                 0x0010
/** @brief ST25DV AFI lock register address. */
#define ST25DV_LOCKAFI_REG                   0x0011
/** @brief ST25DV DSFID register address. */
#define ST25DV_DSFID_REG                     0x0012
/** @brief ST25DV AFI register address. */
#define ST25DV_AFI_REG                       0x0013
/** @brief ST25DV Memory size register address. */
#define ST25DV_MEM_SIZE_REG                  0x0014
/** @brief ST25DV ICref register address. */
#define ST25DV_ICREF_REG                     0x0017
/** @brief ST25DV UID register address. */
#define ST25DV_UID_REG                       0x0018
/** @brief ST25DV IC revision register address. */
#define ST25DV_ICREV_REG                     0x0020
/** @brief ST25DV I2C password register address. */
#define ST25DV_I2CPASSWD_REG                 0x0900

/* Dynamic Registers i2c address */
/** @brief ST25DV GPO dynamic register address. */
#define ST25DV_GPO_DYN_REG                   0x2000
/** @brief ST25DV Energy Harvesting control dynamic register address. */
#define ST25DV_EH_CTRL_DYN_REG               0x2002
/** @brief ST25DV RF management dynamic register address. */
#define ST25DV_RF_MNGT_DYN_REG               0x2003
/** @brief ST25DV I2C secure session opened dynamic register address. */
#define ST25DV_I2C_SSO_DYN_REG               0x2004
/** @brief ST25DV Interrupt status dynamic register address. */
#define ST25DV_ITSTS_DYN_REG                 0x2005
/** @brief ST25DV Mailbox control dynamic register address. */
#define ST25DV_MB_CTRL_DYN_REG               0x2006
/** @brief ST25DV Mailbox message length dynamic register address. */
#define ST25DV_MBLEN_DYN_REG                 0x2007
/** @brief ST25DV Mailbox buffer address. */
#define ST25DV_MAILBOX_RAM_REG               0x2008

/* Registers fields definitions */
/* MB_MODE */
#define ST25DV_MB_MODE_RW_SHIFT              (0)
#define ST25DV_MB_MODE_RW_FIELD              0xFE
#define ST25DV_MB_MODE_RW_MASK               0x01

/* MB_LEN_Dyn */
#define ST25DV_MBLEN_DYN_MBLEN_SHIFT         (0)
#define ST25DV_MBLEN_DYN_MBLEN_FIELD         0x00
#define ST25DV_MBLEN_DYN_MBLEN_MASK          0xFF

/* MB_CTRL_Dyn */
#define ST25DV_MB_CTRL_DYN_MBEN_SHIFT        (0)
#define ST25DV_MB_CTRL_DYN_MBEN_FIELD        0xFE
#define ST25DV_MB_CTRL_DYN_MBEN_MASK         0x01
#define ST25DV_MB_CTRL_DYN_HOSTPUTMSG_SHIFT  (1)
#define ST25DV_MB_CTRL_DYN_HOSTPUTMSG_FIELD  0xFD
#define ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK   0x02
#define ST25DV_MB_CTRL_DYN_RFPUTMSG_SHIFT    (2)
#define ST25DV_MB_CTRL_DYN_RFPUTMSG_FIELD    0xFB
#define ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK     0x04
#define ST25DV_MB_CTRL_DYN_STRESERVED_SHIFT  (3)
#define ST25DV_MB_CTRL_DYN_STRESERVED_FIELD  0xF7
#define ST25DV_MB_CTRL_DYN_STRESERVED_MASK   0x08
#define ST25DV_MB_CTRL_DYN_HOSTMISSMSG_SHIFT (4)
#define ST25DV_MB_CTRL_DYN_HOSTMISSMSG_FIELD 0xEF
#define ST25DV_MB_CTRL_DYN_HOSTMISSMSG_MASK  0x10
#define ST25DV_MB_CTRL_DYN_RFMISSMSG_SHIFT   (5)
#define ST25DV_MB_CTRL_DYN_RFMISSMSG_FIELD   0xDF
#define ST25DV_MB_CTRL_DYN_RFMISSMSG_MASK    0x20
#define ST25DV_MB_CTRL_DYN_CURRENTMSG_SHIFT  (6)
#define ST25DV_MB_CTRL_DYN_CURRENTMSG_FIELD  0x3F
#define ST25DV_MB_CTRL_DYN_CURRENTMSG_MASK   0xC0

/* MB_WDG */
#define ST25DV_MB_WDG_DELAY_SHIFT            (0)
#define ST25DV_MB_WDG_DELAY_FIELD            0xF8
#define ST25DV_MB_WDG_DELAY_MASK             0x07

/* GPO */
#define ST25DV_GPO_RFUSERSTATE_SHIFT         (0)
#define ST25DV_GPO_RFUSERSTATE_FIELD         0xFE
#define ST25DV_GPO_RFUSERSTATE_MASK          0x01
#define ST25DV_GPO_RFACTIVITY_SHIFT          (1)
#define ST25DV_GPO_RFACTIVITY_FIELD          0xFD
#define ST25DV_GPO_RFACTIVITY_MASK           0x02
#define ST25DV_GPO_RFINTERRUPT_SHIFT         (2)
#define ST25DV_GPO_RFINTERRUPT_FIELD         0xFB
#define ST25DV_GPO_RFINTERRUPT_MASK          0x04
#define ST25DV_GPO_FIELDCHANGE_SHIFT         (3)
#define ST25DV_GPO_FIELDCHANGE_FIELD         0xF7
#define ST25DV_GPO_FIELDCHANGE_MASK          0x08
#define ST25DV_GPO_RFPUTMSG_SHIFT            (4)
#define ST25DV_GPO_RFPUTMSG_FIELD            0xEF
#define ST25DV_GPO_RFPUTMSG_MASK             0x10
#define ST25DV_GPO_RFGETMSG_SHIFT            (5)
#define ST25DV_GPO_RFGETMSG_FIELD            0xDF
#define ST25DV_GPO_RFGETMSG_MASK             0x20
#define ST25DV_GPO_RFWRITE_SHIFT             (6)
#define ST25DV_GPO_RFWRITE_FIELD             0xBF
#define ST25DV_GPO_RFWRITE_MASK              0x40
#define ST25DV_GPO_ENABLE_SHIFT              (7)
#define ST25DV_GPO_ENABLE_FIELD              0x7F
#define ST25DV_GPO_ENABLE_MASK               0x80
#define ST25DV_GPO_ALL_MASK                  0xFF

/* GPO_Dyn */
#define ST25DV_GPO_DYN_RFUSERSTATE_SHIFT     (0)
#define ST25DV_GPO_DYN_RFUSERSTATE_FIELD     0xFE
#define ST25DV_GPO_DYN_RFUSERSTATE_MASK      0x01
#define ST25DV_GPO_DYN_RFACTIVITY_SHIFT      (1)
#define ST25DV_GPO_DYN_RFACTIVITY_FIELD      0xFD
#define ST25DV_GPO_DYN_RFACTIVITY_MASK       0x02
#define ST25DV_GPO_DYN_RFINTERRUPT_SHIFT     (2)
#define ST25DV_GPO_DYN_RFINTERRUPT_FIELD     0xFB
#define ST25DV_GPO_DYN_RFINTERRUPT_MASK      0x04
#define ST25DV_GPO_DYN_FIELDCHANGE_SHIFT     (3)
#define ST25DV_GPO_DYN_FIELDCHANGE_FIELD     0xF7
#define ST25DV_GPO_DYN_FIELDCHANGE_MASK      0x08
#define ST25DV_GPO_DYN_RFPUTMSG_SHIFT        (4)
#define ST25DV_GPO_DYN_RFPUTMSG_FIELD        0xEF
#define ST25DV_GPO_DYN_RFPUTMSG_MASK         0x10
#define ST25DV_GPO_DYN_RFGETMSG_SHIFT        (5)
#define ST25DV_GPO_DYN_RFGETMSG_FIELD        0xDF
#define ST25DV_GPO_DYN_RFGETMSG_MASK         0x20
#define ST25DV_GPO_DYN_RFWRITE_SHIFT         (6)
#define ST25DV_GPO_DYN_RFWRITE_FIELD         0xBF
#define ST25DV_GPO_DYN_RFWRITE_MASK          0x40
#define ST25DV_GPO_DYN_ENABLE_SHIFT          (7)
#define ST25DV_GPO_DYN_ENABLE_FIELD          0x7F
#define ST25DV_GPO_DYN_ENABLE_MASK           0x80
#define ST25DV_GPO_DYN_ALL_MASK              0xFF

/* ITTIME */
#define ST25DV_ITTIME_DELAY_SHIFT            (0)
#define ST25DV_ITTIME_DELAY_FIELD            0xFC
#define ST25DV_ITTIME_DELAY_MASK             0x03

/* ITSTS_Dyn */
#define ST25DV_ITSTS_RFUSERSTATE_SHIFT       (0)
#define ST25DV_ITSTS_RFUSERSTATE_FIELD       0xFE
#define ST25DV_ITSTS_RFUSERSTATE_MASK        0x01
#define ST25DV_ITSTS_RFACTIVITY_SHIFT        (1)
#define ST25DV_ITSTS_RFACTIVITY_FIELD        0xFD
#define ST25DV_ITSTS_RFACTIVITY_MASK         0x02
#define ST25DV_ITSTS_RFINTERRUPT_SHIFT       (2)
#define ST25DV_ITSTS_RFINTERRUPT_FIELD       0xFB
#define ST25DV_ITSTS_RFINTERRUPT_MASK        0x04
#define ST25DV_ITSTS_FIELDFALLING_SHIFT      (3)
#define ST25DV_ITSTS_FIELDFALLING_FIELD      0xF7
#define ST25DV_ITSTS_FIELDFALLING_MASK       0x08
#define ST25DV_ITSTS_FIELDRISING_SHIFT       (4)
#define ST25DV_ITSTS_FIELDRISING_FIELD       0xEF
#define ST25DV_ITSTS_FIELDRISING_MASK        0x10
#define ST25DV_ITSTS_RFPUTMSG_SHIFT          (5)
#define ST25DV_ITSTS_RFPUTMSG_FIELD          0xDF
#define ST25DV_ITSTS_RFPUTMSG_MASK           0x20
#define ST25DV_ITSTS_RFGETMSG_SHIFT          (6)
#define ST25DV_ITSTS_RFGETMSG_FIELD          0xBF
#define ST25DV_ITSTS_RFGETMSG_MASK           0x40
#define ST25DV_ITSTS_RFWRITE_SHIFT           (7)
#define ST25DV_ITSTS_RFWRITE_FIELD           0x7F
#define ST25DV_ITSTS_RFWRITE_MASK            0x80

/* EH_MODE */
#define ST25DV_EH_MODE_SHIFT                 (0)
#define ST25DV_EH_MODE_FIELD                 0xFE
#define ST25DV_EH_MODE_MASK                  0x01

/* EH_CTRL_Dyn */
#define ST25DV_EH_CTRL_DYN_EH_EN_SHIFT       (0)
#define ST25DV_EH_CTRL_DYN_EH_EN_FIELD       0xFE
#define ST25DV_EH_CTRL_DYN_EH_EN_MASK        0x01
#define ST25DV_EH_CTRL_DYN_EH_ON_SHIFT       (1)
#define ST25DV_EH_CTRL_DYN_EH_ON_FIELD       0xFD
#define ST25DV_EH_CTRL_DYN_EH_ON_MASK        0x02
#define ST25DV_EH_CTRL_DYN_FIELD_ON_SHIFT    (2)
#define ST25DV_EH_CTRL_DYN_FIELD_ON_FIELD    0xFB
#define ST25DV_EH_CTRL_DYN_FIELD_ON_MASK     0x04
#define ST25DV_EH_CTRL_DYN_VCC_ON_SHIFT      (3)
#define ST25DV_EH_CTRL_DYN_VCC_ON_FIELD      0xF7
#define ST25DV_EH_CTRL_DYN_VCC_ON_MASK       0x08

/* RF_MNGT */
#define ST25DV_RF_MNGT_RFDIS_SHIFT           (0)
#define ST25DV_RF_MNGT_RFDIS_FIELD           0xFE
#define ST25DV_RF_MNGT_RFDIS_MASK            0x01
#define ST25DV_RF_MNGT_RFSLEEP_SHIFT         (1)
#define ST25DV_RF_MNGT_RFSLEEP_FIELD         0xFD
#define ST25DV_RF_MNGT_RFSLEEP_MASK          0x02

/* RF_MNGT_Dyn */
#define ST25DV_RF_MNGT_DYN_RFDIS_SHIFT       (0)
#define ST25DV_RF_MNGT_DYN_RFDIS_FIELD       0xFE
#define ST25DV_RF_MNGT_DYN_RFDIS_MASK        0x01
#define ST25DV_RF_MNGT_DYN_RFSLEEP_SHIFT     (1)
#define ST25DV_RF_MNGT_DYN_RFSLEEP_FIELD     0xFD
#define ST25DV_RF_MNGT_DYN_RFSLEEP_MASK      0x02

/* RFZSS */
#define ST25DV_RFZSS_PWDCTRL_SHIFT           (0)
#define ST25DV_RFZSS_PWDCTRL_FIELD           0xFC
#define ST25DV_RFZSS_PWDCTRL_MASK            0x03
#define ST25DV_RFZSS_RWPROT_SHIFT            (2)
#define ST25DV_RFZSS_RWPROT_FIELD            0xF3
#define ST25DV_RFZSS_RWPROT_MASK             0x0C

/* I2CZSS */
#define ST25DV_I2CZSS_PZ1_SHIFT              (0)
#define ST25DV_I2CZSS_PZ1_FIELD              0xFC
#define ST25DV_I2CZSS_PZ1_MASK               0x03
#define ST25DV_I2CZSS_PZ2_SHIFT              (2)
#define ST25DV_I2CZSS_PZ2_FIELD              0xF3
#define ST25DV_I2CZSS_PZ2_MASK               0x0C
#define ST25DV_I2CZSS_PZ3_SHIFT              (4)
#define ST25DV_I2CZSS_PZ3_FIELD              0xCF
#define ST25DV_I2CZSS_PZ3_MASK               0x30
#define ST25DV_I2CZSS_PZ4_SHIFT              (6)
#define ST25DV_I2CZSS_PZ4_FIELD              0x3F
#define ST25DV_I2CZSS_PZ4_MASK               0xC0

/* LOCKCCFILE */
#define ST25DV_LOCKCCFILE_BLCK0_SHIFT        (0)
#define ST25DV_LOCKCCFILE_BLCK0_FIELD        0xFE
#define ST25DV_LOCKCCFILE_BLCK0_MASK         0x01
#define ST25DV_LOCKCCFILE_BLCK1_SHIFT        (1)
#define ST25DV_LOCKCCFILE_BLCK1_FIELD        0xFD
#define ST25DV_LOCKCCFILE_BLCK1_MASK         0x02

/* LOCKCFG */
#define ST25DV_LOCKCFG_B0_SHIFT              (0)
#define ST25DV_LOCKCFG_B0_FIELD              0xFE
#define ST25DV_LOCKCFG_B0_MASK               0x01

/* I2C_SSO_Dyn */
#define ST25DV_I2C_SSO_DYN_I2CSSO_SHIFT      (0)
#define ST25DV_I2C_SSO_DYN_I2CSSO_FIELD      0xFE
#define ST25DV_I2C_SSO_DYN_I2CSSO_MASK       0x01



/* External variables --------------------------------------------------------*/
/* NFCTAG driver structure */
extern NFCTAG_DrvTypeDef St25Dv_i2c_Drv;
extern NFCTAG_ExtDrvTypeDef St25Dv_i2c_ExtDrv;

/* Exported macro ------------------------------------------------------------*/
/* Imported functions ------------------------------------------------------- */
extern NFCTAG_StatusTypeDef ST25DV_IO_Init(void);
extern NFCTAG_StatusTypeDef ST25DV_IO_MemWrite(const uint8_t *const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size);
extern NFCTAG_StatusTypeDef ST25DV_IO_Write(const uint8_t *const pData, const uint8_t DevAddr, const uint16_t Size);
extern NFCTAG_StatusTypeDef ST25DV_IO_MemRead(uint8_t *const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size);
extern NFCTAG_StatusTypeDef ST25DV_IO_Read(uint8_t *const pData, const uint8_t DevAddr, const uint16_t Size);
extern uint8_t ST25DV_IO_IsNacked(void);
extern NFCTAG_StatusTypeDef ST25DV_IO_IsDeviceReady(const uint8_t DevAddr, const uint32_t Trials);

/* Exported functions ------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif
#endif /* __ST25DV_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

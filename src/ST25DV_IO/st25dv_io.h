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
#ifndef _ST25DV_IO_H_
#define _ST25DV_IO_H_
#include "Arduino.h"
#include <Wire.h>
#include <Stream.h>

#define ST25DV_OK                            0
#define ST25DV_MAX_INSTANCE                  1

/* Exported constants --------------------------------------------------------*/
/** @brief ST25DV 4Kbits */
#define I_AM_ST25DV04                        0x24
/** @brief ST25DV 64Kbits */
#define I_AM_ST25DV64                        0x26
/** @brief ST25DV 4Kbits */
#define I_AM_ST25DV04KC                      0x50
/** @brief ST25DV 64Kbits */
#define I_AM_ST25DV64KC                      0x51

#ifndef NULL
#define NULL      (void *) 0
#endif

#define NFCTAG_4K_SIZE            ((uint32_t) 0x200)
#define NFCTAG_16K_SIZE           ((uint32_t) 0x800)
#define NFCTAG_64K_SIZE           ((uint32_t) 0x2000)

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
/** @brief ST25DVxxKC GPO1 register address. */
#define ST25DVXXKC_GPO1_REG                  0x0000
/** @brief ST25DVxxKC GPO2 register address. */
#define ST25DVXXKC_GPO2_REG                  0x0001
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
#define ST25DV_MB_LEN_DYN_REG                 0x2007
/** @brief For backward compatibility. */
#define ST25DV_MBLEN_DYN_REG _Pragma ("GCC warning \"'ST25DV_MBLEN_DYN_REG' definition is deprecated use 'ST25DV_MB_LEN_DYN_REG' instead\"") ST25DV_MB_LEN_DYN_REG
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
#define ST25DV_MB_WDG_DELAY_RW_SHIFT         (1)
#define ST25DV_MB_WDG_DELAY_FIELD            0xF8
#define ST25DV_MB_WDG_DELAY_MASK             0x0E

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
#define ST25DVXXKC_RF_MNGT_DYN_RFOFF_SHIFT   (uint8_t)(2)
#define ST25DVXXKC_RF_MNGT_DYN_RFOFF_FIELD   (uint8_t)0xFB
#define ST25DVXXKC_RF_MNGT_DYN_RFOFF_MASK    (uint8_t)0x04
#define ST25DVXXKC_RF_MNGT_DYN_ALL_SHIFT     (uint8_t)(0)
#define ST25DVXXKC_RF_MNGT_DYN_ALL_MASK      (uint8_t)0x07

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

/**
 * @brief  ST25DV Ack Nack enumerator definition
 */
typedef enum {
    I2CANSW_ACK = 0,
    I2CANSW_NACK
} ST25DV_I2CANSW_E;

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


#if defined(ARDUINO_SAM_DUE)
#define WIRE Wire1
#else
#define WIRE Wire
#endif

class ST25DV_IO {
public:
    ST25DV_IO(int32_t gpo, int32_t ldp, TwoWire *i2c, Stream *serial = NULL);

    NFCTAG_StatusTypeDef ST25DV_i2c_Init(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadID(uint8_t *const pICRef);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadICRev(uint8_t *const pICRev);
    NFCTAG_StatusTypeDef ST25DV_i2c_IsDeviceReady(const uint32_t Trials);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetGPOStatus(uint16_t *const pGPOStatus);
    NFCTAG_StatusTypeDef ST25DV_i2c_ConfigureGPO(const uint16_t ITConf);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadITPulse(ST25DV_PULSE_DURATION *const pITtime);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteITPulse(const ST25DV_PULSE_DURATION ITtime);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadData(uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteData(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadDataCurrentAddr(uint8_t *const pData, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadRegister(uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteRegister(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadUID(ST25DV_UID *const pUid);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadDSFID(uint8_t *const pDsfid);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadDsfidRFProtection(ST25DV_LOCK_STATUS *const pLockDsfid);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadAFI(uint8_t *const pAfi);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadAfiRFProtection(ST25DV_LOCK_STATUS *const pLockAfi);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadI2CProtectZone(ST25DV_I2C_PROT_ZONE *const pProtZone);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteI2CProtectZonex(const ST25DV_PROTECTION_ZONE Zone, const ST25DV_PROTECTION_CONF ReadWriteProtection);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadLockCCFile(ST25DV_LOCK_CCFILE *const pLockCCFile);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteLockCCFile(const ST25DV_CCFILE_BLOCK NbBlockCCFile, const ST25DV_LOCK_STATUS LockCCFile);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadLockCFG(ST25DV_LOCK_STATUS *const pLockCfg);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteLockCFG(const ST25DV_LOCK_STATUS LockCfg);
    NFCTAG_StatusTypeDef ST25DV_i2c_PresentI2CPassword(const ST25DV_PASSWD PassWord);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteI2CPassword(const ST25DV_PASSWD PassWord);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadRFZxSS(const ST25DV_PROTECTION_ZONE Zone, ST25DV_RF_PROT_ZONE *const pRfprotZone);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteRFZxSS(const ST25DV_PROTECTION_ZONE Zone, const ST25DV_RF_PROT_ZONE RfProtZone);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadEndZonex(const ST25DV_END_ZONE EndZone, uint8_t *const pEndZ);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteEndZonex(const ST25DV_END_ZONE EndZone, const uint8_t EndZ);
    NFCTAG_StatusTypeDef ST25DV_i2c_InitEndZone(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_CreateUserZone(uint16_t Zone1Length, uint16_t Zone2Length, uint16_t Zone3Length, uint16_t Zone4Length);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadMemSize(ST25DV_MEM_SIZE *const pSizeInfo);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadEHMode(ST25DV_EH_MODE_STATUS *const pEH_mode);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteEHMode(const ST25DV_EH_MODE_STATUS EH_mode);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadRFMngt(ST25DV_RF_MNGT *const pRF_Mngt);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteRFMngt(const uint8_t Rfmngt);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetRFDisable(ST25DV_EN_STATUS *const pRFDisable);
    NFCTAG_StatusTypeDef ST25DV_i2c_SetRFDisable(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ResetRFDisable(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetRFSleep(ST25DV_EN_STATUS *const pRFSleep);
    NFCTAG_StatusTypeDef ST25DV_i2c_SetRFSleep(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ResetRFSleep(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadMBMode(ST25DV_EN_STATUS *const pMB_mode);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteMBMode(const ST25DV_EN_STATUS MB_mode);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadMBWDG(uint8_t *const pWdgDelay);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteMBWDG(const uint8_t WdgDelay);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadMailboxData(uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteMailboxData(const uint8_t *const pData, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadMailboxRegister(uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteMailboxRegister(const uint8_t *const pData, const uint16_t TarAddr, const uint16_t NbByte);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadI2CSecuritySession_Dyn(ST25DV_I2CSSO_STATUS *const pSession);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadITSTStatus_Dyn(uint8_t *const pITStatus);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadGPO_Dyn(uint8_t *GPOConfig);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetGPO_en_Dyn(ST25DV_EN_STATUS *const pGPO_en);
    NFCTAG_StatusTypeDef ST25DV_i2c_SetGPO_en_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ResetGPO_en_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadEHCtrl_Dyn(ST25DV_EH_CTRL *const pEH_CTRL);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetEHENMode_Dyn(ST25DV_EN_STATUS *const pEH_Val);
    NFCTAG_StatusTypeDef ST25DV_i2c_SetEHENMode_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ResetEHENMode_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetEHON_Dyn(ST25DV_EN_STATUS *const pEHON);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetRFField_Dyn(ST25DV_FIELD_STATUS *const pRF_Field);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetVCC_Dyn(ST25DV_VCC_STATUS *const pVCC);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadRFMngt_Dyn(ST25DV_RF_MNGT *const pRF_Mngt);
    NFCTAG_StatusTypeDef ST25DV_i2c_WriteRFMngt_Dyn(const uint8_t RF_Mngt);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetRFDisable_Dyn(ST25DV_EN_STATUS *const pRFDisable);
    NFCTAG_StatusTypeDef ST25DV_i2c_SetRFDisable_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ResetRFDisable_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetRFSleep_Dyn(ST25DV_EN_STATUS *const pRFSleep);
    NFCTAG_StatusTypeDef ST25DV_i2c_SetRFSleep_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ResetRFSleep_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadMBCtrl_Dyn(ST25DV_MB_CTRL_DYN_STATUS *const pCtrlStatus);
    NFCTAG_StatusTypeDef ST25DV_i2c_GetMBEN_Dyn(ST25DV_EN_STATUS *const pMBEN);
    NFCTAG_StatusTypeDef ST25DV_i2c_SetMBEN_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ResetMBEN_Dyn(void);
    NFCTAG_StatusTypeDef ST25DV_i2c_ReadMBLength_Dyn(uint8_t *const pMBLength);

    NFCTAG_StatusTypeDef ST25DV_IO_Init(void);
    NFCTAG_StatusTypeDef ST25DV_IO_MemWrite(const uint8_t *const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size);
    NFCTAG_StatusTypeDef ST25DV_IO_MemRead(uint8_t *const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size);
    NFCTAG_StatusTypeDef ST25DV_IO_Read(uint8_t *const pData, const uint8_t DevAddr, const uint16_t Size);
    NFCTAG_StatusTypeDef ST25DV_IO_IsDeviceReady(const uint8_t DevAddr, const uint32_t Trials);
    NFCTAG_StatusTypeDef NFCTAG_ConvertStatus(uint8_t ret);

    int32_t get_gpo();
    int32_t get_lpd();
    TwoWire *get_pwire();
    Stream *get_pserial();

protected:
    int32_t _gpo;
    int32_t _lpd;
    TwoWire *_pwire;
    Stream *_serial;
};

#endif

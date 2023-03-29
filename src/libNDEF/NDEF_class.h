#ifndef _NDEFCLASS_H_
#define _NDEFCLASS_H_
#include "libNDEF/NDEFcommon.h"
#include "libNDEF/tagtype5_wrapper.h"
#include "libNDEF/lib_NDEF_URI.h"
#include "libNDEF/lib_NDEF_AAR.h"
#include "libNDEF/lib_NDEF_Bluetooth.h"
#include "libNDEF/lib_NDEF_Email.h"
#include "libNDEF/lib_NDEF_Handover.h"
#include "libNDEF/lib_NDEF_Geo.h"
#include "libNDEF/lib_NDEF_MyApp.h"
#include "libNDEF/lib_NDEF_SMS.h"
#include "libNDEF/lib_NDEF_Text.h"
#include "libNDEF/lib_NDEF_Vcard.h"
#include "libNDEF/lib_NDEF_Wifi.h"
#include "ST25DV_IO/st25dv_io.h"

/**
  * @brief  Tag Type 5 State enumeration definition.
  */
typedef enum {
  TT5_NO_NDEF = 0,  /**< No data detected in the tag. */
  TT5_INITIALIZED,  /**< Capability container detected. */
  TT5_READ_WRITE,   /**< Read-Write data detected. */
  TT5_READ          /**< Read-Only data message detected. */
} TT5_State;

/** @brief Type5 Tag Capability Container Magic numbers as defined by the NFC Forum. */
typedef enum {
  NFCT5_MAGICNUMBER_E1_CCFILE = 0xE1, /**<  Complete data area can be read by 1-byte block adrdess commands. */
  NFCT5_MAGICNUMBER_E2_CCFILE = 0xE2  /**<  Last part of the data area can be only read by 2-bytes block address commands.\n
                                            The first 256 blocks can be read by 1-byte block address commands. */
} TT5_MagicNumber_t;

/**
  * @brief  Type5 Tag Capability Container structure.
  */
typedef struct {
  TT5_MagicNumber_t MagicNumber;  /**< CCfile[0]: Magic Number should be E1h or E2h (for extended API) */
  uint8_t Version;                /**< CCfile[1]: Capability container version (b7-b4) and access conditions (b3-b0) */
  uint8_t MemorySize;             /**< CCfile[2]: Memory size, expressed in 8 bytes blocks, set to 0 if tag size is greater than 16kbits. */
  uint8_t TT5Tag;                 /**< CCfile[3]: Additional information on the Type5 Tag:\n
                                                  b0: supports `read multiple block` commands\n
                                                  b1: RFU\n
                                                  b2: RFU\n
                                                  b3: supports `lock block` commands\n
                                                  b4: requires the `special frame` format
                                    */
  uint8_t rsved1;                 /**< RFU */
  uint8_t rsved2;                 /**< RFU */
  uint16_t ExtMemorySize;         /**< CCfile[6],CCfile[7]: Memory size, expressed in 8 bytes blocks, when tag size is greater than 16kbits. */
  TT5_State State;                /**< Indicates if a NDEF message is present. */
  uint32_t NDEF_offset;           /**< Indicates the address of a NDEF message in the tag. */
} sCCFileInfo;

class NDEF {
  public:
    NDEF();

    uint16_t begin(ST25DV_IO *dev);

    //lib_NDEF
    uint16_t NDEF_IdentifyNDEF(sRecordInfo_t *pRecordStruct, uint8_t *pNDEF);
    uint16_t NDEF_IdentifyBuffer(sRecordInfo_t *pRecordStruct, uint8_t *pNDEF);
    uint16_t NDEF_ReadNDEF(uint8_t *pNDEF);
    uint16_t NDEF_WriteNDEF(uint16_t NDEF_Size, uint8_t *pNDEF);
    uint16_t NDEF_ClearNDEF(void);
    uint16_t NDEF_getNDEFSize(uint16_t *Size);
    uint32_t NDEF_WriteRecord(sRecordInfo_t *pRecord, uint8_t *pNDEF);
    uint16_t NDEF_AppendRecord(sRecordInfo_t  *Record);
    uint32_t NDEF_GetRecordLength(sRecordInfo_t *pRecord);

    //lib_NDEF_AAR
    uint16_t NDEF_AddAAR(const sAARInfo *pAARStruct);

    //lib_NDEF_Bluetooth
    uint8_t *NDEF_BluetoothCopy(uint8_t *dst, uint8_t *src, uint32_t length);
    uint16_t NDEF_ReadBluetoothOOB(sRecordInfo_t *pRecordStruct, Ndef_Bluetooth_OOB_t *pBluetooth);
    uint16_t NDEF_AppendBluetoothOOB(Ndef_Bluetooth_OOB_t *pBluetooth, char *RecordID);
    void NDEF_PrepareBluetoothMessage(Ndef_Bluetooth_OOB_t  *pBluetooth, uint8_t *pNDEFMessage, uint16_t *size);
    uint32_t NDEF_GetBluetoothOOBLength(Ndef_Bluetooth_OOB_t *pBluetooth);

    //lib_NDEF_Email
    uint16_t NDEF_ReadEmail(sRecordInfo_t *pRecordStruct, sEmailInfo *pEmailStruct);
    uint16_t NDEF_WriteEmail(sEmailInfo *pEmailStruct);
    void NDEF_PrepareEmailMessage(sEmailInfo *pEmailStruct, uint8_t *pNDEFMessage, uint16_t *size);
    void NDEF_closeEmail(sEmailInfo *pEmailStruct);

    //lib_NDEF_Geo
    uint16_t NDEF_ReadGeo(sRecordInfo_t *pRecordStruct, sGeoInfo *pGeoStruct);
    uint16_t NDEF_WriteGeo(sGeoInfo *pGeoStruct);
    void NDEF_PrepareGeoMessage(sGeoInfo *pGeoStruct, uint8_t *pNDEFMessage, uint16_t *size);

    //lib_NDEF_Handover
    uint16_t NDEF_ReadHandover(sRecordInfo_t *pRecord,  Ndef_Handover_t *pHandover);
    uint16_t NDEF_ReadAC(uint8_t ac_nb, Ndef_Handover_t *pHandover, Ndef_Handover_alternative_carrier_t *pAC);
    uint16_t NDEF_ReadAuxData(uint8_t aux_data_nb, Ndef_Handover_alternative_carrier_t *pAC, sRecordInfo_t *pRecord);
    uint16_t NDEF_CreateHandover(Ndef_Handover_t  *pHandover, sRecordInfo_t *pRecord);
    uint16_t NDEF_AddAlternativeCarrier(Ndef_Handover_alternative_carrier_t *pAC, char *CarrierDataRef, char **AuxDataRefID, sRecordInfo_t *pRecord);
    uint16_t NDEF_WriteHandover(sRecordInfo_t *pRecord, uint8_t *pNdef);
    uint32_t NDEF_GetACDataLength(Ndef_Handover_alternative_carrier_t *pAC, char *CarrierDataRef, char **AuxDataRefID);

    //lib_NDEF_MyApp
    uint16_t NDEF_ReadMyApp(sRecordInfo_t *pRecordStruct, sMyAppInfo *pMyAppStruct);
    uint16_t NDEF_WriteMyApp(sMyAppInfo *pMyAppStruct);

    //lib_NDEF_SMS
    uint16_t NDEF_ReadSMS(sRecordInfo_t *pRecordStruct, sSMSInfo *pSMSStruct);
    uint16_t NDEF_WriteSMS(sSMSInfo *pSMSStruct);
    void NDEF_PrepareSMSMessage(sSMSInfo *pSMSStruct, uint8_t *pNDEFMessage, uint16_t *size);

    //lib_NDEF_Text
    uint16_t NDEF_WriteText(char *text);
    uint16_t NDEF_ReadText(sRecordInfo_t *pRecordStruct, NDEF_Text_info_t *pText);

    //lib_NDEF_URI
    uint16_t NDEF_ReadURI(sRecordInfo_t *pRecordStruct, sURI_Info *pURI);
    uint16_t NDEF_WriteURI(sURI_Info *pURI);
    void NDEF_PrepareURIMessage(sURI_Info *pURI, uint8_t *pNDEFMessage, uint16_t *size);
    char getUriType(char *protocol);

    //lib_NDEF_Vcard
    uint16_t NDEF_ReadVcard(sRecordInfo_t *pRecordStruct, sVcardInfo *pVcardStruct);
    uint16_t NDEF_WriteVcard(sVcardInfo *pVcardStruct);
    void NDEF_PrepareVcardMessage(sVcardInfo *pVcardStruct, uint8_t *pNDEFMessage, uint16_t *size);
    int NDEF_getVcardPicture(uint8_t *pPayload, uint32_t PayloadSize,  uint8_t *pPict);

    //lib_NDEF_Wifi
    uint16_t NDEF_ReadWifiToken(struct sRecordInfo *pRecordStruct, sWifiTokenInfo *pWifiTokenStruct);
    uint16_t NDEF_WriteWifiToken(sWifiTokenInfo *pWifiTokenStruct);

    //lib_wrapper
    uint16_t NfcTag_ReadNDEF(uint8_t *pData);
    uint16_t NfcTag_WriteNDEF(uint16_t Length, uint8_t *pData);
    uint16_t NfcTag_WriteProprietary(uint16_t Length, uint8_t *pData);
    uint16_t NfcTag_GetLength(uint16_t *Length);

    //tagtype5_wrapper
    /* Exported functions ------------------------------------------------------- */
    uint16_t NfcType5_WriteCCFile(const uint8_t *const pCCBuffer);
    uint16_t NfcType5_ReadCCFile(uint8_t *const pCCBuffer);
    uint16_t NfcTag_WriteData(uint8_t Type, uint16_t Length, uint8_t *pData);
    uint16_t NfcType5_TT5Init(void);
    uint16_t NfcType5_NDEFDetection(void);

    //libNDEF.c static functions
    uint16_t NDEF_IsNDEFPresent(void);
    uint16_t NDEF_ParseRecordHeader(sRecordInfo_t *pRecordStruct);
    void NDEF_ParseWellKnownType(sRecordInfo_t *pRecordStruct);
    void NDEF_ParseMediaType(sRecordInfo_t *pRecordStruct);
    void NDEF_ParseForumExternalType(sRecordInfo_t *pRecordStruct);
    void NDEF_ParseURI(sRecordInfo_t *pRecordStruct);
    void NDEF_ParseSP(sRecordInfo_t *pRecordStruct);
    uint16_t NDEF_IdentifySPRecord(sRecordInfo_t *pRecordStruct, uint8_t *pPayload);

    //Email static
    void NDEF_FillEmailStruct(uint8_t *pPayload, uint32_t PayloadSize, sEmailInfo *pEmailStruct);
    void NDEF_ReadURI_Email(sRecordInfo_t *pRecordStruct, sEmailInfo *pEmailStruct);
  private:

    //Geo static
    void NDEF_FillGeoStruct(uint8_t *pPayload, uint32_t PayloadSize, sGeoInfo *pGeoStruct);
    void NDEF_ReadURI_Geo(sRecordInfo_t *pRecordStruct, sGeoInfo *pGeoStruct);

    //MyApp static
    void NDEF_Extract_M24SRDiscoveryApp_Input(sRecordInfo_t *pRecordStruct, sMyAppInfo *pMyAppStruct);

    //SMS static
    void NDEF_FillSMSStruct(uint8_t *pPayload, uint32_t PayloadSize, sSMSInfo *pSMSStruct);
    void NDEF_ReadURI_SMS(sRecordInfo_t *pRecordStruct, sSMSInfo *pSMSStruct);

    //URI static
    void NDEF_Parse_WellKnowType(sRecordInfo_t *pRecordStruct, sURI_Info *pURI);


    //Vcard static
    void NDEF_FillVcardStruct(uint8_t *pPayload, uint32_t PayloadSize, char *pKeyWord, uint32_t SizeOfKeyWord, uint8_t *pString);
    void NDEF_ExtractVcard(sRecordInfo_t *pRecordStruct, sVcardInfo *pVcardStruct);

    //Wifi static
    void NDEF_FillWifiTokenStruct(uint8_t *pPayload, uint32_t PayloadSize, sWifiTokenInfo *pWifiTokenStruct);
    void NDEF_Read_WifiToken(struct sRecordInfo *pRecordStruct, sWifiTokenInfo *pWifiTokenStruct);

    //libNDEF.c
    /** @brief This buffer is used to store the data sent/received by the TAG. */
    uint8_t NDEF_Buffer [NDEF_MAX_SIZE];
    /** @brief Size of the buffer used to build the NDEF messages. */
    uint32_t NDEF_Buffer_size = NDEF_MAX_SIZE;
    /** @brief This buffer is used when it's required to prepare a record before adding it to the NDEF_Buffer. */
    uint8_t NDEF_Record_Buffer [NDEF_RECORD_MAX_SIZE];
    /** @brief Size of the buffer used when a record has to be prepared. */
    uint32_t NDEF_Record_Buffer_size = NDEF_RECORD_MAX_SIZE;

    /* In case of smart Poster composed with different record, 3 records supported so far */
    sRecordInfo_t SPRecordStruct1;
    sRecordInfo_t SPRecordStruct2;
    sRecordInfo_t SPRecordStruct3;
    sRecordInfo_t SPRecordStruct4;
    sRecordInfo_t *SPRecordStructAdd[SP_MAX_RECORD];


    //NDEF_AAR
    /**
     * @brief  This buffer contains the data send/received by TAG
     */
    //extern uint8_t NDEF_Buffer [];

    //tagtype5_wrapper
    /** @brief Capability Container structure instance (global). */
    sCCFileInfo CCFileStruct;

  private:

    ST25DV_IO *mydev;
};
#endif /* _NDEF_CLASS_H */

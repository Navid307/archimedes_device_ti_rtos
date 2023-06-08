/*
 * arch_ble_pressure_service.c
 *
 *  Created on: Jun 7, 2023
 *      Author: navid
 */

#include <string.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log

#include <icall.h>
#include "icall_ble_api.h"

#include "arch_ble_pressure_service.h"

// Button_Service Service UUID
CONST uint8_t PressureServiceUUID[ATT_UUID_SIZE] = {
        ARCH_BLE_PRESSURE_SERVICE_UUID_BASE128(
                ARCH_BLE_PRESSURE_SERVICE_SERV_UUID)
};

CONST uint8_t ps_BEEFUUID[ATT_UUID_SIZE] =
        { PS_BEEF_UUID_BASE128(PS_BEEF_UUID) }; // TODO: 128 UUID for three characteristics

CONST uint8_t ps_CAFEUUID[ATT_UUID_SIZE] =
        { PS_CAFE_UUID_BASE128(PS_CAFE_UUID) }; // TODO: 128 UUID for three characteristics

CONST uint8_t ps_FACEUUID[ATT_UUID_SIZE] =
        { PS_FACE_UUID_BASE128(PS_FACE_UUID) }; // TODO: 128 UUID for three characteristics

static PressureServiceCBs_t *pAppCBs = NULL;
static uint8_t ps_icall_rsp_task_id = INVALID_TASK_ID;

// Service declaration
static CONST gattAttrType_t PressureServiceDecl = { ATT_UUID_SIZE,
                                                    PressureServiceUUID };

// TODO: rename the variables
static uint8_t ps_BEEFProps = GATT_PROP_NOTIFY;
static uint8_t ps_BEEFVal[PS_BEEF_LEN] = { 0 };
static uint16_t ps_BEEFValLen = PS_BEEF_LEN_MIN;
static gattCharCfg_t *ps_BEEFConfig;

static uint8_t ps_CAFEProps = GATT_PROP_WRITE;
static uint8_t ps_CAFEVal[PS_CAFE_LEN] = { 0 };
static uint16_t ps_CAFEValLen = PS_CAFE_LEN_MIN;
static gattCharCfg_t *ps_CAFEConfig;

static uint8_t ps_FACEProps = GATT_PROP_INDICATE;
static uint8_t ps_FACEVal[PS_FACE_LEN] = { 0 };
static uint16_t ps_FACValLen = PS_FACE_LEN_MIN;
static gattCharCfg_t *ps_FACEConfig;

static gattAttribute_t Pressure_ServiceAttrTbl[] = {
// Pressure_Service Service Declaration
        { { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
          0, (uint8_t*) &PressureServiceDecl },

        { { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
          0, &ps_BEEFProps },

        { { ATT_UUID_SIZE, ps_BEEFUUID },
        GATT_PERMIT_READ,
          0, ps_BEEFVal },

        { { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
          0, (uint8_t*) &ps_BEEFConfig },
//
        { { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
          0, &ps_CAFEProps },

        { { ATT_UUID_SIZE, ps_CAFEUUID },
        GATT_PERMIT_READ,
          0, ps_CAFEVal },

        { { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
          0, (uint8_t*) &ps_CAFEConfig },
//
        { { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
          0, &ps_FACEProps },

        { { ATT_UUID_SIZE, ps_FACEUUID },
        GATT_PERMIT_READ,
          0, ps_FACEVal },

        { { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
          0, (uint8_t*) &ps_FACEConfig }, };

static bStatus_t Pressure_Service_ReadAttrCB(uint16_t connHandle,
                                             gattAttribute_t *pAttr,
                                             uint8_t *pValue, uint16_t *pLen,
                                             uint16_t offset, uint16_t maxLen,
                                             uint8_t method);
static bStatus_t Pressure_Service_WriteAttrCB(uint16_t connHandle,
                                              gattAttribute_t *pAttr,
                                              uint8_t *pValue, uint16_t len,
                                              uint16_t offset, uint8_t method);

CONST gattServiceCBs_t Pressure_ServiceCBs = { Pressure_Service_ReadAttrCB, // Read callback function pointer
        Pressure_Service_WriteAttrCB, // Write callback function pointer
        NULL                     // Authorization callback function pointer
        };

extern bStatus_t PressureService_AddService(uint8_t rspTaskId)
{
    uint8_t status;

    // Allocate Client Characteristic Configuration table
    ps_BEEFConfig = (gattCharCfg_t*) ICall_malloc(
            sizeof(gattCharCfg_t) * linkDBNumConns);
    if (ps_BEEFConfig == NULL)
    {
        return (bleMemAllocError);
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(LINKDB_CONNHANDLE_INVALID, ps_BEEFConfig);

    // Allocate Client Characteristic Configuration table
    ps_CAFEConfig = (gattCharCfg_t*) ICall_malloc(
            sizeof(gattCharCfg_t) * linkDBNumConns);
    if (ps_CAFEConfig == NULL)
    {
        return (bleMemAllocError);
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(LINKDB_CONNHANDLE_INVALID, ps_CAFEConfig);

    // Allocate Client Characteristic Configuration table
    ps_FACEConfig = (gattCharCfg_t*) ICall_malloc(
            sizeof(gattCharCfg_t) * linkDBNumConns);
    if (ps_FACEConfig == NULL)
    {
        return (bleMemAllocError);
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(LINKDB_CONNHANDLE_INVALID, ps_FACEConfig);

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(Pressure_ServiceAttrTbl,
                                         GATT_NUM_ATTRS(Pressure_ServiceAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &Pressure_ServiceCBs);
    Log_info1("Registered service, %d attributes",
              GATT_NUM_ATTRS(Pressure_ServiceAttrTbl));
    ps_icall_rsp_task_id = rspTaskId;

    return (status);
}

bStatus_t PressureService_RegisterAppCBs(PressureServiceCBs_t *appCallbacks)
{
    if (appCallbacks)
    {
        pAppCBs = appCallbacks;
        Log_info1("Registered callbacks to application. Struct %p",
                  (uintptr_t )appCallbacks);
        return (SUCCESS);
    }
    else
    {
        Log_warning0("Null pointer given for app callbacks.");
        return (FAILURE);
    }
}

bStatus_t PressureService_SetParameter(uint8_t param, uint16_t len, void *value)
{
    bStatus_t ret = SUCCESS;
    uint8_t *pAttrVal;
    uint16_t *pValLen;
    uint16_t valMinLen;
    uint16_t valMaxLen;
    uint8_t sendNotiInd = FALSE;
    gattCharCfg_t *attrConfig;
    uint8_t needAuth;

    switch (param)
    {
    case 0:
        Log_error1("First ID", param);
        break;

    case 1:
        Log_error1("First ID", param);
        break;

    default:
        Log_error1("SetParameter: Parameter #%d not valid.", param);
        return (INVALIDPARAMETER);
    }

    // Check bounds, update value and send notification or indication if possible.
    if (len <= valMaxLen && len >= valMinLen)
    {
        memcpy(pAttrVal, value, len);
        *pValLen = len; // Update length for read and get.

        if (sendNotiInd)
        {
            Log_info2(
                    "Trying to send noti/ind: connHandle %x, %s",
                    attrConfig[0].connHandle,
                    (uintptr_t)(
                            (attrConfig[0].value == 0) ?
                                    "\x1b[33mNoti/ind disabled\x1b[0m" :
                            (attrConfig[0].value == 1) ?
                                    "Notification enabled" :
                                    "Indication enabled"));
            // Try to send notification.
            GATTServApp_ProcessCharCfg(attrConfig, pAttrVal, needAuth,
                                       Pressure_ServiceAttrTbl,
                                       GATT_NUM_ATTRS(Pressure_ServiceAttrTbl),
                                       ps_icall_rsp_task_id,
                                       Pressure_Service_ReadAttrCB);
        }
    }
    else
    {
        Log_error3("Length outside bounds: Len: %d MinLen: %d MaxLen: %d.", len,
                   valMinLen, valMaxLen);
        ret = bleInvalidRange;
    }

    return (ret);
}

bStatus_t PressureService_GetParameter(uint8_t param, uint16_t *len,
                                       void *value)
{
    bStatus_t ret = SUCCESS;
    switch (param)
    {
    default:
        Log_error1("GetParameter: Parameter #%d not valid.", param);
        ret = INVALIDPARAMETER;
        break;
    }
    return (ret);
}

static uint8_t Pressure_Service_findCharParamId(gattAttribute_t *pAttr)
{
    // Is this a Client Characteristic Configuration Descriptor?
    if (ATT_BT_UUID_SIZE == pAttr->type.len
            && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t*) pAttr->type.uuid)
    {
        return (Pressure_Service_findCharParamId(pAttr - 1)); // Assume the value attribute precedes CCCD and recurse
    }
    // Is this attribute in "BUTTON0"?
    else if (ATT_UUID_SIZE == pAttr->type.len
            && !memcmp(pAttr->type.uuid, ps_BEEFUUID, pAttr->type.len))
    {
        return (0); // TODO: ??
    }
    // Is this attribute in "BUTTON1"?
    else if (ATT_UUID_SIZE == pAttr->type.len
            && !memcmp(pAttr->type.uuid, ps_CAFEUUID, pAttr->type.len))
    {
        return (1);  // TODO: ??
    }
    // Is this attribute in "BUTTON2"?
    else if (ATT_UUID_SIZE == pAttr->type.len
            && !memcmp(pAttr->type.uuid, ps_FACEUUID, pAttr->type.len))
    {
        return (2);  // TODO: ??
    }

    else
    {
        return (0xFF); // Not found. Return invalid.
    }
}

static bStatus_t Pressure_Service_ReadAttrCB(uint16_t connHandle,
                                             gattAttribute_t *pAttr,
                                             uint8_t *pValue, uint16_t *pLen,
                                             uint16_t offset, uint16_t maxLen,
                                             uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint16_t valueLen;
    uint8_t paramID = 0xFF;

    // Find settings for the characteristic to be read.
    paramID = Pressure_Service_findCharParamId(pAttr);
    switch (paramID)
    {
    case 0:
        valueLen = ps_BEEFValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t )"BUTTON0", connHandle, offset, method);
        /* Other considerations for BUTTON1 can be inserted here */
        break;

    case 1:
        valueLen = ps_CAFEValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t )"BUTTON1", connHandle, offset, method);
        /* Other considerations for BUTTON1 can be inserted here */
        break;

    default:
        Log_error0("Attribute was not found.");
        return (ATT_ERR_ATTR_NOT_FOUND);
    }
    // Check bounds and return the value
    if (offset > valueLen)   // Prevent malicious ATT ReadBlob offsets.
    {
        Log_error0("An invalid offset was requested.");
        status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
        *pLen = MIN(maxLen, valueLen - offset); // Transmit as much as possible
        memcpy(pValue, pAttr->pValue + offset, *pLen);
    }

    return (status);
}

static bStatus_t Pressure_Service_WriteAttrCB(uint16_t connHandle,
                                            gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len,
                                            uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;

    // See if request is regarding a Client Characterisic Configuration
    if (ATT_BT_UUID_SIZE == pAttr->type.len
            && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t*) pAttr->type.uuid)
    {
        Log_info3(
                "WriteAttrCB (CCCD): param: %d connHandle: %d %s",
                Pressure_Service_findCharParamId(pAttr),
                connHandle,
                (uintptr_t)(method == GATT_LOCAL_WRITE ? "- restoring bonded state" : "- OTA write"));

        // Allow notification and indication, but do not check if really allowed per CCCD.
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                offset,
                                                GATT_CLIENT_CFG_NOTIFY |
                                                GATT_CLIENT_CFG_INDICATE);
        if (SUCCESS == status && pAppCBs && pAppCBs->pfnCfgChangeCb)
        {
            pAppCBs->pfnCfgChangeCb(connHandle,
                                    Pressure_Service_findCharParamId(pAttr), len,
                                    pValue);
        }

        return (status);
    }

    // Find settings for the characteristic to be written.
    paramID = Pressure_Service_findCharParamId(pAttr);
    switch (paramID)
    {
    default:
        Log_error0("Attribute was not found.");
        return (ATT_ERR_ATTR_NOT_FOUND);
    }
}


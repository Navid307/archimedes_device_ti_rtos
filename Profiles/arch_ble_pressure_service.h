/*
 * arch_ble_pressure_service.h
 *
 *  Created on: Jun 7, 2023
 *      Author: navid
 */

#ifndef _ARCH_BLE_PRESSURE_SERVICE_H_
#define _ARCH_BLE_PRESSURE_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <bcomdef.h>

#define ARCH_BLE_PRESSURE_SERVICE_SERV_UUID 0xF00D
#define ARCH_BLE_PRESSURE_SERVICE_UUID_BASE128(uuid) 0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, LO_UINT16(uuid), HI_UINT16(uuid), 0x00, 0x00, 0x00

#define ARCH_BLE_PRESSURE_SERVICE_NOTIFY_CHAR_UUID 0xBEEF
#define ARCH_BLE_PRESSURE_SERVICE_WRITE_CHAR_UUID 0xCAFE
#define ARCH_BLE_PRESSURE_SERVICE_INDICATE_CHAR_UUID 0xFACE

typedef void (*PressureServiceChange_t)(uint16_t connHandle, uint8_t paramID,
                                        uint16_t len, uint8_t *pValue);

typedef struct
{
    PressureServiceChange_t pfnChangeCb;
    PressureServiceChange_t pfnCfgChangeCb;
} PressureServiceCBs_t;

extern pStatus_t PressureService_AddService(uint8_t rspTaskId);
extern pStatus_t PressureService_RegisterAppCBs(
        PressureServiceCBs_t *appCallbacks);
extern pStatus_t PressureService_SetParameter(uint8_t param, uint16_t len,
                                              void *value);

extern pStatus_t PressureService_GetParameter(uint8_t param, uint16_t *len,
                                              void *value);
#ifdef __cplusplus
}
#endif

#endif /* _ARCH_BLE_PRESSURE_SERVICE_H_ */


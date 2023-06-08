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

#include <string.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log

#include <icall.h>
#include "icall_ble_api.h"

#include <bcomdef.h>

// Service UUID
#define PRESSURE_SERVICE_SERV_UUID 0x1120
#define PRESSURE_SERVICE_UUID_BASE128(uuid) 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, LO_UINT16(uuid), HI_UINT16(uuid), \
    0x00, 0xF0

// BEEF Characteristic defines
#define PS_BEEF_ID                 0
#define PS_BEEF_UUID               0x1121
#define PS_BEEF_UUID_BASE128(uuid) 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0xB0, 0x00, 0x40, 0x51, 0x04, LO_UINT16(uuid), HI_UINT16(uuid), 0x00, 0xF0
#define PS_BEEF_LEN                1
#define PS_BEEF_LEN_MIN            1

// CAFE Characteristic defines
#define PS_CAFE_ID                 1
#define PS_CAFE_UUID               0x1122
#define PS_CAFE_UUID_BASE128(uuid) 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0xB0, 0x00, 0x40, 0x51, 0x04, LO_UINT16(uuid), HI_UINT16(uuid), 0x00, 0xF0
#define PS_CAFE_LEN                1
#define PS_CAFE_LEN_MIN            1

// FACE Characteristic defines
#define PS_FACE_ID                 1
#define PS_FACE_UUID               0x1123
#define PS_FACE_UUID_BASE128(uuid) 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0xB0, 0x00, 0x40, 0x51, 0x04, LO_UINT16(uuid), HI_UINT16(uuid), 0x00, 0xF0
#define PS_FACE_LEN                1
#define PS_FACE_LEN_MIN            1

typedef void (*PressureServiceChange_t)(uint16_t connHandle, uint8_t paramID,
                                        uint16_t len, uint8_t *pValue);

typedef struct
{
    PressureServiceChange_t pfnChangeCb;
    PressureServiceChange_t pfnCfgChangeCb;
} PressureServiceCBs_t;

extern bStatus_t PressureService_AddService(uint8_t rspTaskId);
extern bStatus_t PressureService_RegisterAppCBs(
        PressureServiceCBs_t *appCallbacks);
extern bStatus_t PressureService_SetParameter(uint8_t param, uint16_t len,
                                              void *value);

extern bStatus_t PressureService_GetParameter(uint8_t param, uint16_t *len,
                                              void *value);
#ifdef __cplusplus
}
#endif

#endif /* _ARCH_BLE_PRESSURE_SERVICE_H_ */


/*
 * arch_ble_application.h
 *
 *  Created on: Jun 8, 2023
 *      Author: navid
 */

#ifndef APPLICATION_ARCH_BLE_APPLICATION_H_
#define APPLICATION_ARCH_BLE_APPLICATION_H_



#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/GPIO.h>

#include <bcomdef.h>

extern void ArchBleApplication_createTask(void);

#ifdef __cplusplus
}
#endif



#endif /* APPLICATION_ARCH_BLE_APPLICATION_H_ */

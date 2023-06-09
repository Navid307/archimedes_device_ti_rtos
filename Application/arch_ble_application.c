/*
 * arch_ble_application.c
 *
 *  Created on: Jun 8, 2023
 *      Author: navid
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/utils/List.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log

#include <ti/display/AnsiColor.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include <icall.h>
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

/* Bluetooth Profiles */
#include <devinfoservice.h>
#include <profiles/project_zero/button_service.h>
#include <profiles/project_zero/led_service.h>
#include <profiles/project_zero/data_service.h>
#include <profiles/oad/cc26xx/oad.h>

/* Includes needed for reverting to factory and erasing external flash */
#include <ti/common/cc26xx/oad/oad_image_header.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/NVS.h>
#include DeviceFamily_constructPath(driverlib/flash.h)

#include "arch_ble_application.h"
#include "Profiles/arch_ble_pressure_service.h"

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

/* Application specific includes */
#include <ti_drivers_config.h>

#include <project_zero.h>
#include "ti_ble_config.h"
#include <util.h>

/*********************************************************************
 * MACROS
 */

// Spin if the expression is not true
#define APP_ASSERT(expr) if(!(expr)) {arch_ble_application_spin();}

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0

#define A_TASK_PRIORITY                     1

#ifndef A_TASK_STACK_SIZE
#define A_TASK_STACK_SIZE                   2048
#endif

// Internal Events used by OAD profile
#define A_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define A_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02

// Internal Events for RTOS application
#define A_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define A_APP_MSG_EVT                       Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define A_ALL_EVENTS                        (A_ICALL_EVT | \
                                              A_APP_MSG_EVT | \
                                              A_OAD_QUEUE_EVT | \
                                              A_OAD_COMPLETE_EVT)

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
#define A_SERVICE_WRITE_EVT     0  /* A characteristic value has been written     */
#define A_SERVICE_CFG_EVT       1  /* A characteristic configuration has changed  */
#define A_UPDATE_CHARVAL_EVT    2  /* Request from ourselves to update a value    */
#define A_BUTTON_DEBOUNCED_EVT  3  /* A button has been debounced with new value  */
#define A_PAIRSTATE_EVT         4  /* The pairing state is updated                */
#define A_PASSCODE_EVT          5  /* A pass-code/PIN is requested during pairing */
#define A_ADV_EVT               6  /* A subscribed advertisement activity         */
#define A_START_ADV_EVT         7  /* Request advertisement start from task ctx   */
#define A_SEND_PARAM_UPD_EVT    8  /* Request parameter update req be sent        */
#define A_CONN_EVT              9  /* Connection Event End notice                 */
#define A_READ_RPA_EVT         10  /* Read RPA event                              */

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Connection interval conversion rate to miliseconds
#define CONN_INTERVAL_MS_CONVERSION           1.25

// Struct for messages sent to the application task
typedef struct { uint8_t event; void *pData; } aMsg_t;

// Struct for messages about characteristic data
typedef struct { uint16_t svcUUID; // UUID of the service
uint16_t dataLen;//
uint8_t paramID;// Index of the characteristic
uint8_t data[];// Flexible array member, extended to malloc - sizeof(.)
} aCharacteristicData_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct { uint16_t connHandle; uint8_t uiInputs; uint8_t uiOutputs; uint32_t numComparison; } aPasscodeReq_t;

// Struct for message about a pending parameter update request.
typedef struct { uint16_t connHandle; } aSendParamReq_t;

// Struct for message about button state
typedef struct { uint_least8_t gpioId; uint8_t state; } aButtonState_t; // TODO: Replace this GPIO with the BLE switch.

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct { uint8_t state; uint16_t connHandle; uint8_t status; } aPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct { uint8_t deviceAddr[B_ADDR_LEN]; uint16_t connHandle; uint8_t uiInputs; uint8_t uiOutputs; uint32_t numComparison; } aPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct { uint32_t event; void *pBuf; } aGapAdvEventData_t;

// List element for parameter update and PHY command status lists
typedef struct { List_Elem elem; uint16_t *connHandle; } aConnHandleEntry_t;

// Connected device information
typedef struct { uint16_t connHandle;                    // Connection Handle
Clock_Struct* pUpdateClock;// pointer to clock struct
bool phyCngRq;// Set to true if PHY change request is in progress
uint8_t currPhy;// The active PHY for a connection
uint8_t rqPhy;// The requested PHY for a connection
uint8_t phyRqFailCnt;// PHY change request fail count
} aConnRec_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct { uint8_t event; uint8_t data[]; } aClockEventData_t;

// Task configuration
Task_Struct aTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(appTaskStack, 8)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__ ((aligned (8)))
#else
#pragma data_alignment=8
#endif
uint8_t appTaskStack[A_TASK_STACK_SIZE];

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue; static Queue_Handle appMsgQueueHandle;

// Advertising handles
static uint8_t advHandleLegacy;

// Per-handle connection info
static aConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;

static Clock_Struct button1DebounceClock;

static Clock_Handle button0DebounceClockHandle;

static Clock_Handle button1DebounceClockHandle;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// State of the buttons 
static uint8_t button0State = 0;// TODO: Remove after test

static uint8_t button1State = 0;

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0; static bool oadWaitReboot = false;

// Flag to be stored in NV that tracks whether service changed
// indications needs to be sent out
static uint32_t sendSvcChngdOnNextBoot = FALSE;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpaAddr[B_ADDR_LEN] = {0};

/* Task functions */
static void ArchBleApplication_init(void); static void ArchBleApplication_taskFxn(UArg a0, UArg a1);

/* Event message processing functions */
static void ArchBleApplication_processStackEvent(uint32_t stack_event);

static void ArchBleApplication_processApplicationMessage(aMsg_t *pMsg);

static uint8_t ArchBleApplication_processGATTMsg(gattMsgEvent_t *pMsg);

static void ArchBleApplication_processGapMessage(gapEventHdr_t *pMsg);

static void ArchBleApplication_processHCIMsg(ICall_HciExtEvt *pMsg);

static void ArchBleApplication_processPairState(aPairStateData_t *pPairState);

static void ArchBleApplication_processPasscode(aPasscodeReq_t *pReq);

static void ArchBleApplication_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static void ArchBleApplication_processAdvEvent(aGapAdvEventData_t *pEventData);

/* Profile value change handlers */
static void ArchBleApplication_updateCharVal(aCharacteristicData_t *pCharData);

static void ArchBleApplication_PressureService_ValueChangeHandler( aCharacteristicData_t *pCharData);

static void ArchBleApplication_PressureService_CfgChangeHandler( aCharacteristicData_t *pCharData);

/* Stack or profile callback function */
static void ArchBleApplication_advCallback(uint32_t event, void *pBuf, uintptr_t arg);

static void ArchBleApplication_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);

static void ArchBleApplication_pairStateCb(uint16_t connHandle, uint8_t state, uint8_t status);

static void ArchBleApplication_PressureService_ValueChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

static void ArchBleApplication_PressureService_CfgChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

/* Connection handling functions */
static uint8_t ArchBleApplication_getConnIndex(uint16_t connHandle);

static uint8_t ArchBleApplication_clearConnListEntry(uint16_t connHandle);

static uint8_t ArchBleApplication_addConn(uint16_t connHandle);

static uint8_t ArchBleApplication_removeConn(uint16_t connHandle);

static void ArchBleApplication_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);

static void ArchBleApplication_handleUpdateLinkParamReq( gapUpdateLinkParamReqEvent_t *pReq);

static void ArchBleApplication_sendParamUpdate(uint16_t connHandle);

static void ArchBleApplication_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt);

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ

static void ArchBleApplication_paramUpdClockHandler(UArg arg);

#endif

static void ArchBleApplication_clockHandler(UArg arg);

static void ArchBleApplication_processConnEvt(Gap_ConnEventRpt_t *pReport);

static void ArchBleApplication_connEvtCB(Gap_ConnEventRpt_t *pReport);

/* Button handling functions */
static void buttonDebounceSwiFxn(UArg buttonId); static void GPIO_Board_keyCallback(uint_least8_t index);

static void ArchBleApplication_handleButtonPress(aButtonState_t *pState);

/* Utility functions */
static status_t ArchBleApplication_enqueueMsg(uint8_t event, void *pData);

static char * util_arrtohex(uint8_t const *src, uint8_t src_len, uint8_t *dst, uint8_t dst_len, uint8_t reverse);

static char * util_getLocalNameStr(const uint8_t *advData, uint8_t len);

static void ArchBleApplication_processOadWriteCB(uint8_t event, uint16_t arg);

static void ArchBleApplication_processL2CAPMsg(l2capSignalEvent_t *pMsg);

static void ArchBleApplication_checkSvcChgndFlag(uint32_t flag);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Bond Manager Callbacks
static gapBondCBs_t ArchBleApplication_BondMgrCBs = { ArchBleApplication_passcodeCb, // Passcode callback
ArchBleApplication_pairStateCb// Pairing/Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */

// Pressure Service callback handler.
// The type Data_ServiceCBs_t is defined in data_service.h
static PressureServiceCBs_t ArchBleApplication_Pressure_ServiceCBs = { .pfnChangeCb = ArchBleApplication_PressureService_ValueChangeCB, // Characteristic value change callback handler
.pfnCfgChangeCb = ArchBleApplication_PressureService_CfgChangeCB
, // Noti/ind configuration callback handler
        };

// OAD Service callback handler.
// The type oadTargetCBs_t is defined in oad.h
static oadTargetCBs_t ArchBleApplication_oadCBs = { .pfnOadWrite =
        ArchBleApplication_processOadWriteCB // Write Callback.
        };

/*********************************************************************
 * @fn     project_zero_spin
 *
 * @brief   Spin forever
 */
static void arch_ble_application_spin(void)
{
    volatile uint8_t x = 0;
    ;

    while (1)
    {
        x++; // TODO: Add a timeout here
    }
}

void ArchBleApplication_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = appTaskStack;
    taskParams.stackSize = A_TASK_STACK_SIZE;
    taskParams.priority = A_TASK_PRIORITY;

    Task_construct(&aTask, ArchBleApplication_taskFxn, &taskParams, NULL);
}

static void ArchBleApplication_init(void)
{
    // ******************************************************************
    // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
    // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
    HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
    RCOSC_enableCalibration();
#endif // USE_RCOSC

    // Initialize queue for application messages.
    // Note: Used to transfer control to application thread from e.g. interrupts.
    Queue_construct(&appMsgQueue, NULL);
    appMsgQueueHandle = Queue_handle(&appMsgQueue);

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Configure GAP for param update
    {
        uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

        // Pass all parameter update requests to the app for it to decide
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
    }

    // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
    // section in the User's Guide
    setBondManagerParameters();

    // ******************************************************************
    // BLE Service initialization
    // ******************************************************************
    GGS_AddService(GAP_SERVICE);// GAP GATT Service
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT Service

    // Add services to GATT server and give ID of this task for Indication acks.
    PressureService_AddService(selfEntity);

    // Capture the current OAD version and log it
    static uint8_t versionStr[OAD_SW_VER_LEN + 1];
    OAD_getSWVersion(versionStr, OAD_SW_VER_LEN);

    // Add in Null terminator
    versionStr[OAD_SW_VER_LEN] = 0;

    // Display Image version
    Log_info1("OAD Image v%s", (uintptr_t )versionStr);

    // Register callbacks with the generated services that
    // can generate events (writes received) to the application
    PressureService_RegisterAppCBs(&ArchBleApplication_Pressure_ServiceCBs);
    // Placeholder variable for characteristic intialization
    uint8_t initVal[40] = { 0 };
    uint8_t initString[] = "This is a pretty long string, isn't it!";

    // Initalization of characteristics in LED_Service that can provide data.
    PressureService_SetParameter(PS_BEEF_ID, PS_BEEF_LEN, initVal);
    PressureService_SetParameter(PS_CAFE_ID, PS_CAFE_LEN, initVal);
    PressureService_SetParameter(PS_FACE_ID, PS_FACE_LEN, initVal);

    // Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&ArchBleApplication_BondMgrCBs);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the HCI section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Set default values for Data Length Extension
    // Extended Data Length Feature is already enabled by default
    {
        // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
        // Some brand smartphone is essentially needing 251/2120, so we set them here.
#define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

        // This API is documented in hci.h
        // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
        // http://software-dl.ti.com/lprf/ble5stack-latest/
        HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE,
                                               APP_SUGGESTED_TX_TIME);
    }

    // Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
    GATT_InitClient();

    // Initialize Connection List
    ArchBleApplication_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

    //Initialize GAP layer for Peripheral role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode,
                   &pRandomAddress);

    // Process the Service changed flag
    ArchBleApplication_checkSvcChgndFlag(sendSvcChngdOnNextBoot);
}

static void ArchBleApplication_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    ArchBleApplication_init();

    // Application main loop
    for (;;)
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, A_ALL_EVENTS,
        ICALL_TIMEOUT_FOREVER);

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void**) &pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8_t safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event*) pMsg;

                    // Check for BLE stack events first
                    if (pEvt->signature == 0xffff)
                    {
                        // Process stack events
                        ArchBleApplication_processStackEvent(pEvt->event_flag);
                    }
                    else
                    {
                        switch (pMsg->hdr.event)
                        {
                        case GAP_MSG_EVENT:
                            // Process GAP message
                            ArchBleApplication_processGapMessage(
                                    (gapEventHdr_t*) pMsg);
                            break;

                        case GATT_MSG_EVENT:
                            // Process GATT message
                            safeToDealloc = ArchBleApplication_processGATTMsg(
                                    (gattMsgEvent_t*) pMsg);
                            break;

                        case HCI_GAP_EVENT_EVENT:
                            ArchBleApplication_processHCIMsg(pMsg);
                            break;

                        case L2CAP_SIGNAL_EVENT:
                            // Process L2CAP free buffer notification
                            ArchBleApplication_processL2CAPMsg(
                                    (l2capSignalEvent_t*) pMsg);
                            break;

                        default:
                            // do nothing
                            break;
                        }
                    }
                }

                if (pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // Process messages sent from another task or another context.
            while (!Queue_empty(appMsgQueueHandle))
            {
                aMsg_t *pMsg = (aMsg_t*) Util_dequeueMsg(appMsgQueueHandle);
                if (pMsg)
                {
                    // Process application-layer message probably sent from ourselves.
                    ArchBleApplication_processApplicationMessage(pMsg);
                    // Free the received message.
                    ICall_free(pMsg);
                }
            }

            // OAD events
            if (events & A_OAD_QUEUE_EVT)
            {
                // Process the OAD Message Queue
                uint8_t status = OAD_processQueue();

                // If the OAD state machine encountered an error, print it
                // Return codes can be found in oad_constants.h
                if (status == OAD_DL_COMPLETE)
                {
                    Log_info0("OAD DL Complete, wait for enable");
                }
                else if (status == OAD_IMG_ID_TIMEOUT)
                {
                    Log_info0("ImgID Timeout, disconnecting");

                    // This may be an attack, terminate the link,
                    // Note HCI_DISCONNECT_REMOTE_USER_TERM seems to most closet reason for
                    // termination at this state
                    MAP_GAP_TerminateLinkReq(
                            OAD_getactiveCxnHandle(),
                            HCI_DISCONNECT_REMOTE_USER_TERM);
                }
                else if (status != OAD_SUCCESS)
                {
                    Log_info1("OAD Error: %d", status);
                }
            }

            if (events & A_OAD_COMPLETE_EVT)
            {
                // Register for L2CAP Flow Control Events
                L2CAP_RegisterFlowCtrlTask(selfEntity);
            }
        }
    }
}

static void ArchBleApplication_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
    static bool firstRun = TRUE;

    switch (pMsg->opcode)
    {
    case L2CAP_NUM_CTRL_DATA_PKT_EVT:
    {
        /*
         * We cannot reboot the device immediately after receiving
         * the enable command, we must allow the stack enough time
         * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
         * command. This command will determine the number of
         * packets currently queued up by the LE controller.
         */
        if (firstRun)
        {
            firstRun = false;

            // We only want to set the numPendingMsgs once
            numPendingMsgs = MAX_NUM_PDU
                    - pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

            // Wait until all PDU have been sent on cxn events
            Gap_RegisterConnEventCb(ArchBleApplication_connEvtCB,
                                    GAP_CB_REGISTER, GAP_CB_CONN_EVENT_ALL,
                                    OAD_getactiveCxnHandle());

            /* Set the flag so that the connection event callback will
             * be processed in the context of a pending OAD reboot
             */
            oadWaitReboot = true;
        }

        break;
    }
    default:
        break;
    }
}

static void ArchBleApplication_checkSvcChgndFlag(uint32_t flag)
{
    /*
     * When booting for the first time after an OAD the device must send a service
     * changed indication. This will cause any peers to rediscover services.
     *
     * To prevent sending a service changed IND on every boot, a flag is stored
     * in NV to determine whether or not the service changed IND needs to be
     * sent
     */
    uint8_t status = osal_snv_read(BLE_NVID_CUST_START, sizeof(flag),
                                   (uint8* )&flag);
    if (status != SUCCESS)
    {
        /*
         * On first boot the NV item will not have yet been initialzed, and the read
         * will fail. Do a write to set the initial value of the flash in NV
         */
        osal_snv_write(BLE_NVID_CUST_START, sizeof(flag), (uint8* )&flag);
    }
}

static void ArchBleApplication_processStackEvent(uint32_t stack_event)
{
    // Intentionally blank
}

static uint8_t ArchBleApplication_processGATTMsg(gattMsgEvent_t *pMsg)
{
    if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
        Log_error1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
        OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
        Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return (TRUE);
}

static void ArchBleApplication_processApplicationMessage(aMsg_t *pMsg)
{
    // Cast to aCharacteristicData_t* here since it's a common message pdu type.
    aCharacteristicData_t *pCharData = (aCharacteristicData_t*) pMsg->pData;

    switch (pMsg->event)
    {
    case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
        break;

    case A_SERVICE_WRITE_EVT: /* Message about received value write */
        /* Call different handler per service */
        switch (pCharData->svcUUID)
        {
        // TODO: Add the characteristics for the write event
        case PRESSURE_SERVICE_SERV_UUID:
            ArchBleApplication_PressureService_ValueChangeHandler(pCharData);
            break;
        }
        break;

    case A_SERVICE_CFG_EVT: /* Message about received CCCD write */
        /* Call different handler per service */
        switch (pCharData->svcUUID)
        {
        case PRESSURE_SERVICE_SERV_UUID:
            ArchBleApplication_PressureService_CfgChangeHandler(pCharData);
            break;
        }
        break;

    case A_UPDATE_CHARVAL_EVT: /* Message from ourselves to send  */
        ArchBleApplication_updateCharVal(pCharData);
        break;

    case A_BUTTON_DEBOUNCED_EVT: /* Message from swi about gpio change */
    {
        aButtonState_t *pButtonState = (aButtonState_t*) pMsg->pData;
        ArchBleApplication_handleButtonPress(pButtonState);
    }
        break;

    case A_ADV_EVT:
        ArchBleApplication_processAdvEvent((aGapAdvEventData_t*) (pMsg->pData));
        break;

    case A_SEND_PARAM_UPD_EVT:
    {
        // Send connection parameter update
        aSendParamReq_t *req = (aSendParamReq_t*) pMsg->pData;
        ArchBleApplication_sendParamUpdate(req->connHandle);
    }
        break;

    case A_START_ADV_EVT:
        if (linkDB_NumActive() < MAX_NUM_BLE_CONNS)
        {
            // Enable advertising if there is room for more connections
            GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        }
        break;

    case A_PAIRSTATE_EVT: /* Message about the pairing state */
        ArchBleApplication_processPairState((aPairStateData_t*) (pMsg->pData));
        break;

    case A_PASSCODE_EVT: /* Message about pairing PIN request */
    {
        aPasscodeReq_t *pReq = (aPasscodeReq_t*) pMsg->pData;
        ArchBleApplication_processPasscode(pReq);
    }
        break;

    case A_CONN_EVT:
        ArchBleApplication_processConnEvt((Gap_ConnEventRpt_t*) (pMsg->pData));
        break;

    case A_READ_RPA_EVT:
    {
        uint8_t *pRpaNew;
        // Need static so string persists until printed in idle thread.
        static uint8_t rpaAddrStr[3 * B_ADDR_LEN + 1];

        // Read the current RPA.
        pRpaNew = GAP_GetDevAddress(FALSE);
        if (pRpaNew != NULL)
        {
            if (memcmp(pRpaNew, rpaAddr, B_ADDR_LEN) != 0)
            {
                util_arrtohex(pRpaNew, B_ADDR_LEN, rpaAddrStr,
                              sizeof(rpaAddrStr),
                              UTIL_ARRTOHEX_REVERSE);
                //print RP address
                Log_info1(
                        "RP ADDR: " ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                        (uintptr_t )rpaAddrStr);

                memcpy(rpaAddr, pRpaNew, B_ADDR_LEN);
            }
        }
        break;
    }
    default:
        break;
    }

    if (pMsg->pData != NULL)
    {
        ICall_free(pMsg->pData);
    }
}

static void ArchBleApplication_processGapMessage(gapEventHdr_t *pMsg)
{
    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        uint8_t *pRpaNew;
        bStatus_t status = FAILURE;
        // Need static so string persists until printed in idle thread.
        static uint8_t rpaAddrStr[3 * B_ADDR_LEN + 1];

        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
            // Store the system ID
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = pPkt->devAddr[0];
            systemId[1] = pPkt->devAddr[1];
            systemId[2] = pPkt->devAddr[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = pPkt->devAddr[5];
            systemId[6] = pPkt->devAddr[4];
            systemId[5] = pPkt->devAddr[3];

            // Set Device Info Service Parameter
            //DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
            //                     systemId);
            // Display device address
            // Need static so string persists until printed in idle thread.
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
            UTIL_ARRTOHEX_REVERSE);
            Log_info1(
                    "GAP is started. Our address: " ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                    (uintptr_t )addrStr);

            // Setup and start Advertising
            // For more information, see the GAP section in the User's Guide:
            // http://software-dl.ti.com/lprf/ble5stack-latest/

            // Create Advertisement set #1 and assign handle
            status = GapAdv_create(&ArchBleApplication_advCallback, &advParams1,
                                   &advHandleLegacy);

            APP_ASSERT(status == SUCCESS);

            Log_info1(
                    "Name in advData1 array: " ANSI_COLOR(FG_YELLOW) "%s" ANSI_COLOR(ATTR_RESET),
                    (uintptr_t )util_getLocalNameStr(advData1,
                                                     sizeof(advData1)));

            // Load advertising data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advData1), advData1);
            APP_ASSERT(status == SUCCESS);

            // Load scan response data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy,
                                         GAP_ADV_DATA_TYPE_SCAN_RSP,
                                         sizeof(scanResData1), scanResData1);
            APP_ASSERT(status == SUCCESS);

            // Set event mask for set #1
            status = GapAdv_setEventMask(
                    advHandleLegacy,
                    GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                            | GAP_ADV_EVT_MASK_END_AFTER_DISABLE
                            | GAP_ADV_EVT_MASK_SET_TERMINATED);

            // Enable legacy advertising for set #1
            status = GapAdv_enable(advHandleLegacy,
                                   GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
            APP_ASSERT(status == SUCCESS);

            if (addrMode > ADDRMODE_RANDOM)
            {
                // Read the current RPA.
                pRpaNew = GAP_GetDevAddress(FALSE);
                if (pRpaNew != NULL)
                {
                    // Update the current RPA.
                    memcpy(rpaAddr, pRpaNew, B_ADDR_LEN);
                    //print RP address
                    util_arrtohex(pRpaNew, B_ADDR_LEN, rpaAddrStr,
                                  sizeof(rpaAddrStr),
                                  UTIL_ARRTOHEX_REVERSE);
                    Log_info1(
                            "RP ADDR: " ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                            (uintptr_t )rpaAddrStr);

                    // Create one-shot clock for RPA check event.
                    Util_constructClock(&clkRpaRead,
                                        ArchBleApplication_clockHandler,
                                        READ_RPA_PERIOD,
                                        0, true, A_READ_RPA_EVT);
                }
            }
        }
        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t*) pMsg;

        // Display the amount of current connections
        Log_info2("Link establish event, status 0x%02x. Num Conns: %d",
                  pPkt->hdr.status, linkDB_NumActive());

        if (pPkt->hdr.status == SUCCESS)
        {
            // Add connection to list
            ArchBleApplication_addConn(pPkt->connectionHandle);

            // Display the address of this connection
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
            UTIL_ARRTOHEX_REVERSE);
            Log_info1(
                    "Connected. Peer address: " ANSI_COLOR(FG_GREEN)"%s"ANSI_COLOR(ATTR_RESET),
                    (uintptr_t )addrStr);

            // If we are just connecting after an OAD send SVC changed
            if (sendSvcChngdOnNextBoot == TRUE)
            {
                /* Warning: This requires -DV41_FEATURES=L2CAP_COC_CFG to be
                 * defined in the build_config.opt of the stack project
                 * If L2CAP CoC is not desired comment the following code out
                 */
                GAPBondMgr_ServiceChangeInd(pPkt->connectionHandle, TRUE);

                sendSvcChngdOnNextBoot = FALSE;
            }
        }

        if (linkDB_NumActive() < MAX_NUM_BLE_CONNS)
        {
            // Start advertising since there is room for more connections
            GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        }
    }
        break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t*) pMsg;

        // Display the amount of current connections
        Log_info0("Device Disconnected!");
        Log_info1("Num Conns: %d", linkDB_NumActive());

        // Remove the connection from the list and disable RSSI if needed
        ArchBleApplication_removeConn(pPkt->connectionHandle);

        // Cancel the OAD if one is going on
        // A disconnect forces the peer to re-identify
        OAD_cancel();
    }
        break;

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
        ArchBleApplication_handleUpdateLinkParamReq(
                (gapUpdateLinkParamReqEvent_t*) pMsg);
        break;
    case GAP_LINK_PARAM_UPDATE_EVENT:
        ArchBleApplication_handleUpdateLinkEvent((gapLinkUpdateEvent_t*) pMsg);
        break;

    case GAP_PAIRING_REQ_EVENT:
        // Disable advertising so that the peer device can be added to
        // the resolving list
        GapAdv_disable(advHandleLegacy);
        break;

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
        {
          linkDBInfo_t linkInfo;
          gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

          // Get the address from the connection handle
          linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

          // Display the address of the connection update failure
          Log_info2("Peer Device's Update Request Rejected 0x%x: %s", pPkt->opcode,
                     Util_convertBdAddr2Str(linkInfo.addr));

          break;
        }
#endif

    default:
        break;
    }
}

void ArchBleApplication_processHCIMsg(ICall_HciExtEvt *pEvt)
{
    ICall_Hdr *pMsg = (ICall_Hdr*) pEvt;

    // Process HCI message
    switch (pMsg->status)
    {
    case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        ArchBleApplication_processCmdCompleteEvt((hciEvt_CmdComplete_t*) pMsg);
        break;

    case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
        break;

        // HCI Commands Events
    case HCI_COMMAND_STATUS_EVENT_CODE:
    {
        hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;
        switch (pMyMsg->cmdOpcode)
        {
        case HCI_LE_SET_PHY:
        {
            if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
            {
                Log_info0("PHY Change failure, peer does not support this");
            }
            else
            {
                Log_info1("PHY Update Status Event: 0x%x", pMyMsg->cmdStatus);
            }

            ArchBleApplication_updatePHYStat(HCI_LE_SET_PHY, (uint8_t*) pMsg);
        }
            break;

        default:
            break;
        }
    }
        break;

        // LE Events
    case HCI_LE_EVENT_CODE:
    {
        hciEvt_BLEPhyUpdateComplete_t *pPUC =
                (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

        // A Phy Update Has Completed or Failed
        if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
        {
            if (pPUC->status != SUCCESS)
            {
                Log_info0("PHY Change failure");
            }
            else
            {
                // Only symmetrical PHY is supported.
                // rxPhy should be equal to txPhy.
                Log_info1(
                        "PHY Updated to %s",
                        (uintptr_t)((pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" : (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" : (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value"));
            }

            ArchBleApplication_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT,
                                             (uint8_t*) pMsg);
        }
    }
        break;

    default:
        break;
    }
}

static void ArchBleApplication_processAdvEvent(aGapAdvEventData_t *pEventData)
{
    switch (pEventData->event)
    {
    /* Sent on the first advertisement after a GapAdv_enable */
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        Log_info1("Adv Set %d Enabled", *(uint8_t* )(pEventData->pBuf));
        break;

        /* Sent after advertising stops due to a GapAdv_disable */
    case GAP_EVT_ADV_END_AFTER_DISABLE:
        Log_info1("Adv Set %d Disabled", *(uint8_t* )(pEventData->pBuf));
        break;

        /* Sent at the beginning of each advertisement. (Note that this event
         * is not enabled by default, see GapAdv_setEventMask). */
    case GAP_EVT_ADV_START:
        break;

        /* Sent after each advertisement. (Note that this event is not enabled
         * by default, see GapAdv_setEventMask). */
    case GAP_EVT_ADV_END:
        break;

        /* Sent when an advertisement set is terminated due to a
         * connection establishment */
    case GAP_EVT_ADV_SET_TERMINATED:
    {
        GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t*) (pEventData->pBuf);

        Log_info2("Adv Set %d disabled after conn %d", advSetTerm->handle,
                  advSetTerm->connHandle);
    }
        break;

        /* Sent when a scan request is received. (Note that this event
         * is not enabled by default, see GapAdv_setEventMask). */
    case GAP_EVT_SCAN_REQ_RECEIVED:
        break;

        /* Sent when an operation could not complete because of a lack of memory.
         This message is not allocated on the heap and must not be freed */
    case GAP_EVT_INSUFFICIENT_MEMORY:
        break;

    default:
        break;
    }

    // All events have associated memory to free except the insufficient memory
    // event
    if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
    {
        ICall_free(pEventData->pBuf);
    }
}

static void ArchBleApplication_processPairState(aPairStateData_t *pPairData)
{
    uint8_t state = pPairData->state;
    uint8_t status = pPairData->status;

    switch (state)
    {
    case GAPBOND_PAIRING_STATE_STARTED:
        Log_info0("Pairing started");
        break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
        if (status == SUCCESS)
        {
            Log_info0("Pairing success");
        }
        else
        {
            Log_info1("Pairing fail: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
        if (status == SUCCESS)
        {
            Log_info0("Encryption success");
        }
        else
        {
            Log_info1("Encryption failed: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
        if (status == SUCCESS)
        {
            Log_info0("Bond save success");
        }
        else
        {
            Log_info1("Bond save failed: %d", status);
        }
        break;

    default:
        break;
    }
}

static void ArchBleApplication_processPasscode(aPasscodeReq_t *pReq)
{
    Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
              (uintptr_t)(pReq->uiInputs ? "Sending" : "Displaying"),
              B_APP_DEFAULT_PASSCODE);

    // Send passcode response.
    GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

static void ArchBleApplication_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
    /* If we are waiting for an OAD Reboot, process connection events to ensure
     * that we are not waiting to send data before restarting
     */
    if (oadWaitReboot)
    {
        // Wait until all pending messages are sent
        if (numPendingMsgs == 0)
        {
            // Store the flag to indicate that a service changed IND will
            // be sent at the next boot
            sendSvcChngdOnNextBoot = TRUE;

            uint8_t status = osal_snv_write(BLE_NVID_CUST_START,
                                            sizeof(sendSvcChngdOnNextBoot),
                                            (uint8* )&sendSvcChngdOnNextBoot);
            if (status != SUCCESS)
            {
                Log_error1("SNV WRITE FAIL: %d", status);
            }

            // Reset the system
            SysCtrlSystemReset();
        }
        else
        {
            numPendingMsgs--;
        }
    }
    else
    {
        // Process connection events normally
        Log_info1("Connection event done for connHandle: %d", pReport->handle);

    }
}

void ArchBleApplication_clockHandler(UArg arg)
{
    uint8_t evtId = (uint8_t)(arg & 0xFF);

    switch (evtId)
    {
    case A_READ_RPA_EVT:
    {
        // Restart timer
        Util_startClock(&clkRpaRead);
        // Let the application handle the event
        ArchBleApplication_enqueueMsg(A_READ_RPA_EVT, NULL);
        break;
    }

    default:
        break;
    }
}

static void ArchBleApplication_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
    // Enqueue the event for processing in the app context.
    if (ArchBleApplication_enqueueMsg(A_CONN_EVT, pReport) != SUCCESS)
    {
        ICall_free(pReport);
    }
}

static void ArchBleApplication_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
    uint8_t status = pMsg->pReturnParam[0];

    //Find which command this command complete is for
    switch (pMsg->cmdOpcode)
    {
    case HCI_READ_RSSI:
    {
        int8 rssi = (int8) pMsg->pReturnParam[3];

        // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
        if (status == SUCCESS)
        {
            uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1],
                                           pMsg->pReturnParam[2]);

            Log_info2("RSSI:%d, connHandle %d", (uint32_t)(rssi),
                      (uint32_t )handle);
        } // end of if (status == SUCCESS)
        break;
    }

    case HCI_LE_READ_PHY:
    {
        if (status == SUCCESS)
        {
            Log_info2("RXPh: %d, TXPh: %d", pMsg->pReturnParam[3],
                      pMsg->pReturnParam[4]);
        }
        break;
    }

    default:
        break;
    } // end of switch (pMsg->cmdOpcode)
}

static void ArchBleApplication_handleUpdateLinkParamReq(
        gapUpdateLinkParamReqEvent_t *pReq)
{
    gapUpdateLinkParamReqReply_t rsp;

    rsp.connectionHandle = pReq->req.connectionHandle;
    rsp.signalIdentifier = pReq->req.signalIdentifier;

    // Only accept connection intervals with slave latency of 0
    // This is just an example of how the application can send a response
    if (pReq->req.connLatency == 0)
    {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
    }
    else
    {
        rsp.accepted = FALSE;
    }

    // Send Reply
    VOID GAP_UpdateLinkParamReqReply(&rsp);
}

static void ArchBleApplication_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt)
{
    // Get the address from the connection handle
    linkDBInfo_t linkInfo;
    linkDB_GetInfo(pEvt->connectionHandle, &linkInfo);

    static uint8_t addrStr[3 * B_ADDR_LEN + 1];
    util_arrtohex(linkInfo.addr, B_ADDR_LEN, addrStr, sizeof addrStr,
    UTIL_ARRTOHEX_REVERSE);

    if (pEvt->status == SUCCESS)
    {
        uint8_t ConnIntervalFracture = 25 * (pEvt->connInterval % 4);
        // Display the address of the connection update
        Log_info5(
                "Updated params for %s, interval: %d.%d ms, latency: %d, timeout: %d ms",
                (uintptr_t )addrStr,
                (uintptr_t)(pEvt->connInterval*CONN_INTERVAL_MS_CONVERSION),
                ConnIntervalFracture, pEvt->connLatency,
                pEvt->connTimeout*CONN_TIMEOUT_MS_CONVERSION);
    }
    else
    {
        // Display the address of the connection update failure
        Log_info2("Update Failed 0x%02x: %s", pEvt->opcode,
                  (uintptr_t )addrStr);
    }

    // Check if there are any queued parameter updates
    aConnHandleEntry_t *connHandleEntry = (aConnHandleEntry_t*) List_get(
            &paramUpdateList);
    if (connHandleEntry != NULL)
    {
        // Attempt to send queued update now
        ArchBleApplication_sendParamUpdate(*(connHandleEntry->connHandle));

        // Free list element
        ICall_free(connHandleEntry->connHandle);
        ICall_free(connHandleEntry);
    }
}

static uint8_t ArchBleApplication_addConn(uint16_t connHandle)
{
    uint8_t i;
    uint8_t status = bleNoResources;

    // Try to find an available entry
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
        {
            // Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
            // Create a clock object and start
            connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
                    sizeof(Clock_Struct));

            if (connList[i].pUpdateClock)
            {
                Util_constructClock(connList[i].pUpdateClock,
                                    ArchBleApplication_paramUpdClockHandler,
                                    SEND_PARAM_UPDATE_DELAY,
                                    0, true, (uintptr_t) connHandle);
            }
#endif

            // Set default PHY to 1M
            connList[i].currPhy = HCI_PHY_1_MBPS; // TODO: Is this true, neccessarily?

            break;
        }
    }

    return (status);
}

static uint8_t ArchBleApplication_getConnIndex(uint16_t connHandle)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == connHandle)
        {
            return (i);
        }
    }

    return (MAX_NUM_BLE_CONNS);
}
static uint8_t ArchBleApplication_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if (connHandle != LINKDB_CONNHANDLE_ALL)
    {
        // Get connection index from handle
        connIndex = ArchBleApplication_getConnIndex(connHandle);
        if (connIndex >= MAX_NUM_BLE_CONNS)
        {
            return (bleInvalidRange);
        }
    }

    // Clear specific handle or all handles
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if ((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
        {
            connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
            connList[i].currPhy = 0;
            connList[i].phyCngRq = 0;
            connList[i].phyRqFailCnt = 0;
            connList[i].rqPhy = 0;
        }
    }

    return (SUCCESS);
}

static uint8_t ArchBleApplication_removeConn(uint16_t connHandle)
{
    uint8_t connIndex = ArchBleApplication_getConnIndex(connHandle);

    if (connIndex < MAX_NUM_BLE_CONNS)
    {
        Clock_Struct *pUpdateClock = connList[connIndex].pUpdateClock;

        if (pUpdateClock != NULL)
        {
            // Stop and destruct the RTOS clock if it's still alive
            if (Util_isActive(pUpdateClock))
            {
                Util_stopClock(pUpdateClock);
            }

            // Destruct the clock object
            Clock_destruct(pUpdateClock);
            // Free clock struct
            ICall_free(pUpdateClock);
        }
        // Clear Connection List Entry
        ArchBleApplication_clearConnListEntry(connHandle);
    }

    return connIndex;
}

static void ArchBleApplication_sendParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

    connIndex = ArchBleApplication_getConnIndex(connHandle);
    APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);
    // Free clock struct
    ICall_free(connList[connIndex].pUpdateClock);
    connList[connIndex].pUpdateClock = NULL;

    // Send parameter update
    bStatus_t status = GAP_UpdateLinkParamReq(&req);

    // If there is an ongoing update, queue this for when the update completes
    if (status == bleAlreadyInRequestedMode)
    {
        aConnHandleEntry_t *connHandleEntry = ICall_malloc(
                sizeof(aConnHandleEntry_t));
        if (connHandleEntry)
        {
            connHandleEntry->connHandle = ICall_malloc(sizeof(uint16_t));

            if (connHandleEntry->connHandle)
            {
                *(connHandleEntry->connHandle) = connHandle;

                List_put(&paramUpdateList, (List_Elem*) &connHandleEntry);
            }
        }
    }
}

static void ArchBleApplication_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
    uint8_t connIndex;
    aConnHandleEntry_t *connHandleEntry;

    switch (eventCode)
    {
    case HCI_LE_SET_PHY:
    {
        // Get connection handle from list
        connHandleEntry = (aConnHandleEntry_t*) List_get(&setPhyCommStatList);

        if (connHandleEntry)
        {
            // Get index from connection handle
            connIndex = ArchBleApplication_getConnIndex(
                    *(connHandleEntry->connHandle));
            APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

            ICall_free(connHandleEntry->connHandle);
            ICall_free(connHandleEntry);

            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;

            if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
            {
                // Update the phy change request status for active RSSI tracking connection
                connList[connIndex].phyCngRq = FALSE;
                connList[connIndex].phyRqFailCnt++;
            }
        }
        break;
    }

        // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
        hciEvt_BLEPhyUpdateComplete_t *pPUC =
                (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

        if (pPUC)
        {
            // Get index from connection handle
            uint8_t index = ArchBleApplication_getConnIndex(pPUC->connHandle);
            APP_ASSERT(index < MAX_NUM_BLE_CONNS);

            // Update the phychange request status for active RSSI tracking connection
            connList[index].phyCngRq = FALSE;

            if (pPUC->status == SUCCESS)
            {
                connList[index].currPhy = pPUC->rxPhy;
            }
            if (pPUC->rxPhy != connList[index].rqPhy)
            {
                connList[index].phyRqFailCnt++;
            }
            else
            {
                // Reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
                connList[index].rqPhy = 0;
            }
        }

        break;
    }

    default:
        break;
    } // end of switch (eventCode)
}

static void ArchBleApplication_handleButtonPress(aButtonState_t *pState)
{
    //Log_info2("Switch toggled"); // TODO: GPIO implementation
}

void ArchBleApplication_PressureService_ValueChangeHandler(
        aCharacteristicData_t *pCharData)
{
    static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
    util_arrtohex(pCharData->data, pCharData->dataLen, pretty_data_holder,
                  sizeof(pretty_data_holder),
                  UTIL_ARRTOHEX_NO_REVERSE);

    switch (pCharData->paramID)
    {
    case PS_BEEF_ID:
        Log_info3("Value Change msg: %s %s: %s", (uintptr_t )"LED Service",
                  (uintptr_t )"LED0", (uintptr_t )pretty_data_holder);

        break;

    case PS_FACE_ID:
        Log_info3("Value Change msg: %s %s: %s", (uintptr_t )"LED Service",
                  (uintptr_t )"LED1", (uintptr_t )pretty_data_holder);

        break;

    default:
        return;
    }
}

void ArchBleApplication_PressureService_CfgChangeHandler(
        aCharacteristicData_t *pCharData)
{
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t*) pCharData->data;
    char *configValString;

    // Determine what to tell the user
    switch (configValue)
    {
    case GATT_CFG_NO_OPERATION:
        configValString = "Noti/Ind disabled";
        break;
    case GATT_CLIENT_CFG_NOTIFY:
        configValString = "Notifications enabled";
        break;
    case GATT_CLIENT_CFG_INDICATE:
        configValString = "Indications enabled";
        break;
    default:
        configValString = "Unsupported operation";
    }

    switch (pCharData->paramID)
    {
    case PS_BEEF_ID:
        Log_info3("CCCD Change msg: %s %s: %s", (uintptr_t )"Button Service",
                  (uintptr_t )"BUTTON0", (uintptr_t )configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;

    case PS_FACE_ID:
        Log_info3("CCCD Change msg: %s %s: %s", (uintptr_t )"Button Service",
                  (uintptr_t )"BUTTON1", (uintptr_t )configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;
    }
}

static void ArchBleApplication_updateCharVal(aCharacteristicData_t *pCharData)
{
    switch (pCharData->svcUUID)
    {
    case PRESSURE_SERVICE_SERV_UUID:
        PressureService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                     pCharData->data);
        break;
    }
}

static void ArchBleApplication_advCallback(uint32_t event, void *pBuf,
                                           uintptr_t arg)
{
    aGapAdvEventData_t *eventData = ICall_malloc(sizeof(aGapAdvEventData_t));

    if (eventData != NULL)
    {
        eventData->event = event;
        eventData->pBuf = pBuf;

        if (ArchBleApplication_enqueueMsg(A_ADV_EVT, eventData) != SUCCESS)
        {
            ICall_free(eventData);
        }
    }
}

static void ArchBleApplication_pairStateCb(uint16_t connHandle, uint8_t state,
                                           uint8_t status)
{
    aPairStateData_t *pairState = (aPairStateData_t*) ICall_malloc(
            sizeof(aPairStateData_t));

    if (pairState != NULL)
    {
        pairState->state = state;
        pairState->connHandle = connHandle;
        pairState->status = status;

        if (ArchBleApplication_enqueueMsg(A_PAIRSTATE_EVT, pairState) != SUCCESS)
        {
            ICall_free(pairState);
        }
    }
}

static void ArchBleApplication_passcodeCb(uint8_t *pDeviceAddr,
                                          uint16_t connHandle, uint8_t uiInputs,
                                          uint8_t uiOutputs,
                                          uint32_t numComparison)
{
    aPasscodeReq_t *req = (aPasscodeReq_t*) ICall_malloc(
            sizeof(aPasscodeReq_t));
    if (req != NULL)
    {
        req->connHandle = connHandle;
        req->uiInputs = uiInputs;
        req->uiOutputs = uiOutputs;
        req->numComparison = numComparison;

        if (ArchBleApplication_enqueueMsg(A_PASSCODE_EVT, req) != SUCCESS)
        {
            ICall_free(req);
        }
    };
}

static void ArchBleApplication_PressureService_ValueChangeCB(
        uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue)
{
    // See the service header file to compare paramID with characteristic.
    Log_info1("(CB) Pressure Service Characteristic value change: paramID(%d). "
              "Sending msg to app.",
              paramID);

    aCharacteristicData_t *pValChange = ICall_malloc(
            sizeof(aCharacteristicData_t) + len);

    if (pValChange != NULL)
    {
        pValChange->svcUUID = LED_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if (ArchBleApplication_enqueueMsg(A_SERVICE_WRITE_EVT,
                                          pValChange) != SUCCESS)
        {
            ICall_free(pValChange);
        }
    }
}

static void ArchBleApplication_PressureService_CfgChangeCB(uint16_t connHandle,
                                                           uint8_t paramID,
                                                           uint16_t len,
                                                           uint8_t *pValue)
{
    Log_info1("(CB) Pressure Service Char config change paramID(%d). "
              "Sending msg to app.",
              paramID);

    aCharacteristicData_t *pValChange = ICall_malloc(
            sizeof(aCharacteristicData_t) + len);

    if (pValChange != NULL)
    {
        pValChange->svcUUID = PRESSURE_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if (ArchBleApplication_enqueueMsg(A_SERVICE_CFG_EVT,
                                          pValChange) != SUCCESS)
        {
            ICall_free(pValChange);
        }
    }
}

static void ArchBleApplication_processOadWriteCB(uint8_t event, uint16_t arg)
{
    Event_post(syncEvent, event);
}

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
static void ArchBleApplication_paramUpdClockHandler(UArg arg)
{
    aSendParamReq_t *req = (aSendParamReq_t*) ICall_malloc(
            sizeof(aSendParamReq_t));
    if (req)
    {
        req->connHandle = (uint16_t) arg;
        if (ArchBleApplication_enqueueMsg(A_SEND_PARAM_UPD_EVT, req) != SUCCESS)
        {
            ICall_free(req);
        }
    }
}
#endif

static void GPIO_Board_keyCallback(uint_least8_t index)
{
    Log_info1(
            "Button interrupt: %s",
            (uintptr_t)((index == CONFIG_GPIO_BTN1) ? "Button 0" : "Button 1"));

    // Disable interrupt on that gpio for now. Re-enabled after debounce.
    GPIO_setConfig(index,
    GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INT_DISABLE);

    // Start debounce timer
    switch (index)
    {
    case CONFIG_GPIO_BTN1:
        Util_startClock((Clock_Struct*) button0DebounceClockHandle);
        break;
    case CONFIG_GPIO_BTN2:
        Util_startClock((Clock_Struct*) button1DebounceClockHandle);
        break;
    }
}

static status_t ArchBleApplication_enqueueMsg(uint8_t event, void *pData)
{
    uint8_t success;
    aMsg_t *pMsg = ICall_malloc(sizeof(aMsg_t));

    if (pMsg)
    {
        pMsg->event = event;
        pMsg->pData = pData;

        success = Util_enqueueMsg(appMsgQueueHandle, syncEvent,
                                  (uint8_t*) pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return (bleMemAllocError);
}

char* util_arrtohex(uint8_t const *src, uint8_t src_len, uint8_t *dst,
                    uint8_t dst_len, uint8_t reverse)
{
    char hex[] = "0123456789ABCDEF";
    uint8_t *pStr = dst;
    uint8_t avail = dst_len - 1;
    int8_t inc = 1;
    if (reverse)
    {
        src = src + src_len - 1;
        inc = -1;
    }

    memset(dst, 0, avail);

    while (src_len && avail > 3)
    {
        if (avail < dst_len - 1)
        {
            *pStr++ = ':';
            avail -= 1;
        }

        *pStr++ = hex[*src >> 4];
        *pStr++ = hex[*src & 0x0F];
        src += inc;
        avail -= 2;
        src_len--;
    }

    if (src_len && avail)
    {
        *pStr++ = ':'; // Indicate not all data fit on line.
    }
    return ((char*) dst);
}

static char* util_getLocalNameStr(const uint8_t *data, uint8_t len)
{
    uint8_t nuggetLen = 0;
    uint8_t nuggetType = 0;
    uint8_t advIdx = 0;

    static char localNameStr[32] = { 0 };
    memset(localNameStr, 0, sizeof(localNameStr));

    for (advIdx = 0; advIdx < len;)
    {
        nuggetLen = data[advIdx++];
        nuggetType = data[advIdx];
        if ((nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE
                || nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT))
        {
            uint8_t len_temp =
                    nuggetLen < (sizeof(localNameStr) - 1) ?
                            (nuggetLen - 1) : (sizeof(localNameStr) - 2);
            // Only copy the first 31 characters, if name bigger than 31.
            memcpy(localNameStr, &data[advIdx + 1], len_temp);
            break;
        }
        else
        {
            advIdx += nuggetLen;
        }
    }

    return (localNameStr);
}

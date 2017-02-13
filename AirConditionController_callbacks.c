//

// This callback file is created for your convenience. You may add application
// code to this file. If you regenerate this file over a previous version, the
// previous version will be overwritten and any code you have added will be
// lost.

#include "app/framework/include/af.h"
#include "hal/micro/led-blink.h"
#include "app/util/common/form-and-join.h"
#include "crc16-modbus.h"
#include "adapter-command.h"
#include "RingBufferUtils.h"
#include "attributeReporting.h"

// Custom event stubs. Custom events will be run along with all other events in
// the application framework. They should be managed using the Ember Event API
// documented in stack/include/events.h

uint8_t u8ButtonTimes = 0;
#define BUTTON_PRESS_TIMES	4

// Event control struct declarations
EmberEventControl searchZnetEventControl;
EmberEventControl UARTEventControl;
EmberEventControl customMessageEventControl;
extern EmberEventControl attributeReportingEventControl;
extern EmberEventControl MultiAttributeReportingEventControl;
EmberEventControl customEventControl;
EmberEventControl clearButtonTimesControl;
extern EmberEventControl buttonReadEventControl;
extern EmberEventControl emberAfPluginButtonInterfaceClearButtonShortContinuePressCntEventControl;
extern EmberEventControl emberAfPluginButtonInterfaceButton2ReleasedEventControl;
extern EmberEventControl emberAfPluginButtonInterfaceButton2PressedEventControl;
EmberEventControl synchronizeIndoorStatus;
EmberEventControl synchronizeAdapterStatus;
EmberEventControl synchronizeIndoorConnectionStatus;

// Event function forward declarations
void searchZnetEventHandler(void);
void UARTEventHandler(void);
void RS485EventHandler(void);
void customMessageEventFunction(void);
void attributeReportingEventHandler(void);
void MultiAttributeReportingEventHandler(void);
void customEventFunction(void);
void clearButtonTimesEventHandler(void);
void buttonReadEventHandler(void);
void emberAfPluginButtonInterfaceClearButtonShortContinuePressCntEventHandler(void);
void emberAfPluginButtonInterfaceButton2ReleasedEventHandler(void);
void emberAfPluginButtonInterfaceButton2PressedEventHandler(void);
void synchronizeIndoorStatusHandler(void);
void synchronizeAdapterStatusHandler(void);
void synchronizeIndoorConnectionStatusHandler(void);
//Attribute Write from Local
bool bWriteFromLocal = FALSE;

void synchronizeAdapterStatusHandler(void)
{
    emberEventControlSetInactive(synchronizeAdapterStatus);
    tsSL_Msg sSL_Msg;
    sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
    sSL_Msg.eMsgType = E_SL_MSG_GET_ADAPTER_STATUS_REQUEST;
    sSL_Msg.u8EndPoint = 0xFF;
    sSL_Msg.u8RetryCount = 0;
    if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
    {
        emberAfAppPrintln("MsgQueue is Full");
        ring_buffer_display(&sMsgTxRingBuffer);
    }
    if(Adapter.bStatus == E_ADAPTER_NOREADY)
        emberEventControlSetDelayMS(synchronizeAdapterStatus,SYNCHRONIZE_ADAPTER_STATUS_SHORT_CYCLE);
    else
        emberEventControlSetDelayMS(synchronizeAdapterStatus,SYNCHRONIZE_ADAPTER_STATUS_LONG_CYCLE);
}

void synchronizeIndoorConnectionStatusHandler(void)
{
    emberEventControlSetInactive(synchronizeIndoorConnectionStatus);
    if(Adapter.bStatus == E_ADAPTER_NOREADY)
    {
        emberEventControlSetDelayMS(synchronizeIndoorConnectionStatus,SYNCHRONIZE_INDOOR_CONNECTION_STATUS_CYCLE);
        return ;
    }
    tsSL_Msg sSL_Msg;
    sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
    sSL_Msg.eMsgType = E_SL_MSG_GET_INDOOR_CONNECTION_STATUS_REQUEST;
    sSL_Msg.u8EndPoint = 0xFF;
    sSL_Msg.u8RetryCount = 0;
    if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
    {
        emberAfAppPrintln("MsgQueue is Full");
        ring_buffer_display(&sMsgTxRingBuffer);
    }
    emberEventControlSetDelayMS(synchronizeIndoorConnectionStatus,SYNCHRONIZE_INDOOR_CONNECTION_STATUS_CYCLE);
}


uint8_t synchronizeIndoorIndex = 0;
void synchronizeIndoorStatusHandler(void)
{
    emberEventControlSetInactive(synchronizeIndoorStatus);

    if(/*(Adapter.bStatus == E_ADAPTER_NOREADY)||*/((Adapter.u16IndoorConnectionStatus[0] == 00)&& (Adapter.u16IndoorConnectionStatus[1] == 00)&& (Adapter.u16IndoorConnectionStatus[2] == 00)&&( Adapter.u16IndoorConnectionStatus[3] == 0)))
    {
        emberEventControlSetDelayMS(synchronizeIndoorStatus,SYNCHRONIZE_INDOOR_STATUS_CYCLE);
        return ;
    }
    emberAfAppPrintln("get status");

    while(1)
    {
        synchronizeIndoorIndex ++;
        synchronizeIndoorIndex %= 64;

        if(Adapter.u16IndoorConnectionStatus[synchronizeIndoorIndex/16] & (1<<(synchronizeIndoorIndex%16)))
        {
            tsSL_Msg sSL_Msg;
            //�����ȡ�ڻ�״̬����
            sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
            sSL_Msg.eMsgType = E_SL_MSG_GET_INDOOR_STATUS_REQUEST;
            sSL_Msg.u8EndPoint = synchronizeIndoorIndex + ADAPTER_INDOOR_ENDPOINT_SHIFT;
            sSL_Msg.u8RetryCount = 0;
            sSL_Msg.uMessage.sGetIndoorPerformanceInfoRequest.u8IndoorAddress = synchronizeIndoorIndex + 0x10;
            emberAfAppPrintln("u8IndoorAddress:0x%x,endpoint:%d",sSL_Msg.uMessage.sGetIndoorPerformanceInfoRequest.u8IndoorAddress,sSL_Msg.u8EndPoint);
            if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
            {
                emberAfAppPrintln("MsgQueue is Full");
                ring_buffer_display(&sMsgTxRingBuffer);
            }
            emberEventControlSetDelayMS(synchronizeIndoorStatus,SYNCHRONIZE_INDOOR_STATUS_CYCLE);
            return ;
        }
    }
}
void clearButtonTimesEventHandler(void)
{
    emberEventControlSetInactive(clearButtonTimesControl);
    emberAfAppPrintln("button timeout");
    u8ButtonTimes = 0;
}


void customEventFunction()
{
    eACMD_EnableRx();
}

void RS485EventHandler(void)
{
    eACMD_EnableRx();
}


// Event function stubs
void searchZnetEventHandler(void)
{
    EmberNetworkStatus state = EMBER_NO_NETWORK;
    emberEventControlSetInactive(searchZnetEventControl);
    state = emberAfNetworkState();
    emberAfAppPrintln("EmberNetworkStatus:%d",state);
    if (state == EMBER_NO_NETWORK)
    {
        if(!emberFormAndJoinIsScanning())
        {
            emberAfAppPrintln("Not Scanning,have to start");
            emberAfStartSearchForJoinableNetwork();
            halMultiLedBlinkBlink(255, 500,BOARDLED0);
        }
        else
        {
            emberAfAppPrintln("Is Scanning,not have to start");
        }
    }
}
extern unsigned char Check_Rx_Data_Flag_0;
extern unsigned char Rx_Data_Start_Flag_0;
extern unsigned char USART_Tx_Rx_Count_0;
extern unsigned char rx_buffer0[];
void UARTEventHandler(void)
{


}

#define BUILD_UINT16(loByte, hiByte) ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))


//void attributeReportingEventHandler(void) { }
//void MultiAttributeReportingEventHandler(void) { }


/** @brief Button0 Pressed Short
 *
 * This function returns the number of times a button was short pressed.
 *
 * @param timePressedMs Time (in ms) button 0 was pressed  Ver.: always
 */
void emberAfPluginButtonInterfaceButton0PressedShortCallback(uint16_t timePressedMs)
{
    // Record button pressed times
    u8ButtonTimes ++;
    // if u8ButtonTimes == 1  and Is not Scanning,So start to Joinning
    if((u8ButtonTimes == 1) && (!emberFormAndJoinIsScanning()))
        emberEventControlSetDelayMS(searchZnetEventControl,0);
    emberAfAppPrintln("button0 short:%d,times:%d",timePressedMs,u8ButtonTimes);
    //Try to clear button pressed times
    emberEventControlSetDelayMS(clearButtonTimesControl,3000);
}

/** @brief Button0 Pressed Long
 *
 * This function returns the number of times a button was short pressed.
 *
 * @param timePressedMs Amount of time button 0 was pressed.  Ver.: always
 * @param pressedAtReset Was the button pressed at startup.  Ver.: always
 */
void emberAfPluginButtonInterfaceButton0PressedLongCallback(uint16_t timePressedMs,
        bool pressedAtReset)
{
    emberAfAppPrintln("button0 long release:%d",timePressedMs);
}

bool tryToStopScan = FALSE;
/** @brief Button0 Pressing
 *
 * This function is periodically called when button 0 is being pressed.
 *
 */
void emberAfPluginButtonInterfaceButton0PressingCallback(void)
{
    EmberStatus status;
    emberAfAppPrintln("button0 long:%d");
    if(u8ButtonTimes == BUTTON_PRESS_TIMES)
    {
        emberAfAppPrintln("leave NWK");
        status = emberLeaveNetwork();
        UNUSED_VAR(status);
        emberAfAppPrintln("%p 0x%x", "leave",  status);
        /*EMBER_INVALID_CALL indicates that the node is either not	joined to a network or is already in the process of leaving.*/
        if(status == EMBER_INVALID_CALL)
        {
            //Try to Stop Scan
            tryToStopScan = TRUE;
            //emberAfStartSearchForJoinableNetwork();
            emberAfAppPrintln("Network Down,Flash LED");
            // Quick blink,continue 20s
            halMultiLedBlinkBlink(200, 50,BOARDLED0);
            // Start to Scan in 21s
            emberEventControlSetDelayMS(searchZnetEventControl,21000);
        }
    }
}

/** @brief Button0 High
 *
 * This function is called when the GPIO tied to button zero goes high
 *
 */
void emberAfPluginButtonInterfaceButton0HighCallback(void)
{

}

/** @brief Button0 Low
 *
 * This function is called when the GPIO tied to button zero goes low
 *
 */
void emberAfPluginButtonInterfaceButton0LowCallback(void)
{

}

void emberAfPluginButtonInterfaceButton2HighCallback(void)
{

}

void emberAfPluginButtonInterfaceButton2PressedShortCallback(
    uint16_t timePressedMs)
{

}

void emberAfPluginButtonInterfaceButton2LowCallback(void)
{

}

void emberAfPluginButtonInterfaceButton2PressingCallback(uint8_t shortPressCnt)
{

}

void emberAfPluginButtonInterfaceButton2PressedLongCallback(
    uint16_t timePressedMs,
    bool pressedAtReset)
{


}



/** @brief Main Init
 *
 * This function is called from the application's main function. It gives the
 * application a chance to do any initialization required at system startup. Any
 * code that you would normally put into the top of the application's main()
 * routine should be put into this function. This is called before the clusters,
 * plugins, and the network are initialized so some functionality is not yet
 * available.
        Note: No callback in the Application Framework is
 * associated with resource cleanup. If you are implementing your application on
 * a Unix host where resource cleanup is a consideration, we expect that you
 * will use the standard Posix system calls, including the use of atexit() and
 * handlers for signals such as SIGTERM, SIGINT, SIGCHLD, SIGPIPE and so on. If
 * you use the signal() function to register your signal handler, please mind
 * the returned value which may be an Application Framework function. If the
 * return value is non-null, please make sure that you call the returned
 * function from your handler to avoid negating the resource cleanup of the
 * Application Framework itself.
 *
 */
extern void buttonReadEventHandler();
void emberAfMainInitCallback(void)
{

    emberAfAppPrintln("In emberAfMainInitCallback");    // add log test

    //�յ�ϵͳ��ʼ��
    eACMD_InitSystem();

    //��ʼ�� IO
    //RS485 control pin
    halGpioSetConfig(RS485_CONTROL_PIN, GPIOCFG_OUT);

    //����ͬ��������״̬:���������û��׼����,
    emberEventControlSetDelayMS(synchronizeAdapterStatus,SYNCHRONIZE_ADAPTER_STATUS_SHORT_CYCLE);

    //����ͬ�����ڻ�����״̬
    emberEventControlSetDelayMS(synchronizeIndoorConnectionStatus,SYNCHRONIZE_INDOOR_CONNECTION_STATUS_CYCLE);

    //����ͬ�����ڻ���״̬
    emberEventControlSetDelayMS(synchronizeIndoorStatus,SYNCHRONIZE_INDOOR_STATUS_CYCLE);

    //memcpy(rx_buffer0,0,RX_BUFFER_SIZE0);
    //����������Ϣ�¼�����
    emberEventControlSetDelayMS(customMessageEventControl,3000);



//���ް����ж�IO��ֱ������������ѯ
#ifndef BUTTON_INT
    emberEventControlSetDelayMS(buttonReadEventControl,0);
#endif

    //��������
    emberEventControlSetDelayMS(searchZnetEventControl,200);

}


/** @brief Finished
 *
 * This callback is fired when the network-find plugin is finished with the
 * forming or joining process. The result of the operation will be returned in
 * the status parameter.
 *
 * @param status   Ver.: always
 */
void emberAfPluginNetworkFindFinishedCallback(EmberStatus status)
{
    if(tryToStopScan)
    {
        emberAfAppPrintln("stop scan");
        tryToStopScan = FALSE;
        return ;
    }
    //��֤ѭ������ɨ��ɼ��������
    if(status != EMBER_SUCCESS)   //
    {
        if(!emberFormAndJoinIsScanning())
        {
            emberAfStartSearchForJoinableNetwork();
        }
    }
    emberAfAppPrintln("emberAfPluginNetworkFindFinishedCallback@%d",status);
}

/** @brief Stack Status
 *
 * This function is called by the application framework from the stack status
 * handler.  This callbacks provides applications an opportunity to be notified
 * of changes to the stack status and take appropriate action.  The return code
 * from this callback is ignored by the framework.  The framework will always
 * process the stack status after the callback returns.
 *
 * @param status   Ver.: always
 */
boolean emberAfStackStatusCallback(EmberStatus status)
{
    // If we go up or down, let the user know, although the down case shouldn't
    // happen.
    if (status == EMBER_NETWORK_UP)
    {
        emberAfAppPrintln("Network Up,Disable LED");
        //������ر���˸
        halMultiLedBlinkLedOff(0, BOARDLED0);
    }
    else if (status == EMBER_NETWORK_DOWN)
    {
        emberAfAppPrintln("Network Down,Flash LED");
        //��������,����20s
        halMultiLedBlinkBlink(200, 50,BOARDLED0);
        //21s����������
        emberEventControlSetDelayMS(searchZnetEventControl,21000);
    }
    return false;
}

/** @brief External Attribute Read
 *
 * Like emberAfExternalAttributeWriteCallback above, this function is called
 * when the framework needs to read an attribute that is not stored within the
 * Application Framework's data structures.
        All of the important
 * information about the attribute itself is passed as a pointer to an
 * EmberAfAttributeMetadata struct, which is stored within the application and
 * used to manage the attribute. A complete description of the
 * EmberAfAttributeMetadata struct is provided in
 * app/framework/include/af-types.h
        This function assumes that the
 * application is able to read the attribute, write it into the passed buffer,
 * and return immediately. Any attributes that require a state machine for
 * reading and writing are not really candidates for externalization at the
 * present time. The Application Framework does not currently include a state
 * machine for reading or writing attributes that must take place across a
 * series of application ticks. Attributes that cannot be read in a timely
 * manner should be stored within the Application Framework and updated
 * occasionally by the application code from within the
 * emberAfMainTickCallback.
        If the application was successfully able to
 * read the attribute and write it into the passed buffer, it should return a
 * value of EMBER_ZCL_STATUS_SUCCESS. Any other return value indicates the
 * application was not able to read the attribute.
 *
 * @param endpoint   Ver.: always
 * @param clusterId   Ver.: always
 * @param attributeMetadata   Ver.: always
 * @param manufacturerCode   Ver.: always
 * @param buffer   Ver.: always
 */
EmberAfStatus emberAfExternalAttributeReadCallback(int8u endpoint,
        EmberAfClusterId clusterId,
        EmberAfAttributeMetadata *attributeMetadata,
        int16u manufacturerCode,
        int8u *buffer)
{
    emberAfAppPrintln("ReadCallback");
    emberAfAppPrintln("EP:%d,CID:%x,AID:%x,",endpoint,clusterId,attributeMetadata->attributeId);
    halCommonDelayMilliseconds(60);
    return EMBER_ZCL_STATUS_FAILURE;
}

/** @brief External Attribute Write
 *
 * This function is called whenever the Application Framework needs to write an
 * attribute which is not stored within the data structures of the Application
 * Framework itself. One of the new features in Version 2 is the ability to
 * store attributes outside the Framework. This is particularly useful for
 * attributes that do not need to be stored because they can be read off the
 * hardware when they are needed, or are stored in some central location used by
 * many modules within the system. In this case, you can indicate that the
 * attribute is stored externally. When the framework needs to write an external
 * attribute, it makes a call to this callback.
        This callback is very
 * useful for host micros which need to store attributes in persistent memory.
 * Because each host micro (used with an Ember NCP) has its own type of
 * persistent memory storage, the Application Framework does not include the
 * ability to mark attributes as stored in flash the way that it does for Ember
 * SoCs like the EM35x. On a host micro, any attributes that need to be stored
 * in persistent memory should be marked as external and accessed through the
 * external read and write callbacks. Any host code associated with the
 * persistent storage should be implemented within this callback.
        All of
 * the important information about the attribute itself is passed as a pointer
 * to an EmberAfAttributeMetadata struct, which is stored within the application
 * and used to manage the attribute. A complete description of the
 * EmberAfAttributeMetadata struct is provided in
 * app/framework/include/af-types.h.
        This function assumes that the
 * application is able to write the attribute and return immediately. Any
 * attributes that require a state machine for reading and writing are not
 * candidates for externalization at the present time. The Application Framework
 * does not currently include a state machine for reading or writing attributes
 * that must take place across a series of application ticks. Attributes that
 * cannot be written immediately should be stored within the Application
 * Framework and updated occasionally by the application code from within the
 * emberAfMainTickCallback.
        If the application was successfully able to
 * write the attribute, it returns a value of EMBER_ZCL_STATUS_SUCCESS. Any
 * other return value indicates the application was not able to write the
 * attribute.
 *
 * @param endpoint   Ver.: always
 * @param clusterId   Ver.: always
 * @param attributeMetadata   Ver.: always
 * @param manufacturerCode   Ver.: always
 * @param buffer   Ver.: always
 */
EmberAfStatus emberAfExternalAttributeWriteCallback(int8u endpoint,
        EmberAfClusterId clusterId,
        EmberAfAttributeMetadata *attributeMetadata,
        int16u manufacturerCode,
        int8u *buffer)
{
    emberAfAppPrintln("WriteCallback");
    emberAfAppPrintln("EP:%d,CID:%x,AID:%x,",endpoint,clusterId,attributeMetadata->attributeId);
    //halCommonDelayMilliseconds(60);
    return EMBER_ZCL_STATUS_FAILURE;
}

/** @brief On/off Cluster On
 *
 *
 * @Description	: ���ڻ� On
 */
boolean emberAfOnOffClusterOnCallback(void)
{
    emberAfAppPrintln("OnCallback");
    EmberAfStatus status;
    bool currentValue, newValue;
    uint8_t endpoint;
    uint8_t command;
    tsSL_Msg sSL_Msg;

    //Get endpoint and comID
    endpoint = emberAfCurrentEndpoint();
    command = ZCL_ON_COMMAND_ID;
    emberAfAppPrintln("On/Off set value: %x %x", endpoint, command);

    if(endpoint == 1)			//Ӧ�ò㲻���� endpoint1 ���������
        return TRUE;

    // read current on/off value
    status = emberAfReadAttribute(endpoint,
                                  ZCL_ON_OFF_CLUSTER_ID,
                                  ZCL_ON_OFF_ATTRIBUTE_ID,
                                  CLUSTER_MASK_SERVER,
                                  (uint8_t *)&currentValue,
                                  sizeof(currentValue),
                                  NULL); // data type
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        emberAfAppPrintln("ERR: reading on/off %x", status);
        return status;
    }


    // if the value is already what we want to set it to then do nothing
    if ((currentValue && command == ZCL_ON_COMMAND_ID))
    {
        emberAfAppPrintln("On/off already set to new value");
    }
    else
    {
        // we either got a toggle, or an on when off, or an off when on,
        // so we need to swap the value
        newValue = !currentValue;
        emberAfAppPrintln("Toggle on/off from %x to %x", currentValue, newValue);

        // write the new on/off value
        status = emberAfWriteAttribute(endpoint,
                                       ZCL_ON_OFF_CLUSTER_ID,
                                       ZCL_ON_OFF_ATTRIBUTE_ID,
                                       CLUSTER_MASK_SERVER,
                                       (uint8_t *)&newValue,
                                       ZCL_BOOLEAN_ATTRIBUTE_TYPE);
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: writing on/off %x", status);
            return status;
        }
    }

    emberAfSendImmediateDefaultResponse(status);

    //������Ϣ����
    sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
    sSL_Msg.eMsgType = E_SL_MSG_SET_INDOOR_ONOFF_MODE_REQUEST;
    sSL_Msg.u8EndPoint = endpoint;			//endpoint �ķ�Χ�� 2 ~ 65
    sSL_Msg.u8RetryCount = 0;
    //record pre status
    sSL_Msg.uMessage.sSetIndoorOnOffModeRequest.ePreOnOff = (teIndoorOnOff_Mode)currentValue;
    sSL_Msg.uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress = endpoint + 0x0E;
    sSL_Msg.uMessage.sSetIndoorOnOffModeRequest.eOnOff = E_INDOOR_TURN_ON;

    if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
    {
        emberAfAppPrintln("MsgQueue is Full");
        ring_buffer_display(&sMsgTxRingBuffer);
    }
    return true;

}

/** @brief On/off Cluster Off
 *
 *
 * @Description	: ���ڻ� OFF
 */
boolean emberAfOnOffClusterOffCallback(void)
{
    emberAfAppPrintln("OffCallback");
    EmberAfStatus status = EMBER_ZCL_STATUS_SUCCESS;
    bool currentValue, newValue;
    uint8_t endpoint;
    uint8_t command;
    tsSL_Msg sSL_Msg;

    endpoint = emberAfCurrentEndpoint();
    command = ZCL_OFF_COMMAND_ID;

    emberAfAppPrintln("On/Off set value: %x %x", endpoint, command);

    if(endpoint == 1)			//don't handle cmd from endpoint1
        return TRUE;

    // read current on/off value
    status = emberAfReadAttribute(endpoint,
                                  ZCL_ON_OFF_CLUSTER_ID,
                                  ZCL_ON_OFF_ATTRIBUTE_ID,
                                  CLUSTER_MASK_SERVER,
                                  (uint8_t *)&currentValue,
                                  sizeof(currentValue),
                                  NULL); // data type
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        emberAfAppPrintln("ERR: reading on/off %x", status);
        return status;
    }


    // if the value is already what we want to set it to then do nothing
    if ((!currentValue && command == ZCL_OFF_COMMAND_ID))
    {
        emberAfAppPrintln("On/off already set to new value");
    }
    else
    {
        // we either got a toggle, or an on when off, or an off when on,
        // so we need to swap the value
        newValue = !currentValue;
        emberAfAppPrintln("Toggle on/off from %x to %x", currentValue, newValue);

        // write the new on/off value
        status = emberAfWriteAttribute(endpoint,
                                       ZCL_ON_OFF_CLUSTER_ID,
                                       ZCL_ON_OFF_ATTRIBUTE_ID,
                                       CLUSTER_MASK_SERVER,
                                       (uint8_t *)&newValue,
                                       ZCL_BOOLEAN_ATTRIBUTE_TYPE);
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: writing on/off %x", status);
            return status;
        }
    }
    emberAfSendImmediateDefaultResponse(status);

    //������Ϣ����
    sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
    sSL_Msg.eMsgType = E_SL_MSG_SET_INDOOR_ONOFF_MODE_REQUEST;
    sSL_Msg.u8EndPoint = endpoint;			//endpoint �ķ�Χ�� 2 ~ 65
    sSL_Msg.u8RetryCount = 0;
    sSL_Msg.uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress = endpoint + 0x0E;
    sSL_Msg.uMessage.sSetIndoorOnOffModeRequest.ePreOnOff = (teIndoorOnOff_Mode)currentValue;
    sSL_Msg.uMessage.sSetIndoorOnOffModeRequest.eOnOff = E_INDOOR_TURN_OFF;

    if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
    {
        emberAfAppPrintln("MsgQueue is Full");
        ring_buffer_display(&sMsgTxRingBuffer);
    }
    return TRUE;

}

/** @brief Thermostat Cluster Setpoint Raise Lower
 *
 *
 *
 * @param mode   Ver.: always
 * @param amount   Ver.: always
 * @Description	: ���ü���/�����¶�
 */
boolean emberAfThermostatClusterSetpointRaiseLowerCallback(int8u mode,int8s amount)
{
    uint8_t endpoint;
    EmberAfStatus status = EMBER_ZCL_STATUS_SUCCESS;
    tsSL_Msg sSL_Msg;
    int16_t currentValue;
    int16_t newValue;

    endpoint = emberAfCurrentEndpoint();
    emberAfAppPrintln("SetpointRaiseLowerCallback");
    emberAfAppPrintln("mode:%d amount:%d",mode,amount);
    emberAfAppPrintln("emAfZclBufferLen:%d",emAfZclBufferLen);

    if(endpoint == 1)
        return status;
    if(mode == 00 || mode == 02)		//heat
    {
        //��ȡ��ǰ���õ��¶�ֵ,�����ֵadd��ȥ
        status = emberAfReadAttribute(endpoint,
                                      ZCL_THERMOSTAT_CLUSTER_ID,
                                      ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID,
                                      CLUSTER_MASK_SERVER,
                                      (uint8_t *)&currentValue,
                                      sizeof(currentValue),
                                      NULL); // data type
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: reading heating %x", status);
            return status;
        }

        newValue = currentValue + amount*10;	//match unit

        // write the new heating value
        status = emberAfWriteAttribute(endpoint,
                                       ZCL_THERMOSTAT_CLUSTER_ID,
                                       ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID,
                                       CLUSTER_MASK_SERVER,
                                       (uint8_t *)&newValue,
                                       ZCL_INT16S_ATTRIBUTE_TYPE);
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: writing heating %x", status);
            return status;
        }

    }

    if(mode == 0x01 || mode == 0x02)	//cool
    {
        //��ȡ��ǰ���õ��¶�,�����ֵsub��
        status = emberAfReadAttribute(endpoint,
                                      ZCL_THERMOSTAT_CLUSTER_ID,
                                      ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                      CLUSTER_MASK_SERVER,
                                      (uint8_t *)&currentValue,
                                      sizeof(currentValue),
                                      NULL); // data type
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: reading heating %x", status);
            return status;
        }
        newValue = currentValue + amount*10;		//match unit

        // write the new cool value
        status = emberAfWriteAttribute(endpoint,
                                       ZCL_THERMOSTAT_CLUSTER_ID,
                                       ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                       CLUSTER_MASK_SERVER,
                                       (uint8_t *)&newValue,
                                       ZCL_INT16S_ATTRIBUTE_TYPE);
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: writing heating %x", status);
            return status;
        }

    }

    emberAfSendImmediateDefaultResponse(status);

    //�����趨�¶������Ϣ������
    //������Ϣ����
    sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
    sSL_Msg.eMsgType = E_SL_MSG_SET_INDOOR_SETING_TEMPE_REQUEST;
    sSL_Msg.u8EndPoint = endpoint;			//endpoint �ķ�Χ�� 2 ~ 65
    sSL_Msg.u8RetryCount = 0;
    sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u8IndoorAddress = endpoint + 0x0E;
    sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u8Mode = mode;
    sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u16PreSetingTemperature = currentValue;
    sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u16SetingTemperature = newValue;

    if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
    {
        emberAfAppPrintln("MsgQueue is Full");
        ring_buffer_display(&sMsgTxRingBuffer);
    }
    if (status != EMBER_ZCL_STATUS_SUCCESS)
        return false;
    else
        return true;
}

/** @brief Thermostat Cluster Server Pre Attribute Changed
 *
 * Server Pre Attribute Changed
 *
 * @param endpoint Endpoint that is being initialized  Ver.: always
 * @param attributeId Attribute to be changed  Ver.: always
 * @param attributeType Attribute type  Ver.: always
 * @param size Attribute size  Ver.: always
 * @param value Attribute value  Ver.: always
 */
EmberAfStatus emberAfThermostatClusterServerPreAttributeChangedCallback(int8u endpoint,
        EmberAfAttributeId attributeId,
        EmberAfAttributeType attributeType,
        int8u size,
        int8u *value)
{
    EmberAfStatus status = EMBER_ZCL_STATUS_SUCCESS;
    tsSL_Msg sSL_Msg;
    uint8_t currentValue;
    uint8_t newValue;

    int16_t currentTempValue;
    int16_t newTempValue;

    if(bWriteFromLocal)
        return EMBER_ZCL_STATUS_SUCCESS;


    emberAfAppPrintln("Thermostat,AttributeID:%d,Type:%d",attributeId,attributeType);

    if(endpoint == 1)
        return status;

    if(attributeId == ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID)	//
    {
        emberAfAppPrintln("%d,%d",value[0],value[1]);

        status = emberAfReadAttribute(endpoint,
                                      ZCL_THERMOSTAT_CLUSTER_ID,
                                      ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                      CLUSTER_MASK_SERVER,
                                      (uint8_t *)&currentTempValue,
                                      sizeof(currentTempValue),
                                      NULL); // data type
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: reading heating %x", status);
            return status;
        }
        newTempValue = value[0] + value[1] * 256;

        //�����趨�¶������Ϣ������
        //������Ϣ����
        sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
        sSL_Msg.eMsgType = E_SL_MSG_SET_INDOOR_SETING_TEMPE_REQUEST;
        sSL_Msg.u8EndPoint = endpoint;			//endpoint �ķ�Χ�� 2 ~ 65
        sSL_Msg.u8RetryCount = 0;
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u8IndoorAddress = endpoint + 0x0E;
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u8Mode = 0x01;	//cool
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u16PreSetingTemperature = currentTempValue;
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u16SetingTemperature = newTempValue;

        if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
        {
            emberAfAppPrintln("MsgQueue is Full");
            ring_buffer_display(&sMsgTxRingBuffer);
        }
    }

    if(attributeId == ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID)	//
    {
        emberAfAppPrintln("%d,%d",value[0],value[1]);

        //set temp
        status = emberAfReadAttribute(endpoint,
                                      ZCL_THERMOSTAT_CLUSTER_ID,
                                      ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID,
                                      CLUSTER_MASK_SERVER,
                                      (uint8_t *)&currentTempValue,
                                      sizeof(currentTempValue),
                                      NULL); // data type
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: reading heating %x", status);
            return status;
        }

        newTempValue = value[0] + value[1] * 256;

        //�����趨�¶������Ϣ������
        //������Ϣ����
        sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
        sSL_Msg.eMsgType = E_SL_MSG_SET_INDOOR_SETING_TEMPE_REQUEST;
        sSL_Msg.u8EndPoint = endpoint;			//endpoint �ķ�Χ�� 2 ~ 65
        sSL_Msg.u8RetryCount = 0;
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u8IndoorAddress = endpoint + 0x0E;
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u8Mode = 0x00;	//heat
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u16PreSetingTemperature = currentTempValue;
        sSL_Msg.uMessage.sSetIndoorSetingTempeRequest.u16SetingTemperature = newTempValue;

        if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
        {
            emberAfAppPrintln("MsgQueue is Full");
            ring_buffer_display(&sMsgTxRingBuffer);
        }
    }



    if(attributeId == ZCL_SYSTEM_MODE_ATTRIBUTE_ID)			//Fan mode
    {
        //��ȡ��ǰ����ģʽ
        status = emberAfReadAttribute(endpoint,
                                      ZCL_THERMOSTAT_CLUSTER_ID,
                                      ZCL_SYSTEM_MODE_ATTRIBUTE_ID,
                                      CLUSTER_MASK_SERVER,
                                      (uint8_t *)&currentValue,
                                      sizeof(currentValue),
                                      NULL); // data type
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: reading systemMode %x", status);
        }

        newValue = *value;

        //������Ϣ����
        sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
        sSL_Msg.eMsgType = E_SL_MSG_SET_INDOOR_WORKING_MODE_REQUEST;
        sSL_Msg.u8EndPoint = endpoint;			//endpoint �ķ�Χ�� 2 ~ 65
        sSL_Msg.u8RetryCount = 0;
        sSL_Msg.uMessage.sSetIndoorWorkingModeRequest.u8IndoorAddress = endpoint + 0x0E;
        sSL_Msg.uMessage.sSetIndoorWorkingModeRequest.ePreWorkingMode = (teIndoorWorking_Mode)currentValue;
        sSL_Msg.uMessage.sSetIndoorWorkingModeRequest.eWorkingMode = (teIndoorWorking_Mode)newValue;
        if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
        {
            emberAfAppPrintln("MsgQueue is Full");
            ring_buffer_display(&sMsgTxRingBuffer);
        }

    }

    return EMBER_ZCL_STATUS_SUCCESS;
}

/** @brief Fan Control Cluster Server Pre Attribute Changed
 *
 * Server Pre Attribute Changed
 *
 * @param endpoint Endpoint that is being initialized  Ver.: always
 * @param attributeId Attribute to be changed  Ver.: always
 * @param attributeType Attribute type  Ver.: always
 * @param size Attribute size  Ver.: always
 * @param value Attribute value  Ver.: always
 */
EmberAfStatus emberAfFanControlClusterServerPreAttributeChangedCallback(int8u endpoint,
        EmberAfAttributeId attributeId,
        EmberAfAttributeType attributeType,
        int8u size,
        int8u *value)
{
    EmberAfStatus status = EMBER_ZCL_STATUS_SUCCESS;
    tsSL_Msg sSL_Msg;
    uint8_t currentValue;
    uint8_t newValue;

    emberAfAppPrintln("FanControl,ED:%d,AID:%d",endpoint,attributeId);
    if(endpoint == 1)
        return status;

    if(bWriteFromLocal)
        return EMBER_ZCL_STATUS_SUCCESS;


    if(attributeId == ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID)			//Fan mode
    {
        //��ȡ��ǰ����ģʽ
        status = emberAfReadAttribute(endpoint,
                                      ZCL_FAN_CONTROL_CLUSTER_ID,
                                      ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID,
                                      CLUSTER_MASK_SERVER,
                                      (uint8_t *)&currentValue,
                                      sizeof(currentValue),
                                      NULL); // data type
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            emberAfAppPrintln("ERR: reading fanMode %x", status);
        }
        newValue = *value;

        //������Ϣ����
        sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
        sSL_Msg.eMsgType = E_SL_MSG_SET_INDOOR_BLOWINGRATE_REQUEST;
        sSL_Msg.u8EndPoint = endpoint;			//endpoint �ķ�Χ�� 2 ~ 65
        sSL_Msg.u8RetryCount = 0;
        sSL_Msg.uMessage.sSetIndoorBlowingRateRequest.u8IndoorAddress = endpoint + 0x0E;
        sSL_Msg.uMessage.sSetIndoorBlowingRateRequest.ePreAirQuantity = (teIndooAir_Quantity)currentValue;
        sSL_Msg.uMessage.sSetIndoorBlowingRateRequest.eAirQuantity = (teIndooAir_Quantity)newValue;
        if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
        {
            emberAfAppPrintln("MsgQueue is Full");
            ring_buffer_display(&sMsgTxRingBuffer);
        }
    }
    else		//other attribute,don't care
    {

    }

    return EMBER_ZCL_STATUS_SUCCESS;
}

/****************************************************************************
* Function  : customMessageEventFunction
* Description : ��Ϣ�¼������������������Ͷ����е�message
* Input Para  :
* Output Para :
* Return Value:
****************************************************************************/
void customMessageEventFunction(void)
{
    //emberAfAppPrintln("customMessage Handle");
    EmberAfStatus status;
    tsSL_Msg* sSL_Msg;
    uint8_t u8data[20];
    uint8_t u8dataLen = 0;
    teAdapterCMD_Status teCmdStatus = E_ADAPTER_ERR_ENUM_END;
    teAdapterCMD_Status teRspStatus = E_ADAPTER_ERR_ENUM_END;
    //�жϵ�ǰTX��״̬��
    if(eSL_Status == E_SL_IDLE)
    {
        //�����idle��ô�� tx buffer��ȡ����һ����Ϣ
        if(!ring_buffer_get_data( &sMsgTxRingBuffer, &sSL_Msg))
        {
            emberAfAppPrintln("\r\n\r\nSend CMD.....Type:%d,RetryCount:%d",sSL_Msg->eMsgType,sSL_Msg->u8RetryCount);
            eSL_Status = E_SL_BUSY;
            //�� RS485 ����ʹ�� pin
            eACMD_EnableTx();
            switch(sSL_Msg->eMsgType)
            {
                case E_SL_MSG_GET_ADAPTER_STATUS_REQUEST :  //���������״̬
                    //emberAfAppPrintln("Get Adapter Status");
                    teCmdStatus = eACMD_GetAdapterStatus();
                    break;

                case E_SL_MSG_GET_INDOOR_STATUS_REQUEST : //����ڻ�״̬������������Ի�ȡ��ĳ���ڻ�������״̬
                    //emberAfAppPrintln("\r\n\r\nSend CMD.....Type:%d,RetryCount:%d",sSL_Msg->uMessage.sGetIndoorStatusRequest.);
                    teCmdStatus = eACMD_GetIndoorStatus(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_ONOFF_MODE_REQUEST://����ڻ�����״̬
                    sSL_Msg->uMessage.sGetIndoorOnOffModeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorOnOffMode(sSL_Msg->uMessage.sGetIndoorOnOffModeRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_WORKING_MODE_REQUEST:		//��ȡ�ڻ�����ģʽ
                    sSL_Msg->uMessage.sGetIndoorWorkingModeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorWorkingMode(sSL_Msg->uMessage.sGetIndoorWorkingModeRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_SETING_TEMPE_REQUEST:		//��ȡ�ڻ��趨�¶�
                    sSL_Msg->uMessage.sGetIndoorSetingTempeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorSetTemperature(sSL_Msg->uMessage.sGetIndoorSetingTempeRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_INHALE_TEMPE_REQUEST:		//��ȡ�ڻ������¶�
                    sSL_Msg->uMessage.sGetIndoorInhaleTempeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorInhaleTempe(sSL_Msg->uMessage.sGetIndoorInhaleTempeRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_BLOWINGRATE_REQUEST:		//��ȡ����
                    sSL_Msg->uMessage.sGetIndoorBlowingRateRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorAirQuantity(sSL_Msg->uMessage.sGetIndoorBlowingRateRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_FORCE_STOP_STATUS_REQUEST:	//��ȡǿ��ֹͣ״̬
                    sSL_Msg->uMessage.sGetIndoorForceStopStatusRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorForceStopStatus(sSL_Msg->uMessage.sGetIndoorForceStopStatusRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_ERROR_CODE_REQUEST:		//��ȡ������
                    sSL_Msg->uMessage.sGetIndoorErrorCodeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorForceStopStatus(sSL_Msg->uMessage.sGetIndoorErrorCodeRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_FILTER_SIGNAL_REQUEST:		//��ȡ�����ź�
                    sSL_Msg->uMessage.sGetIndoorFilterSignalRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_GetIndoorFilterSignal(sSL_Msg->uMessage.sGetIndoorFilterSignalRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_COMMUNICATION_STATUS_REQUEST:		//��ȡͨ��״̬
                    teCmdStatus = eACMD_GetIndoorUnitCommunicateStatus();
                    break;
                /********************************************************************************************************************/
                /********************************************************************************************************************/
                /********************************************************************************************************************/
                /********************************************************************************************************************/
                ////////////////////////////////////////////////////////////////////////////////////////////////////
                case E_SL_MSG_SET_INDOOR_ONOFF_MODE_REQUEST:		//�����ڻ� on/off ״̬
                    sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_SetIndoorOnOffMode(sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress,sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.eOnOff);
                    break;

                case E_SL_MSG_SET_INDOOR_WORKING_MODE_REQUEST:		//�����ڻ�����ģʽ
                    sSL_Msg->uMessage.sSetIndoorWorkingModeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    //TODO: ���ǲ�ͬ���ڻ���֧�ֵ�ģʽ��ͬ����Ҫ����ת��
                    teCmdStatus = eACMD_SetIndoorWorkingMode(sSL_Msg->uMessage.sSetIndoorWorkingModeRequest.u8IndoorAddress,sSL_Msg->uMessage.sSetIndoorWorkingModeRequest.eWorkingMode);
                    break;

                case E_SL_MSG_SET_INDOOR_SETING_TEMPE_REQUEST:		//�����ڻ��¶�
                    sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    //TODO: �����¶ȵĵ�λ��ͬ����Ҫ����ת��
                    teCmdStatus = eACMD_SetIndoorSetTemperature(sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8IndoorAddress,sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u16SetingTemperature);
                    break;

                case E_SL_MSG_SET_INDOOR_BLOWINGRATE_REQUEST:		//�����ڻ�����
                    sSL_Msg->uMessage.sSetIndoorBlowingRateRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_SetIndoorAirQuantity(sSL_Msg->uMessage.sSetIndoorBlowingRateRequest.u8IndoorAddress,sSL_Msg->uMessage.sSetIndoorBlowingRateRequest.eAirQuantity);
                    break;

                case E_SL_MSG_RESET_INDOOR_FILTER_SIGNAL_REQUEST:	//�����ڻ������ź�
                    sSL_Msg->uMessage.sSetIndoorFilterSignalRequest.u8IndoorAddress = INDOORUNIT_BASE_ADDRESS + sSL_Msg->u8EndPoint -2;
                    teCmdStatus = eACMD_SetIndoorResetFilterSignal(sSL_Msg->uMessage.sSetIndoorFilterSignalRequest.u8IndoorAddress);
                    break;

                case E_SL_MSG_GET_INDOOR_CONNECTION_STATUS_REQUEST:   //����ڻ�����״̬
                    teCmdStatus = eACMD_GetIndoorUnitConnectionStatus();
                    break;

                case E_SL_MSG_GET_INDOOR_PERFORMANCE_INFO_REQUEST:		//��ȡ�ڻ�����
                    teCmdStatus = eACMD_GetIndoorUnitPerformanceInfomation(sSL_Msg->uMessage.sGetIndoorPerformanceInfoRequest.u8IndoorAddress);
                    break;

                default :
                    break;
            }
            //��������ɹ���ȴ� T1 ms  Լ30ms
            if(teCmdStatus == E_ADAPTER_SUCCESS)
            {
                emberAfAppPrintln("OK,Waiting Rsp\r\n");
                //ʹ��RS485����
                //eACMD_EnableRx();
                //clear uart rx
                USART_Tx_Rx_Count_0 = 0;
                emberEventControlSetDelayMS(customEventControl,8);  //  8ms֮���rx ,���ʱ�䲻׼ȷ  TODO:�Ƿ��а취���
                emberEventControlSetDelayMS(customMessageEventControl,T1);  //T1
            }
            else
            {
                //�����ʧ�� ��Ϊ����,�ȴ�һ�����ں��ٴη���
                emberAfAppPrintln("Cmd Send err:%d,Current Req:%d",teCmdStatus,sSL_Msg->eMsgType);
                eSL_Status = E_SL_IDLE;
                sSL_Msg->u8RetryCount ++;
                if(sSL_Msg->u8RetryCount > SL_MAX_RETRY)//�������ʧ�ܳ�����������ôɾ����������
                    ring_buffer_del_data(&sMsgTxRingBuffer);
                emberEventControlSetDelayMS(customMessageEventControl,T1);  //T1
            }
        }
        else  //buffer �ǿգ�1000ms֮���ټ����Ϣ����,����п�����Ϣ���жϵ���Ϣ�����ǿգ�Ҳ�������������������
        {
            emberEventControlSetDelayMS(customMessageEventControl,1000);
            return;
        }
    }
    else if(eSL_Status == E_SL_BUSY)  //�ȴ�Ӧ��
    {
        eSL_Status = E_SL_IDLE;
        emberAfAppPrintln("Handle Response.....");
        teRspStatus = eACMD_GetInComingResponse(u8data, &u8dataLen);  //��ȡresponse
        if(teRspStatus != E_ADAPTER_SUCCESS)
        {
            //û����ȷ�Ļ����Ӧ
            emberAfAppPrintln("Can't get response");    // add log test
        }
        else
        {
            //emberAfAppPrintln("Get correct response");    // add log test
        }

        //���ݵ�ǰ���͵���Ϣ��������Ӧ
        //����response,�������ȷ����Ӧ,��ô����Ϣ�Ӷ�����ɾ��,���������ô�ط�
        //��������ط�����,����Ϣɾ��������zigbee��Ӧ
        if(!ring_buffer_get_data( &sMsgTxRingBuffer, &sSL_Msg))
        {
            switch(sSL_Msg->eMsgType)
            {
                case E_SL_MSG_GET_ADAPTER_STATUS_REQUEST :      //****************************������״̬**************
                    sSL_Msg->u8RetryCount++;
                    //reponse��ȷ����������׼������
                    if((teRspStatus == E_ADAPTER_SUCCESS)&&((u8data[0] == DS2) && (u8data[1] == READ_INPUT_REGISTER) && (u8data[2] == 2 * ADAPTER_STATUS_REGISTER_NUMBERS))&&(u8data[4] & 0x01))
                    {
                        Adapter.bStatus = (u8data[4] & 0x01)?E_ADAPTER_READY:E_ADAPTER_NOREADY;
                        emberAfAppPrintln("get adapter Status:%d",Adapter.bStatus);
                        ring_buffer_del_data(&sMsgTxRingBuffer);
                    }//reponse����ȷ��������������û��׼����
                    else
                    {
                        if(sSL_Msg->u8RetryCount > SL_MAX_RETRY)
                        {
                            ring_buffer_del_data(&sMsgTxRingBuffer);
                        }
                    }
                    emberEventControlSetDelayMS(customMessageEventControl,T2);
                    break;
                case E_SL_MSG_GET_INDOOR_STATUS_REQUEST:      //��ȡ���ڻ�״̬
                    sSL_Msg->u8RetryCount++;
                    emberAfAppPrintln("get indoor status");

                    //////////////////////////////////////////////////////////
                    teRspStatus = E_ADAPTER_SUCCESS;
                    u8data[0] = DS2;
                    u8data[1] = READ_INPUT_REGISTER;
                    u8data[2] = 2*ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_NUMBERS;
                    //01 04 0C 50 00 42 02 01 18 00 00 01 0A 80 00 F6 FA

                    u8data[3] = 0x50;
                    u8data[4] = 0x01 ;
                    u8data[5] = 0x42 ;
                    u8data[6] = 0x02 ;
                    u8data[7] = 0x01 ;
                    u8data[8] = 0x18 ;
                    u8data[9] = 0x00 ;
                    u8data[10] = 0x00 ;
                    u8data[11] = 0x01 ;
                    u8data[12] = 0x0A ;
                    u8data[13] = 0x80 ;
                    u8data[14] = 0x00 ;

                    //////////////////////////////////////////////////////////
                    if((teRspStatus == E_ADAPTER_SUCCESS)&&((u8data[0] == DS2) && (u8data[1] == READ_INPUT_REGISTER) && (u8data[2] == 2*ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_NUMBERS)))
                    {
                        uint8_t u8reportDelay = 0;
                        //��ȡ���ڻ���״̬���뵱ǰ״̬�Ƚϣ������ͬ,�޸ı��ؼ�¼���ϱ���Ӧ����
                        uint16_t u16indoorStatus[ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_NUMBERS];
                        for(uint8_t i = 0; i < ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_NUMBERS; i++)
                        {
                            u16indoorStatus[i] = BUILD_UINT16(u8data[i*2+4], u8data[i*2+3]);
                        }

                        //�ж� fan mode|����**********************************************************
                        if(1)
                        {
                            uint8_t currentValue = 0;
                            EmberAfStatus status;
                            uint8_t newValue = (u16indoorStatus[0] & 0xF000)>>8;
                            //50 ����hh ����=0x50&0xf0 10=ll,20=l,30=m,40=h,50=hh
                            //0x00	Off
                            //0x01	Low
                            //0x02	Medium
                            //0x03	 High
                            //0x04	On
                            //0x05	Auto
                            //0x06	Smart
                            //0x07	Medium Low
                            //0x08	Medium High
                            //0x09~0xFE	Reserved
                            switch(newValue)
                            {
                                case 0x10:
                                    newValue = 0x01;
                                    break;
                                case 0x20:
                                    newValue = 0x07;
                                    break;
                                case 0x30:
                                    newValue = 0x02;
                                    break;
                                case 0x40:
                                    newValue = 0x08;
                                    break;
                                case 0x50:
                                    newValue = 0x03;
                                    break;
                                default :
                                    newValue = 0x01;
                                    break;
                            }
                            emberAfAppPrintln("newValue(fan mode): %d", newValue);
                            // read current fan mode
                            status = emberAfReadAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS + ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                          ZCL_FAN_CONTROL_CLUSTER_ID,
                                                          ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID,
                                                          CLUSTER_MASK_SERVER,
                                                          (uint8_t *)&currentValue,
                                                          sizeof(currentValue),
                                                          NULL); // data type
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: reading fan mode:%x,ED:%d", status,sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT);
                            }

                            if(currentValue != newValue)    //�ڻ� fan mode ״̬�и��£���Ҫ�ϱ�
                            {
                                emberAfAppPrintln("change fan mode %x to %x", currentValue, newValue);

                                bWriteFromLocal = TRUE;
                                // write the new air value
                                status = emberAfWriteAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                               ZCL_FAN_CONTROL_CLUSTER_ID,
                                                               ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&newValue,
                                                               ZCL_ENUM8_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing fan mode %x", status);
                                }
                                bWriteFromLocal = FALSE;
                                addReportingSchedule(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT, ZCL_FAN_CONTROL_CLUSTER_ID, ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID, u8reportDelay);
                            }
                            else
                            {
                                emberAfAppPrintln("fan mode no need to report");
                            }
                        }



                        //�ж� on/off |����
                        if(1)
                        {
                            uint8_t currentValue = 0;
                            EmberAfStatus status;
                            uint8_t newValue = u16indoorStatus[0] & 0x0001;
                            emberAfAppPrintln("newValue(onoff): %d", newValue);
                            // read current on/off value
                            status = emberAfReadAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                          ZCL_ON_OFF_CLUSTER_ID,
                                                          ZCL_ON_OFF_ATTRIBUTE_ID,
                                                          CLUSTER_MASK_SERVER,
                                                          (uint8_t *)&currentValue,
                                                          sizeof(currentValue),
                                                          NULL); // data type
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: reading on/off:%x,ED:%d", status,sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT);
                            }

                            if(currentValue != newValue)    //�ڻ�on/off״̬�и��£���Ҫ�ϱ�
                            {
                                // we either got a toggle, or an on when off, or an off when on,
                                // so we need to swap the value
                                emberAfAppPrintln("Toggle on/off from %x to %x", currentValue, newValue);

                                // write the new on/off value
                                status = emberAfWriteAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                               ZCL_ON_OFF_CLUSTER_ID,
                                                               ZCL_ON_OFF_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&newValue,
                                                               ZCL_BOOLEAN_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing on/off %x", status);
                                }
                                addReportingSchedule(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT, ZCL_ON_OFF_CLUSTER_ID, ZCL_ON_OFF_ATTRIBUTE_ID, u8reportDelay += 20);

                            }
                            else
                            {
                                emberAfAppPrintln("not need to report on/off");
                            }
                        }

                        //�ж� running mode |��תģʽ
                        if(1)
                        {
                            uint8_t currentValue = 0;
                            EmberAfStatus status;
                            uint8_t newValue = (u16indoorStatus[1] & 0x000F);
                            // read current running mode value
                            status = emberAfReadAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS + ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                          ZCL_THERMOSTAT_CLUSTER_ID,
                                                          ZCL_SYSTEM_MODE_ATTRIBUTE_ID,
                                                          CLUSTER_MASK_SERVER,
                                                          (uint8_t *)&currentValue,
                                                          sizeof(currentValue),
                                                          NULL); // data type
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: reading running mode %x", status);
                            }

                            if(currentValue != newValue)
                            {
                                emberAfAppPrintln("set New running mode %x", newValue);

                                bWriteFromLocal = TRUE;
                                // write the new fan direction value
                                status = emberAfWriteAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                               ZCL_THERMOSTAT_CLUSTER_ID,
                                                               ZCL_SYSTEM_MODE_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&newValue,
                                                               ZCL_ENUM8_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing Dir %x", status);
                                }
                                bWriteFromLocal = FALSE;
                                addReportingSchedule(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT, ZCL_THERMOSTAT_CLUSTER_ID, ZCL_SYSTEM_MODE_ATTRIBUTE_ID, u8reportDelay += 20);

                            }
                            else
                            {
                                emberAfAppPrintln("not need to report Van Direction");
                            }
                        }


                        //���ڻ��趨���¶� setting temperature
                        if(1)
                        {
                            uint16_t currentValue = 0;
                            EmberAfStatus status;
                            uint16_t newValue = u16indoorStatus[2];
                            // read current setting temperature value
                            status = emberAfReadAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS + ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                          ZCL_THERMOSTAT_CLUSTER_ID,
                                                          ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                                          CLUSTER_MASK_SERVER,
                                                          (uint8_t *)&currentValue,
                                                          sizeof(currentValue),
                                                          NULL); // data type
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: reading setting temperature %x", status);
                            }

                            if(currentValue != newValue)
                            {
                                emberAfAppPrintln("set New setting temperature %x", newValue);

                                bWriteFromLocal = TRUE;
                                // write the new setting temperature value
                                status = emberAfWriteAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                               ZCL_THERMOSTAT_CLUSTER_ID,
                                                               ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&newValue,
                                                               ZCL_INT16S_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing setting temperature %x", status);
                                }
                                //write the new setting temperature value
                                status = emberAfWriteAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                               ZCL_THERMOSTAT_CLUSTER_ID,
                                                               ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&newValue,
                                                               ZCL_INT16S_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing setting temperature %x", status);
                                }
                                bWriteFromLocal = FALSE;
                                //TODO:  need to report cool and heart????
                                addReportingSchedule(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT, ZCL_THERMOSTAT_CLUSTER_ID, ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID, u8reportDelay += 20);
                            }
                            else
                            {
                                emberAfAppPrintln("not need to report setting temperature");
                            }
                        }

                        //���ڵ��¶�(�����¶�)
                        if(1)
                        {
                            uint16_t currentValue = 0;
                            EmberAfStatus status;
                            uint16_t newValue = u16indoorStatus[2];
                            // read current setting temperature value
                            status = emberAfReadAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS + ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                          ZCL_THERMOSTAT_CLUSTER_ID,
                                                          ZCL_LOCAL_TEMPERATURE_ATTRIBUTE_ID,
                                                          CLUSTER_MASK_SERVER,
                                                          (uint8_t *)&currentValue,
                                                          sizeof(currentValue),
                                                          NULL); // data type
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: reading setting temperature %x", status);
                            }

                            if(currentValue != newValue)
                            {
                                emberAfAppPrintln("set New setting temperature %x", newValue);

                                bWriteFromLocal = TRUE;
                                // write the new setting temperature value
                                status = emberAfWriteAttribute(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT,
                                                               ZCL_THERMOSTAT_CLUSTER_ID,
                                                               ZCL_LOCAL_TEMPERATURE_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&newValue,
                                                               ZCL_INT16S_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing setting temperature %x", status);
                                }
                                bWriteFromLocal = FALSE;
                                addReportingSchedule(sSL_Msg->uMessage.sGetIndoorStatusRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS+ADAPTER_INDOOR_ENDPOINT_SHIFT, ZCL_THERMOSTAT_CLUSTER_ID, ZCL_LOCAL_TEMPERATURE_ATTRIBUTE_ID, u8reportDelay += 20);
                            }
                            else
                            {
                                emberAfAppPrintln("not need to report setting temperature");
                            }
                        }


                        //�¶ȴ�����״̬

                        //ɾ����������
                        ring_buffer_del_data(&sMsgTxRingBuffer);
                    }
                    else
                    {
                        //û�������յ� rsp,�ȴ���һ�������ط�
                        if(sSL_Msg->u8RetryCount > SL_MAX_RETRY)
                        {
                            //sSL_Msg->u8RetryCount = 0;
                            //ɾ����������
                            ring_buffer_del_data(&sMsgTxRingBuffer);
                        }
                    }
                    emberEventControlSetDelayMS(customMessageEventControl,T2);

                    break;
                case E_SL_MSG_GET_INDOOR_ONOFF_MODE_REQUEST:    //��ȡ���ڻ��Ŀ���״̬
                case E_SL_MSG_GET_INDOOR_FORCE_STOP_STATUS_REQUEST: //��ȡ���ڻ�ǿ��ֹͣ״̬
                case E_SL_MSG_GET_INDOOR_BLOWINGRATE_REQUEST:   //��ȡ���ڻ��ķ���
                    break;
                case E_SL_MSG_GET_INDOOR_WORKING_MODE_REQUEST:    //��ȡ���ڻ��Ĺ���ģʽ
                    break;
                case E_SL_MSG_GET_INDOOR_SETING_TEMPE_REQUEST:    //��ȡ���ڻ��¶��趨ֵ
                    break;
                case E_SL_MSG_GET_INDOOR_INHALE_TEMPE_REQUEST:    //��ȡ���ڻ��������¶�
                    break;
                case E_SL_MSG_GET_INDOOR_ERROR_CODE_REQUEST:    //��ȡ���ڻ��Ĵ�����
                    break;
                case E_SL_MSG_GET_INDOOR_FILTER_SIGNAL_REQUEST:   //��ȡ���ڻ��Ĺ����ź�
                    break;
                case E_SL_MSG_GET_INDOOR_COMMUNICATION_STATUS_REQUEST:  //��ȡ���ڻ���ͨ��״̬
                    break;
                /********************************************************************************************************************************/
                /********************************************************************************************************************************/
                /********************************************************************************************************************************/
                /********************************************************************************************************************************/
                /********************************************************************************************************************************/
                case E_SL_MSG_SET_INDOOR_ONOFF_MODE_REQUEST:    //�������ڻ��Ŀ���״̬
                    sSL_Msg->u8RetryCount++;
                    emberAfAppPrintln("sSL_Msg->u8RetryCount %d", sSL_Msg->u8RetryCount);
                    //�յ�response,����response��ȷ
                    if((teRspStatus == E_ADAPTER_SUCCESS)&&((u8data[0] == DS2) && (u8data[1] == PRESET_SINGLE_REGISTER) && ((sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS + ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS  == (uint16_t)u8data[2]*256 + u8data[3])))
                    {
                        //report on/off
                        addReportingSchedule(sSL_Msg->u8EndPoint,
                                             ZCL_ON_OFF_CLUSTER_ID,
                                             ZCL_ON_OFF_ATTRIBUTE_ID,
                                             0);
                        ring_buffer_del_data(&sMsgTxRingBuffer);
                    }
                    else
                    {
                        if(sSL_Msg->u8RetryCount > SL_MAX_RETRY)
                        {
                            /*	��������سɹ�
                            // failure, write the pre on/off value
                            status = emberAfWriteAttribute(sSL_Msg->u8EndPoint,
                                                           ZCL_ON_OFF_CLUSTER_ID,
                                                           ZCL_ON_OFF_ATTRIBUTE_ID,
                                                           CLUSTER_MASK_SERVER,
                                                           (uint8_t *)&sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.ePreOnOff,
                                                           ZCL_BOOLEAN_ATTRIBUTE_TYPE);
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: writing on/off %x", status);
                            }
                            */
                            //report on/off
                            addReportingSchedule(sSL_Msg->u8EndPoint,
                                                 ZCL_ON_OFF_CLUSTER_ID,
                                                 ZCL_ON_OFF_ATTRIBUTE_ID,
                                                 0);
                            ring_buffer_del_data(&sMsgTxRingBuffer);
                        }

                    }
                    emberEventControlSetDelayMS(customMessageEventControl,T2);
                    break;

                case E_SL_MSG_SET_INDOOR_WORKING_MODE_REQUEST:				//�����ڻ�����ģʽ
                    sSL_Msg->u8RetryCount++;
                    if((teRspStatus == E_ADAPTER_SUCCESS)&&((u8data[0] == DS2) && (u8data[1] == PRESET_SINGLE_REGISTER) && (((sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS + ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS + 1) == ((uint16_t)u8data[2]*256 + u8data[3]))))
                    {
                        //report systemMode
                        addReportingSchedule(sSL_Msg->u8EndPoint,
                                             ZCL_THERMOSTAT_CLUSTER_ID,
                                             ZCL_SYSTEM_MODE_ATTRIBUTE_ID,
                                             0);
                        ring_buffer_del_data(&sMsgTxRingBuffer);
                    }
                    else
                    {
                        if(sSL_Msg->u8RetryCount > SL_MAX_RETRY)
                        {
                            // write the Pre systemMode value
                            /*	��������سɹ�
                            bWriteFromLocal = TRUE;
                            status = emberAfWriteAttribute(sSL_Msg->u8EndPoint,
                                                           ZCL_THERMOSTAT_CLUSTER_ID,
                                                           ZCL_SYSTEM_MODE_ATTRIBUTE_ID,
                                                           CLUSTER_MASK_SERVER,
                                                           (uint8_t *)&sSL_Msg->uMessage.sSetIndoorWorkingModeRequest.ePreWorkingMode,
                                                           ZCL_ENUM8_ATTRIBUTE_TYPE);
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: writing systemMode %x", status);
                            }
                            bWriteFromLocal = FALSE;
                            */
                            //report systemMode
                            addReportingSchedule(sSL_Msg->u8EndPoint,
                                                 ZCL_THERMOSTAT_CLUSTER_ID,
                                                 ZCL_SYSTEM_MODE_ATTRIBUTE_ID,
                                                 0);
                            ring_buffer_del_data(&sMsgTxRingBuffer);
                        }
                    }
                    emberEventControlSetDelayMS(customMessageEventControl,T2);
                    break;

                case E_SL_MSG_SET_INDOOR_SETING_TEMPE_REQUEST:			//�ڻ��¶�����
                    sSL_Msg->u8RetryCount++;
                    if((teRspStatus == E_ADAPTER_SUCCESS)&&((u8data[0] == DS2) && (u8data[1] == PRESET_SINGLE_REGISTER) && ((sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS + ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS  == (uint16_t)u8data[2]<<8 + u8data[3])))
                    {
                        //report temperature
                        if(sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x00 || sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x02)
                        {
                            addReportingSchedule(sSL_Msg->u8EndPoint,
                                                 ZCL_THERMOSTAT_CLUSTER_ID,
                                                 ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID,
                                                 0);
                        }
                        if(sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x01 || sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x02)
                        {
                            addReportingSchedule(sSL_Msg->u8EndPoint,
                                                 ZCL_THERMOSTAT_CLUSTER_ID,
                                                 ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                                 10);
                        }
                        ring_buffer_del_data(&sMsgTxRingBuffer);
                    }
                    else
                    {
                        if(sSL_Msg->u8RetryCount > SL_MAX_RETRY)
                        {
                            //heat
                            sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u16PreSetingTemperature /=10;
                            if(sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x00 || sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x02)
                            {
                                /*	��������سɹ�
                                bWriteFromLocal = TRUE;
                                // write the pre heating value
                                status = emberAfWriteAttribute(sSL_Msg->u8EndPoint,
                                                               ZCL_THERMOSTAT_CLUSTER_ID,
                                                               ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u16PreSetingTemperature,
                                                               ZCL_INT16S_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing heating %x", status);
                                }
                                bWriteFromLocal = FALSE;
                                */
                            }
                            //cool
                            if(sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x01 || sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x02)
                            {
                                /*	��������سɹ�
                                bWriteFromLocal = TRUE;
                                // write tge new heating value
                                status = emberAfWriteAttribute(sSL_Msg->u8EndPoint,
                                                               ZCL_THERMOSTAT_CLUSTER_ID,
                                                               ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                                               CLUSTER_MASK_SERVER,
                                                               (uint8_t *)&sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u16PreSetingTemperature,
                                                               ZCL_INT16S_ATTRIBUTE_TYPE);
                                if (status != EMBER_ZCL_STATUS_SUCCESS)
                                {
                                    emberAfAppPrintln("ERR: writing heating %x", status);
                                }
                                bWriteFromLocal = FALSE;
                                */
                            }

                            //report temperature
                            if(sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x00 || sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x02)
                            {
                                addReportingSchedule(sSL_Msg->u8EndPoint,
                                                     ZCL_THERMOSTAT_CLUSTER_ID,
                                                     ZCL_OCCUPIED_HEATING_SETPOINT_ATTRIBUTE_ID,
                                                     0);
                            }
                            if(sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x01 || sSL_Msg->uMessage.sSetIndoorSetingTempeRequest.u8Mode == 0x02)
                            {
                                addReportingSchedule(sSL_Msg->u8EndPoint,
                                                     ZCL_THERMOSTAT_CLUSTER_ID,
                                                     ZCL_OCCUPIED_COOLING_SETPOINT_ATTRIBUTE_ID,
                                                     10);
                            }
                            ring_buffer_del_data(&sMsgTxRingBuffer);
                        }
                    }
                    emberEventControlSetDelayMS(customMessageEventControl,T2);
                    break;

                case E_SL_MSG_SET_INDOOR_BLOWINGRATE_REQUEST:		//�����ڻ�����
                    sSL_Msg->u8RetryCount++;
                    if((teRspStatus == E_ADAPTER_SUCCESS)&&((u8data[0] == DS2) && (u8data[1] == PRESET_SINGLE_REGISTER) && ((sSL_Msg->uMessage.sSetIndoorOnOffModeRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS + ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS  == (uint16_t)u8data[2]<<8 + u8data[3])))
                    {
                        //report fanMode
                        addReportingSchedule(sSL_Msg->u8EndPoint,
                                             ZCL_FAN_CONTROL_CLUSTER_ID,
                                             ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID,
                                             0);
                        ring_buffer_del_data(&sMsgTxRingBuffer);
                    }
                    else
                    {
                        if(sSL_Msg->u8RetryCount > SL_MAX_RETRY)
                        {
                            /*��������سɹ�
                            bWriteFromLocal = TRUE;
                            // write tge new fanMode value
                            status = emberAfWriteAttribute(sSL_Msg->u8EndPoint,
                                                           ZCL_FAN_CONTROL_CLUSTER_ID,
                                                           ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID,
                                                           CLUSTER_MASK_SERVER,
                                                           (uint8_t *)&sSL_Msg->uMessage.sSetIndoorBlowingRateRequest.ePreAirQuantity,
                                                           ZCL_ENUM8_ATTRIBUTE_TYPE);
                            if (status != EMBER_ZCL_STATUS_SUCCESS)
                            {
                                emberAfAppPrintln("ERR: writing fanMode %x", status);
                            }
                            bWriteFromLocal = FALSE;
                            */
                            //report fanMode
                            addReportingSchedule(sSL_Msg->u8EndPoint,
                                                 ZCL_FAN_CONTROL_CLUSTER_ID,
                                                 ZCL_FAN_CONTROL_FAN_MODE_ATTRIBUTE_ID,
                                                 0);
                            ring_buffer_del_data(&sMsgTxRingBuffer);
                        }
                    }
                    emberEventControlSetDelayMS(customMessageEventControl,T2);
                    break;

                case E_SL_MSG_RESET_INDOOR_FILTER_SIGNAL_REQUEST:

                    break;

                case E_SL_MSG_GET_INDOOR_CONNECTION_STATUS_REQUEST: //���ڻ�������Ϣ
                    sSL_Msg->u8RetryCount++;
                    if(teRspStatus == E_ADAPTER_SUCCESS)
                    {
                        if((u8data[0] == DS2) && (u8data[1] == READ_INPUT_REGISTER) && (u8data[2] == 2*ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS))
                        {
                            uint8_t u8deviceNum = 0;
                            ring_buffer_del_data(&sMsgTxRingBuffer);    //ɾ��������Ϣ
                            emberAfAppPrintln("Get Indoor Connect");
                            for(uint8_t i = 0; i<ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS; i++)
                            {
                                Adapter.u16IndoorConnectionStatus[i] = BUILD_UINT16(u8data[i*2+4], u8data[i*2+3]);
                            }
#if 0			//IRACC��������ʱ����Ҫ��ȡ���ڻ�������
                            //����������������ڻ�����������
                            for(uint8_t i = 0; i<ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS; i++)
                            {
                                for(uint16_t j = 0; j < 8*sizeof(uint16_t); j++)
                                {
                                    if(Adapter.u16IndoorConnectionStatus[i] & ( 1<<j ))
                                    {
                                        u8deviceNum++;
                                        tsSL_Msg sSL_Msg;
                                        //�����ȡ�ڻ�����״̬����
                                        sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
                                        sSL_Msg.eMsgType = E_SL_MSG_GET_INDOOR_PERFORMANCE_INFO_REQUEST;
                                        sSL_Msg.u8EndPoint = 0xFF;
                                        sSL_Msg.u8RetryCount = 0;
                                        sSL_Msg.uMessage.sGetIndoorPerformanceInfoRequest.u8IndoorAddress = (i+1)*INDOORUNIT_BASE_ADDRESS+j;
                                        if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
                                        {
                                            emberAfAppPrintln("MsgQueue is Full");
                                            ring_buffer_display(&sMsgTxRingBuffer);
                                        }
                                    }
                                }
                            }
#endif
                            //���±��ص� endpoint
                            eACMD_UpdateEndpointVisible();
                            emberAfAppPrintln("u8deviceNum:%d",u8deviceNum);
                            //�鿴�������ڻ���״̬
                            for(uint8_t i = 0; i < ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS; i++)
                            {
                                for(uint16_t j = 0; j < 8*sizeof(uint16_t); j++)
                                {
                                    if(Adapter.u16IndoorConnectionStatus[i] & ( 1<<j ))
                                    {
                                        tsSL_Msg sSL_Msg;
                                        //�����ȡ�ڻ�״̬����
                                        sSL_Msg.eMsgStatus = E_MSG_WAIT_SEND;
                                        sSL_Msg.eMsgType = E_SL_MSG_GET_INDOOR_STATUS_REQUEST;
                                        sSL_Msg.u8EndPoint = i*0x10+j+ADAPTER_INDOOR_ENDPOINT_SHIFT;
                                        sSL_Msg.u8RetryCount = 0;
                                        sSL_Msg.uMessage.sGetIndoorPerformanceInfoRequest.u8IndoorAddress = (i+1)*INDOORUNIT_BASE_ADDRESS+j;
                                        emberAfAppPrintln("u8IndoorAddress:0x%x",sSL_Msg.uMessage.sGetIndoorPerformanceInfoRequest.u8IndoorAddress);
                                        if(!ring_buffer_write(&sMsgTxRingBuffer, &sSL_Msg))
                                        {
                                            emberAfAppPrintln("MsgQueue is Full");  //  TODO:�豸�����࣬��full�Ŀ���
                                            ring_buffer_display(&sMsgTxRingBuffer);
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            //response is not match
                            emberAfAppPrintln("Rsp not match");
                            if(sSL_Msg->u8RetryCount >SL_MAX_RETRY)
                            {
                                ring_buffer_del_data(&sMsgTxRingBuffer);
                            }
                        }
                    }
                    else
                    {
                        if(sSL_Msg->u8RetryCount >SL_MAX_RETRY)
                        {
                            //��������ظ�����,��������
                            ring_buffer_del_data(&sMsgTxRingBuffer);
                        }
                    }

                    //�ȴ�T2ms������һ������
                    emberEventControlSetDelayMS(customMessageEventControl,T2);
                    break;
                case E_SL_MSG_GET_INDOOR_PERFORMANCE_INFO_REQUEST:      //���ڻ���������Ϣ
                    sSL_Msg->u8RetryCount++;
                    if(teRspStatus == E_ADAPTER_SUCCESS)
                    {
                        if((u8data[0] == DS2) && (u8data[1] == READ_INPUT_REGISTER) && (u8data[2] == 2*ADAPTER_INDOORUNIT_PERFORMANCE_INFORMATION_REGISTER_NUMBERS))
                        {
                            ring_buffer_del_data(&sMsgTxRingBuffer);    //ɾ��������Ϣ
                            emberAfAppPrintln("Get Indoor Performance");
                            for(uint8_t i = 0; i<ADAPTER_INDOORUNIT_PERFORMANCE_INFORMATION_REGISTER_NUMBERS; i++)
                            {
                                Adapter.Indoor[sSL_Msg->uMessage.sGetIndoorPerformanceInfoRequest.u8IndoorAddress-INDOORUNIT_BASE_ADDRESS].u16PerformanceInfo[i] = BUILD_UINT16(u8data[i*2+4], u8data[i*2+3]);
                            }
                        }
                        else
                        {
                            //response is not match
                            emberAfAppPrintln("Rsp not match");
                            if(sSL_Msg->u8RetryCount >SL_MAX_RETRY)
                            {
                                //��������ظ�����,��������
                                sSL_Msg->u8RetryCount = 0;
                            }
                        }
                    }

                    //�ȴ�T2ms������һ������
                    emberEventControlSetDelayMS(customMessageEventControl,T2);
                    break;

                default :
                    break;
            }
        }
        else
        {
            emberAfAppPrintln("no Msg is Waiting");
            //û����Ϣ�ڵȴ�response
        }
    }
    else
    {

    }
//���TX��״̬�ǵȴ�Ӧ����ô�����д���
//emberEventControlSetDelayMS(customMessageEventControl,1000);

    return ;
}
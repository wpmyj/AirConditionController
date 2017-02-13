/***************************************************************************************
****************************************************************************************
* FILE		: adapter-command.c
* Description	:
*
* Copyright (c) 2016 by ORVIBO. All Rights Reserved.
*
* History:
* Version		Name       		Date			Description
   0.1		sven	2016/11/03	Initial Version

****************************************************************************************
****************************************************************************************/
#include "app/framework/include/af.h"
#include "stack/include/ember.h"
#include "app/framework/util/util.h"
#include "AirConditionController_endpoint_config.h"
#include "adapter-command.h"
#include "crc16-modbus.h"
#include "RingBufferUtils.h"
#include "attributeReporting.h"



teSL_MsgType 	teSL_CurrentREQ = E_SL_MSG_NONE;		//���ڵ�ǰ���������
teSL_MsgType 	teSL_CurrentRSP = E_SL_MSG_NONE;		//���ڵ�ǰ�ȴ�����Ӧ
teSL_Status		eSL_Status	= E_SL_IDLE;	//�����ǵ�ǰ״̬




teAdapter Adapter;			//�������豸
tsSL_Msg sSL_MsgTxQueue[SL_MSG_TX_SIZE];		//������Ϣ���Ͷ���

tsMsgRingBuffer sMsgTxRingBuffer;			//����TXѭ������

/****************************************************************************
* Function	: eACMD_InitSystem
* Description	: ��ʼ��������
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_InitSystem()
{
    emberAfAppPrintln("eACMD_InitSystem");

    memset(&Adapter,0,sizeof(teAdapter));

    //��ʼ�����Ͷ���
    ring_buffer_init(&sMsgTxRingBuffer, sSL_MsgTxQueue, SL_MSG_TX_SIZE);
    ///////////////////////////////////////////////
    Adapter.u16IndoorConnectionStatus[0] = 0x0101;
    Adapter.u16IndoorConnectionStatus[1] = 0x0000;
    Adapter.u16IndoorConnectionStatus[2] = 0x0000;
    Adapter.u16IndoorConnectionStatus[3] = 0x0000;
	Adapter.bStatus = E_ADAPTER_READY;
    eACMD_UpdateEndpointVisible();
    //////////////////////////////////////////////
    // 1.��ȡ��������״̬
    //eACMD_GetAdapterStatus();

    //if(Adapter.bStatus == E_ADAPTER_READY)
    //{
    // 2.��ȡ���ڻ�������״̬
    //    eACMD_GetIndoorUnitConnectionStatus();

    // 3.��ȡ�������������ڻ�������(��������״̬)
    //Adapter.u16IndoorConnectionStatus;
    //eACMD_GetIndoorUnitPerformanceInfomation();
    //}
    //else
    //{
    //������û��׼����
    //}

    return E_ADAPTER_SUCCESS;
}



/****************************************************************************
* Function	: eACMD_UpdateSystem
* Description	: ����������ϵͳ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_UpdateSystem()
{

    return E_ADAPTER_SUCCESS;
}


/****************************************************************************
* Function	: eACMD_UpdateEndpointVisible
* Description	: ���� endpoint visible
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_UpdateEndpointVisible()
{
    uint8_t i = 0;
    uint8_t j = 0;

    if(Adapter.bStatus == E_ADAPTER_READY)
    {
        for(i = 0; i<ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS; i++)
        {
            for(j = 0; j < 16 ; j++)
            {
                emberAfEndpointEnableDisable(i*16+j+ADAPTER_INDOOR_ENDPOINT_SHIFT, ((Adapter.u16IndoorConnectionStatus[i]>>j)&0x01)?TRUE:FALSE);
                if(((Adapter.u16IndoorConnectionStatus[i]>>j)&0x01))
                    emberAfAppPrintln("ok:%d",i*16+j+ADAPTER_INDOOR_ENDPOINT_SHIFT);
            }
        }
        emberAfAppPrintln("i:%d,j:%d",i,j);

        return E_ADAPTER_SUCCESS;
    }
    else
    {
        emberAfAppPrintln("Adapter not ready");
        return E_ADAPTER_FAIL;
    }
}

/****************************************************************************
* Function	: eACMD_AdapterCommandSend
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_AdapterCommandSend(uint8_t u8SlaveAddr,uint8_t u8FunctionCode,uint16_t u16RegisterAddr,uint8_t *pu8Data,uint8_t u8DataLen)
{
    uint8_t u8data[20];
    uint8_t u8dataLen = 0;
    uint16_t u16CRC;
    EmberStatus status;
    u8data[u8dataLen++] = u8SlaveAddr;

    u8data[u8dataLen++] = u8FunctionCode;

    u8data[u8dataLen++] = u16RegisterAddr>>8;

    u8data[u8dataLen++] = (u16RegisterAddr & 0xff);

    if(u8DataLen>12)
        return E_ADAPTER_ERR_PARAMETER_RANGE;

    if(pu8Data == NULL)
        return E_ADAPTER_ERR_PARAMETER_NULL;

    memcpy(&u8data[u8dataLen],pu8Data,u8DataLen);
    u8dataLen += u8DataLen;

    u16CRC = CRC16 (u8data, u8dataLen);

    u8data[u8dataLen++] = (u16CRC & 0xff);

    u8data[u8dataLen++] = (u16CRC >> 8);

    status = emberSerialWriteData(UART_PORT, u8data, u8dataLen);
    if( status == EMBER_SUCCESS)
        return E_ADAPTER_SUCCESS;
    else
    {
        emberAfAppPrintln("uart send err:%d",status);
        for(int i = 0 ; i < u8dataLen; i++)
        {
            emberAfAppPrintln("u8data[%d]:%d",i,u8data[i]);
        }
        return E_ADAPTER_FAIL;
    }
}





/****************************************************************************
* Function	: eACMD_AdapterReadInputRegister
* Description	: ������Ĵ�����ֵ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_AdapterReadInputRegister(uint16_t u16RegisterAddr,uint16_t u16RegisterNum)
{
    uint8_t u8data[2];
    uint8_t u8dataLen = 0;

    if(u16RegisterNum > READ_WRITE_REGISTER_NUM_MAX)
        return E_ADAPTER_ERR_PARAMETER_RANGE;
    u8data[u8dataLen++] = u16RegisterNum>>8;
    u8data[u8dataLen++] = (u16RegisterNum & 0xff);

    return eACMD_AdapterCommandSend(ADAPTER_ADDRESS,
                                    READ_INPUT_REGISTER,
                                    u16RegisterAddr,
                                    u8data,
                                    u8dataLen);
}



/****************************************************************************
* Function	: eACMD_AdapterPresetSingleRegister
* Description	: Ԥ�õ�������Ĵ�����ֵ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_AdapterPresetSingleRegister(uint16_t u16RegisterAddr,uint16_t u16RegisterData)
{
    uint8_t u8data[2];
    uint8_t u8dataLen = 0;
    u8data[u8dataLen++] = u16RegisterData>>8;
    u8data[u8dataLen++] = (u16RegisterData & 0xff);
    return eACMD_AdapterCommandSend(ADAPTER_ADDRESS,
                                    PRESET_SINGLE_REGISTER,
                                    u16RegisterAddr,
                                    u8data,
                                    u8dataLen);
}



/****************************************************************************
* Function	: eACMD_AdapterPresetMultipleRegister
* Description	: Ԥ�ö�����ּĴ�����ֵ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_AdapterPresetMultipleRegister(uint16_t u16RegisterAddr,uint16_t u16RegisterNum,uint8_t u8DataSize,uint8_t *pu8Data)
{
    uint8_t u8data[11];
    uint8_t u8dataLen = 0;

    u8data[u8dataLen++] = u16RegisterNum >> 8;
    u8data[u8dataLen++] = (u16RegisterNum & 0xff);

    if(u8DataSize > 2*READ_WRITE_REGISTER_NUM_MAX)
        return E_ADAPTER_ERR_PARAMETER_RANGE;
    u8data[u8dataLen++] = u8DataSize;

    memcpy(&u8data[u8dataLen],pu8Data,u8DataSize);
    u8dataLen += u8DataSize;

    return eACMD_AdapterCommandSend(ADAPTER_ADDRESS,
                                    PRESET_MULTIPLE_REGISTER,
                                    u16RegisterAddr,
                                    u8data,
                                    u8dataLen);
}


//���
/****************************************************************************
* Function	: eACMD_GetAdapterStatus
* Description	: ��ȡ��������״̬
* Input Para	:
* Output Para	:
* Return Value: ׼����/û��׼����
****************************************************************************/
teAdapterCMD_Status eACMD_GetAdapterStatus()
{
    if(!eACMD_AdapterReadInputRegister(ADAPTER_STATUS_REGISTER_ADDRESS + ADAPTER_INPUT_REGISTER_BASE_ADDRESS,
                                       ADAPTER_STATUS_REGISTER_NUMBERS))
    {
        return E_ADAPTER_SUCCESS;
    }
    return E_ADAPTER_FAIL;
}

/****************************************************************************
* Function	: eACMD_GetIndoorUnitConnectionStatus
* Description	: ��ȡ���ڻ�������״̬
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorUnitConnectionStatus()
{
    if(!eACMD_AdapterReadInputRegister(ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_BASE_ADDRESS + ADAPTER_INPUT_REGISTER_BASE_ADDRESS,
                                       ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS))
    {
        return E_ADAPTER_SUCCESS;
    }
    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_GetIndoorUnitCommunicateStatus
* Description	: ��ȡ���ڻ���ͨ��״̬
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorUnitCommunicateStatus()
{
    if(!eACMD_AdapterReadInputRegister(
                ADAPTER_INDOORUNIT_COMMUNICATION_STATUS_REGISTER_BASE_ADDRESS + ADAPTER_INPUT_REGISTER_BASE_ADDRESS,
                ADAPTER_INDOORUNIT_COMMUNICATION_STATUS_REGISTER_NUMBERS))
    {
        return E_ADAPTER_SUCCESS;
    }
    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_GetIndoorUnitPerformanceInfomation
* Description	: �������ڻ��ĵ�ַ����ȡ���ڻ���������Ϣ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorUnitPerformanceInfomation(uint8_t u8indoorAddr)
{
    if(u8indoorAddr<0x10 || u8indoorAddr >0x4F)
        return E_ADAPTER_ERR_PARAMETER_RANGE;
    if(!eACMD_AdapterReadInputRegister(
                (u8indoorAddr-0x10) * ADAPTER_INDOORUNIT_PERFORMANCE_INFORMATION_REGISTER_NUMBERS \
                + ADAPTER_INDOORUNIT_PERFORMANCE_INFORMATION_REGISTER_BASE_ADDRESS \
                + ADAPTER_INPUT_REGISTER_BASE_ADDRESS,
                ADAPTER_INDOORUNIT_PERFORMANCE_INFORMATION_REGISTER_NUMBERS))
    {
        return E_ADAPTER_SUCCESS;
    }
    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_ModifyAdapterSlaveAddress
* Description	: �޸��������ӻ���ַ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_ModifyAdapterSlaveAddress(uint8_t u8slaveAddress)
{
    uint16_t u16RegisterAddr = 0xAA00 | u8slaveAddress;
    uint8_t u8data[1];
    return eACMD_AdapterCommandSend(ADAPTER_ADDRESS,
                                    PRESET_SINGLE_REGISTER,
                                    u16RegisterAddr,
                                    u8data,
                                    0);
}

/****************************************************************************
* Function	: eACMD_GetIndoorStatus
* Description	: ��ȡ�����ڻ���״̬
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorStatus(uint8_t u8indoorAddr)
{
    if(u8indoorAddr<0x10 || u8indoorAddr >0x4F)
        return E_ADAPTER_ERR_PARAMETER_RANGE;

    if(!eACMD_AdapterReadInputRegister((u8indoorAddr-0x10) * ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_NUMBERS + ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_BASE_ADDRESS + ADAPTER_INPUT_REGISTER_BASE_ADDRESS,
                                       ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_NUMBERS))
    {
        return E_ADAPTER_SUCCESS;
    }
    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_GetIndoorOnOffMode
* Description	: ��ȡ���ڻ�����ת/ֹͣ״̬
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorOnOffMode(uint8_t u8Addr)
{

    //eACMD_AdapterReadInputRegister(uint16_t u16RegisterAddr, uint16_t u16RegisterNum);


    //eACMD_AdapterPresetSingleRegister(uint16_t u16RegisterAddr, uint16_t u16RegisterData);
    return E_ADAPTER_SUCCESS;
}


/****************************************************************************
* Function	: eACMD_GetIndoorAirQuantity
* Description	: ��ȡ���ڻ��ķ���
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorAirQuantity(uint8_t u8Addr)
{
    return E_ADAPTER_SUCCESS;
}

/****************************************************************************
* Function	: eACMD_GetIndoorWorkingMode
* Description	: ��ȡ���ڻ��Ĺ���ģʽ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorWorkingMode(uint8_t u8Addr)
{
    return E_ADAPTER_SUCCESS;
}

/****************************************************************************
* Function	: eACMD_GetIndoorSetTemperature
* Description	: ��ȡ���ڻ��趨���¶�
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorSetTemperature(uint8_t u8Addr)
{
    return E_ADAPTER_SUCCESS;
}

/****************************************************************************
* Function	: eACMD_GetIndoorResetFilterSignal
* Description	: ��ȡ���ڻ��ĸ�λ�����ź�
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorResetFilterSignal(uint8_t u8Addr)
{

    return E_ADAPTER_SUCCESS;
}

/****************************************************************************
* Function	: eACMD_GetIndoorInhaleTempe
* Description	: ��ȡ���ڻ��������¶�(�����¶�)
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorInhaleTempe(uint8_t u8Addr)
{

    return E_ADAPTER_SUCCESS;
}

/****************************************************************************
* Function	: eACMD_GetIndoorForceStopStatus
* Description	: ��ȡ���ڻ���ǿ��ֹͣ״̬
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorForceStopStatus(uint8_t u8Addr)
{
    return E_ADAPTER_SUCCESS;
}

/****************************************************************************
* Function	: eACMD_GetIndoorFilterSignal
* Description	: ��ȡ���ڻ��Ĺ����ź�
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetIndoorFilterSignal(uint8_t u8Addr)
{

    return E_ADAPTER_SUCCESS;
}


/****************************************************����***********************************************/
/****************************************************����***********************************************/
/****************************************************����***********************************************/
/****************************************************����***********************************************/
/****************************************************************************
* Function	: eACMD_SetIndoorWorkMode
* Description	: �������ڻ�����ת/ֹͣ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_SetIndoorOnOffMode(uint8_t u8Addr,teIndoorOnOff_Mode teOnOffMode)
{
    if(u8Addr < 0x10 || u8Addr >0x4F)
        return E_ADAPTER_ERR_PARAMETER_RANGE;

    if(!eACMD_AdapterPresetSingleRegister(ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS+(u8Addr-0x10)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS, 0xFF60|teOnOffMode))
    {
        return E_ADAPTER_SUCCESS;
    }
    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_SetIndoorAirQuantity
* Description	: �����ڻ��ķ���(LL,L,M,H,HH��ȡ�������ڻ�������)
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_SetIndoorAirQuantity(uint8_t u8Addr,teIndooAir_Quantity teAirQuantity)
{
    uint16_t airQuantity = teAirQuantity<<12;
    if(u8Addr < 0x10 || u8Addr >0x4F)
        return E_ADAPTER_ERR_PARAMETER_RANGE;

    if(!eACMD_AdapterPresetSingleRegister(ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS+(u8Addr-0x10)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS, airQuantity|0x00FF))
    {
        return E_ADAPTER_SUCCESS;
    }

    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_SetIndoorAirDirection
* Description	: �����ڻ��ķ���:�ڶ���ֹͣ��ҶƬ�ķ���(ȡ�������ڻ�������)
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_SetIndoorAirDirection(uint8_t u8Addr,teIndooAir_Direction teAirDirection)
{

    return E_ADAPTER_SUCCESS;
}


/****************************************************************************
* Function	: eACMD_SetIndoorWorkingMode
* Description	: �����ڻ�����תģʽ:���䣬���ȣ��ͷ磬��ʪ���Զ�(ȡ�������ڻ�������)
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_SetIndoorWorkingMode(uint8_t u8Addr,teIndoorWorking_Mode teWorkingMode)
{
    if(u8Addr < 0x10 || u8Addr >0x4F)
        return E_ADAPTER_ERR_PARAMETER_RANGE;

    if(!eACMD_AdapterPresetSingleRegister(ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS+(u8Addr-0x10)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS+1, teWorkingMode))
    {
        return E_ADAPTER_SUCCESS;
    }

    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_SetIndoorSetTemperature
* Description	: ����/���ȵ��趨�¶�
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_SetIndoorSetTemperature(uint8_t u8Addr,int16_t i16Temperature)
{
    if(u8Addr < 0x10 || u8Addr >0x4F)
        return E_ADAPTER_ERR_PARAMETER_RANGE;
    //TODO:  i16Temperature �ٴ�ȷ��
    if(!eACMD_AdapterPresetSingleRegister(ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS+(u8Addr-0x10)*ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS+2, i16Temperature))
    {
        return E_ADAPTER_SUCCESS;
    }

    return E_ADAPTER_FAIL;
}


/****************************************************************************
* Function	: eACMD_SetIndoorResetFilterSignal
* Description	: ���ڻ��Ĺ����źŸ�λ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_SetIndoorResetFilterSignal(uint8_t u8Addr)
{
    return E_ADAPTER_SUCCESS;
}

//����ϵͳ��Ϣ




//LEDָʾ




extern unsigned char USART_Tx_Rx_Count_0;
extern unsigned char rx_buffer0[];
/****************************************************************************
* Function	: eACMD_GetInComingResponse
* Description	: ��ȡ���յ�����Ӧ
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_GetInComingResponse(uint8_t *pu8Data,uint8_t *pu8DataLen)
{
    teAdapterCMD_Status status = E_ADAPTER_SUCCESS;

    uint16_t u16CRC;

    if(USART_Tx_Rx_Count_0 >=5 )
    {
        emberAfAppPrintln("Rx:%d",USART_Tx_Rx_Count_0);
        //���� uart0 response
        u16CRC = rx_buffer0[USART_Tx_Rx_Count_0-1];
        u16CRC <<= 8;
        u16CRC |= rx_buffer0[USART_Tx_Rx_Count_0-2];

        if(u16CRC == CRC16(rx_buffer0, USART_Tx_Rx_Count_0-2))
        {
            memcpy(pu8Data,rx_buffer0,USART_Tx_Rx_Count_0);
            status = E_ADAPTER_SUCCESS;
        }
        else
        {
            status = E_ADAPTER_FAIL;
            emberAfAppPrintln("CRC err");
        }
        USART_Tx_Rx_Count_0  =0;
    }
    else
    {
        //uart Rx isn't enough
        emberAfAppPrintln("Rx not enough:%d",USART_Tx_Rx_Count_0);
        status = E_ADAPTER_FAIL;
        USART_Tx_Rx_Count_0  =0;
    }


    return status;
}


teAdapterCMD_Status eACMD_ReportingErrorCode(uint8_t endpoint ,uint16_t u16errCode)
{
#if 0
    EmberAfStatus status;
    // write the new cool value
    status = emberAfWriteAttribute(endpoint,
                                   ZCL_THERMOSTAT_CLUSTER_ID,
                                   ZCL_ERROR_CODE_ATTRIBUTE_ID,
                                   CLUSTER_MASK_SERVER,
                                   (uint8_t *)&u16errCode,
                                   ZCL_INT16U_ATTRIBUTE_TYPE);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        emberAfAppPrintln("ERR: writing heating %x", status);
        return status;
    }
    addReportingSchedule(endpoint, ZCL_THERMOSTAT_CLUSTER_ID, ZCL_ERROR_CODE_ATTRIBUTE_ID, 10);
#endif
}

teAdapterCMD_Status eACMD_ReportingEndpointChanged()
{
    addReportingSchedule(1, ZCL_BASIC_CLUSTER_ID, ZCL_ORVIBO_ACTIVE_ENDPOINTS_ATTRIBUTE_ID, 10);
}
/****************************************************************************
* Function	: eACMD_ClearUartBuffer
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
teAdapterCMD_Status eACMD_ClearUartBuffer()
{
    return E_ADAPTER_SUCCESS;
}






/****************************************************************************
* Function	: eACMD_EnableTx
* Description	: 	uart����ʹ��
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
void eACMD_EnableTx()
{
    halGpioSet(RS485_CONTROL_PIN);
}



/****************************************************************************
* Function	: eACMD_EnableRx
* Description	: 	uart ����ʹ��
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
void eACMD_EnableRx()
{
    halGpioClear(RS485_CONTROL_PIN);
}

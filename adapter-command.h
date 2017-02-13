/***************************************************************************************
****************************************************************************************
* FILE		: adapter-command.h
* Description	:
*
* Copyright (c) 2016 by ORVIBO. All Rights Reserved.
*
* History:
* Version		Name       		Date			Description
   0.1		sven	2016/11/03	Initial Version

****************************************************************************************
****************************************************************************************/
#ifndef _ADAPTER_COMMAND_H_
#define _ADAPTER_COMMAND_H_
#include "AirConditionController_endpoint_config.h"
#include "app/framework/include/af.h"
#include "stack/include/ember.h"
#include "app/framework/util/util.h"

//adapter is a modbus slave device
//you can config the adapter by setting the DS1 and DS2
//DS1 设置串口参数
#define DS1		BAUD_9600
//DS2	设置从机地址
#define DS2		0x01

#define ADAPTER_ADDRESS	DS2

//适配器响应时间
#define ADAPTER_RESPONSE_TIME	(FRAMES_INTERVAL_TIME+20)
//Host 接收到响应到发送下一条命令的时间间隔,大于ADAPTER_RESPONSE_TIME
#define HOST_REQUEST_INTERVAL_TIME	(ADAPTER_RESPONSE_TIME+1)

#define SYNCHRONIZE_INDOOR_STATUS_CYCLE				6000		//同步内机状态的周期
#define SYNCHRONIZE_ADAPTER_STATUS_SHORT_CYCLE		9000		//同步适配器状态的短周期
#define SYNCHRONIZE_ADAPTER_STATUS_LONG_CYCLE		30000		//同步适配器状态的长周期
#define SYNCHRONIZE_INDOOR_CONNECTION_STATUS_CYCLE	10000		//同步室内机连接状态的周期


//功能代码
#define READ_INPUT_REGISTER	0x04			//读输入寄存器，一次最多读4个寄存器
#define READ_INPUT_REGISTER_ERR	(READ_INPUT_REGISTER|0x80)
#define PRESET_SINGLE_REGISTER	0x06		//写单寄存器
#define PRESET_SINGLE_REGISTER_ERR	(PRESET_SINGLE_REGISTER|0x80)
#define PRESET_MULTIPLE_REGISTER	0x10	//写多寄存器，一次最多写4个寄存器
#define PRESET_MULTIPLE_REGISTER_ERR	(PRESET_MULTIPLE_REGISTER|0x80)

//最多每次能够读/写寄存器的数量
#define READ_WRITE_REGISTER_NUM_MAX	24					//TODO: 这个值需要确定

//异常代码
#define ILLEGAL_FUNCTION	0x01			//功能异常:此功能不被支持
#define ILLEGAL_DATA		0x03			//数据异常:此查询包含错误数据


//帧间隔时间
#define FRAMES_INTERVAL_TIME_9600	5
#define FRAMES_INTERVAL_TIME_19200	2.5
#define FRAMES_INTERVAL_TIME 	FRAMES_INTERVAL_TIME_19200

//一帧数据最长时间
#define FRAMES_TIME	7.5

#define T1 (FRAMES_INTERVAL_TIME+20+FRAMES_TIME)
#define T2 (FRAMES_INTERVAL_TIME+20+FRAMES_TIME)


//室内机起始地址------------即室内机的相对地址是0x10-0x4F
#define INDOORUNIT_BASE_ADDRESS	0x10
//室内机数量--最多64个
#define INDOORUNIT_NUMBERS	0x40


//适配器寄存器起始地址
//#define ADAPTER_REGISTER_BASE_ADDRESS	0x7531			//30001

//**********************输入寄存器*********************************//
//适配器输入寄存器起始地址
#define ADAPTER_INPUT_REGISTER_BASE_ADDRESS	0x0000			//0x7531||30001 ::::30001 mapping to 0x00

//适配器状态寄存器
#define ADAPTER_STATUS_REGISTER_ADDRESS	0x00			//30001
#define ADAPTER_STATUS_REGISTER_NUMBERS	0x1

//室内机连接状态寄存器
#define ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_BASE_ADDRESS	0x01	//30002
#define ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS	0x04

//室内机通信状态寄存器
#define ADAPTER_INDOORUNIT_COMMUNICATION_STATUS_REGISTER_BASE_ADDRESS	0x05	//30006
#define ADAPTER_INDOORUNIT_COMMUNICATION_STATUS_REGISTER_NUMBERS	0x04

//室内机性能信息寄存器
#define ADAPTER_INDOORUNIT_PERFORMANCE_INFORMATION_REGISTER_BASE_ADDRESS		0x3E8	//30001+1000(3E8)
#define ADAPTER_INDOORUNIT_PERFORMANCE_INFORMATION_REGISTER_NUMBERS			0x3  //每个内机3个，总共192个

//室内机状态信息寄存器
#define ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_BASE_ADDRESS		0x7D0			//30001+2000(7D0)
#define ADAPTER_INDOORUNIT_STATUS_INFORMATION_REGISTER_NUMBERS			0x6	//每个内机6个，总共384个


//**********************保持寄存器*********************************//
//适配器保存寄存器起始地址
#define ADAPTER_HOLDING_REGISTER_BASE_ADDRESS		0x0000			//0x9C41  40001 mapping to 0x00 

//适配器初始化设置寄存器
#define ADAPTER_INIT_SETTING_REGISTER_BASE_ADDRESS	0x0000			//40001

//室内机控制寄存器
#define ADAPTER_INDOORUNIT_CONTROL_REGISTER_BASE_ADDRESS		0x7D0		//40001+2000(7D0)   42001 mapping to 2000(0x7D0)
#define ADAPTER_INDOORUNIT_CONTROL_REGISTER_NUMBERS				0x3		//每个内机3个，总共192个


//室内机最多支持 64 个，endpoint 从 2~65( 1 是保留的，便于网关读 modeID 快速入网) 映射到 0~63
#define ADAPTER_INDOOR_ENDPOINT_SHIFT	2

#define UART_PORT	1		//serial port ID

typedef enum
{
    E_SL_IDLE	=	00,
    E_SL_BUSY,
} teSL_Status;



#define MAX_INDOOR_COUNT		(FIXED_ENDPOINT_COUNT-1)

// Adapter Command return status codes returned to the user
typedef enum
{
    // General
    E_ADAPTER_SUCCESS = 0x0,
    E_ADAPTER_FAIL,
    E_ADAPTER_ERR_PARAMETER_NULL,
    E_ADAPTER_ERR_PARAMETER_RANGE,
    E_ADAPTER_ERR_HEAP_FAIL,
    E_ADAPTER_ERR_CODE,
    E_ADAPTER_ERR_ENUM_END
} teAdapterCMD_Status;


typedef struct InDoor
{
    uint16_t u16PerformanceInfo[3];				//室内机性能信息
    union
    {
        uint16_t u16StatusInfo[6];				//室内机状态信息
        struct
        {
            uint16_t u16workStatus;				//内机工作状态
            uint16_t u16workModel;				//内机工作模式
            int16_t u16temperatureSetting;		//设定的温度
            uint16_t ExceptionCode;				//异常码
            int16_t u16indoorTemperature;		//内机温度
            int16_t u16temperatureSensor;		//温度传感器状态
        } IndoorStatusInfo;
    } IndoorStatusInfoField;
} teInDoorDevice;


//适配器状态
typedef enum
{
    // General
    E_ADAPTER_NOREADY = 0x0,			//00 没有准备好
    E_ADAPTER_READY,                  	//01 已经准备好
} teAdapter_Status;

//适配器
typedef struct Adapter
{
    teAdapter_Status bStatus;			//适配器状态;   1:已准备好   0:未准备好
    uint16_t u16IndoorConnectionStatus[ADAPTER_INDOORUNIT_CONNECTION_STATUS_REGISTER_NUMBERS];	//室内机的连接状态; 64位表示64个内机
    uint16_t u16IndoorCommunicationStatus[ADAPTER_INDOORUNIT_COMMUNICATION_STATUS_REGISTER_NUMBERS];	//室内机的通信状态；64位表示64个内机
    teInDoorDevice Indoor[MAX_INDOOR_COUNT];				//最多有64个内机
} teAdapter;




/****************************************************
*室内机控制相关定义
************************************************/
//内机的工作状态
typedef enum
{
    E_INDOOR_STOP = 0x0,				//停止
    E_INDOOR_RUNNNING,                  //运行
} teIndoorWorking_Status;

//室内机风向
typedef enum
{
    E_INDOOR_P0 = 0x0,		//水平方向
    E_INDOOR_P1,
    E_INDOOR_P2,
    E_INDOOR_P3,
    E_INDOOR_P4,			//垂直方向
    E_INDOOR_RESERVE,
    E_INDOOR_MOTIONLESS,	//停止
    E_INDOOR_WIGGLE,		//摆动
} teIndooAir_Direction;

//室内机风量:
typedef enum
{
    E_INDOOR_LL = 0x1,
    E_INDOOR_L,
    E_INDOOR_M,
    E_INDOOR_H,
    E_INDOOR_HH,
} teIndooAir_Quantity;

//室内机过滤信号复位:
typedef enum
{
    E_INDOOR_NONE = 0x0,
    E_INDOOR_RESET = 0xF,
} teIndooFilterSignalReset;

//内机的运转模式
typedef enum
{
    E_INDOOR_AIR_SUPPLY = 0x0,	//通风
    E_INDOOR_HEATING, 			//制热
    E_INDOOR_REFRIGERATION, 	//制冷
    E_INDOOR_AUTO,				//自动
    E_INDOOR_XERANSIS = 0x7 ,	//除湿
} teIndoorWorking_Mode;

//内机的开关模式
typedef enum
{
    E_INDOOR_TURN_OFF = 0x0,	//off
    E_INDOOR_TURN_ON, 			//on
} teIndoorOnOff_Mode;


/** Serial link message types */
typedef enum
{
    E_SL_MSG_NONE										=	0x0000,
    E_SL_MSG_GET_ADAPTER_STATUS_REQUEST             	=   0x0001,
    E_SL_MSG_GET_ADAPTER_STATUS_RESPONSE				=   0x8001,

    /*********Monitoring**********/

    E_SL_MSG_GET_INDOOR_ONOFF_MODE_REQUEST				= 	0x0010,
    E_SL_MSG_GET_INDOOR_ONOFF_MODE_RESPONSE				= 	0x8010,

    E_SL_MSG_GET_INDOOR_WORKING_MODE_REQUEST			= 	0x0011,
    E_SL_MSG_GET_INDOOR_WORKING_MODE_RESPONSE			= 	0x8011,

    E_SL_MSG_GET_INDOOR_SETING_TEMPE_REQUEST			= 	0x0012,
    E_SL_MSG_GET_INDOOR_SETING_TEMPE_RESPONSE			= 	0x8012,

    E_SL_MSG_GET_INDOOR_INHALE_TEMPE_REQUEST			= 	0x0013,
    E_SL_MSG_GET_INDOOR_INHALE_TEMPE_RESPONSE			= 	0x8013,

    E_SL_MSG_GET_INDOOR_BLOWINGRATE_REQUEST				=	0x0015,
    E_SL_MSG_GET_INDOOR_BLOWINGRATE_RESPONSE			=	0x8015,

    E_SL_MSG_GET_INDOOR_FORCE_STOP_STATUS_REQUEST		= 	0x0016,
    E_SL_MSG_GET_INDOOR_FORCE_STOP_STATUS_RESPONSE		= 	0x8016,

    E_SL_MSG_GET_INDOOR_ERROR_CODE_REQUEST				= 	0x0017,
    E_SL_MSG_GET_INDOOR_ERROR_CODE_RESPONSE				=	0x8017,

    E_SL_MSG_GET_INDOOR_FILTER_SIGNAL_REQUEST			=	0x0018,
    E_SL_MSG_GET_INDOOR_FILTER_SIGNAL_RESPONSE			=	0x8018,

    E_SL_MSG_GET_INDOOR_COMMUNICATION_STATUS_REQUEST	= 	0x0019,
    E_SL_MSG_GET_INDOOR_COMMUNICATION_STATUS_RESPONSE	= 	0x8019,

	E_SL_MSG_GET_INDOOR_STATUS_REQUEST					= 	0x001A,
    E_SL_MSG_GET_INDOOR_STATUS_RESPONSE					= 	0x801A,
    
    /*********Control**********/
    E_SL_MSG_SET_INDOOR_ONOFF_MODE_REQUEST				= 	0x0020,
    E_SL_MSG_SET_INDOOR_ONOFF_MODE_RESPONSE				= 	0x8020,

    E_SL_MSG_SET_INDOOR_WORKING_MODE_REQUEST			= 	0x0021,
    E_SL_MSG_SET_INDOOR_WORKING_MODE_RESPONSE			= 	0x8021,

    E_SL_MSG_SET_INDOOR_SETING_TEMPE_REQUEST			= 	0x0022,
    E_SL_MSG_SET_INDOOR_SETING_TEMPE_RESPONSE			= 	0x8022,

    //E_SL_MSG_SET_INDOOR_VANE_DIRECTION_REQUEST			= 	0x0023,
    //E_SL_MSG_SET_INDOOR_VANE_DIRECTION_RESPONSE			= 	0x8023,

    E_SL_MSG_SET_INDOOR_BLOWINGRATE_REQUEST				=	0x0024,
    E_SL_MSG_SET_INDOOR_BLOWINGRATE_RESPONSE			=	0x8024,

    E_SL_MSG_RESET_INDOOR_FILTER_SIGNAL_REQUEST			=	0x0025,
    E_SL_MSG_RESET_INDOOR_FILTER_SIGNAL_RESPONSE		=	0x8025,

    /*********Searching System Info**********/
    E_SL_MSG_GET_INDOOR_CONNECTION_STATUS_REQUEST		=	0x0036,
    E_SL_MSG_GET_INDOOR_CONNECTION_STATUS_RESPONSE		=	0x8036,

    E_SL_MSG_GET_INDOOR_PERFORMANCE_INFO_REQUEST		=	0x0037,
    E_SL_MSG_GET_INDOOR_PERFORMANCE_INFO_RESPONSE		=	0x8037,

    /*********LED Indication**********/

} teSL_MsgType;


// Definition of Read  adapter status Request Message Structure
typedef struct
{
    bool                      bStatus;
} tsSL_GetAdapterStatusRequest;

// Definition of Read  indoor status Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorStatusRequest;


// Definition of Read  indoor on/off mode  Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorOnOffModeRequest;

// Definition of Read  indoor working mode  Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorWorkingModeRequest;

// Definition of Read  indoor seting temperature  Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorSetingTempeRequest;

// Definition of Read  indoor inhale temperature  Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorInhaleTempeRequest;

// Definition of Read  indoor vane direction Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorVaneDirectionRequest;


// Definition of Read  indoor blowingRate Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorBlowingRateRequest;

// Definition of Read  indoor ForceStopStatus Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorForceStopStatusRequest;


// Definition of Read  indoor ErrorCode Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorErrorCodeRequest;


// Definition of Read  indoor FilterSignal Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorFilterSignalRequest;


// Definition of Read  indoor CommunicationStatus Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorBlockAddress;		// 0x00 - -0x03
} tsSL_GetIndoorCommunicationStatusRequest;


// Definition of Write  indoor OnOff Mode Request Message Structure
typedef struct
{
    uint8_t                      	u8IndoorAddress;		// 0x10 -- 0x4F
    teIndoorOnOff_Mode			ePreOnOff;					//Pre On/Off status
    teIndoorOnOff_Mode			eOnOff;
} tsSL_SetIndoorOnOffModeRequest;

// Definition of Write  indoor WorkingMode Request Message Structure
typedef struct
{
    uint8_t                      	u8IndoorAddress;		// 0x10 -- 0x4F
    teIndoorWorking_Mode			ePreWorkingMode;    
    teIndoorWorking_Mode			eWorkingMode;
} tsSL_SetIndoorWorkingModeRequest;

// Definition of Write  indoor SetingTemperature Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
    uint8_t						u8Mode;
    uint16_t						u16PreSetingTemperature;    
    uint16_t						u16SetingTemperature;
} tsSL_SetIndoorSetingTempeRequest;


// Definition of Write  indoor VaneDirection Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
    teIndooAir_Direction		ePreAirDirection;
    teIndooAir_Direction		eAirDirection;
} tsSL_SetIndoorVaneDirectionRequest;

// Definition of Write  indoor BlowingRate Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
    teIndooAir_Quantity			ePreAirQuantity;
    teIndooAir_Quantity			eAirQuantity;
} tsSL_SetIndoorBlowingRateRequest;

// Definition of Write  indoor FilterSignal Request Message Structure
typedef struct
{
    uint8_t                      	u8IndoorAddress;		// 0x10 -- 0x4F
    teIndooFilterSignalReset	eFilterSignalReset;
} tsSL_SetIndoorFilterSignalRequest;


/*********Searching System Info**********/
// Definition of Read  indoor ConnectionStatus Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorConnectionStatusRequest;

// Definition of Read  indoor PerformanceInfo Request Message Structure
typedef struct
{
    uint8_t                      u8IndoorAddress;		// 0x10 -- 0x4F
} tsSL_GetIndoorPerformanceInfoRequest;



/** Msg Status */
typedef enum
{
    E_MSG_WAIT_SEND = 0x01,		//等待被发送
    E_MSG_WAIT_RSP ,			//等待应答
} teMsg_Status;


//Definition of SL Tx Msg Structure
typedef struct SL_Msg
{
    teSL_MsgType 								eMsgType;			//消息类型
    teMsg_Status                             	eMsgStatus;			//消息状态
    uint8_t                                    	u8EndPoint;			//消息的目标endpoint，没有就填0
    uint8_t										u8RetryCount;		//重发次数
    union
    {
        tsSL_GetAdapterStatusRequest			sGetAdapterStatusRequest;
		tsSL_GetIndoorStatusRequest				sGetIndoorStatusRequest;
		tsSL_GetIndoorCommunicationStatusRequest	sGetIndoorCommunicationStatusRequest;
        /*********Searching System Info**********/
        tsSL_GetIndoorConnectionStatusRequest	sGetIndoorConnectionStatusRequest;
        tsSL_GetIndoorPerformanceInfoRequest	sGetIndoorPerformanceInfoRequest;

        tsSL_GetIndoorOnOffModeRequest			sGetIndoorOnOffModeRequest;
        tsSL_GetIndoorWorkingModeRequest		sGetIndoorWorkingModeRequest;
        tsSL_GetIndoorSetingTempeRequest		sGetIndoorSetingTempeRequest;
        tsSL_GetIndoorInhaleTempeRequest		sGetIndoorInhaleTempeRequest;
        tsSL_GetIndoorVaneDirectionRequest		sGetIndoorVaneDirectionRequest;
        tsSL_GetIndoorBlowingRateRequest		sGetIndoorBlowingRateRequest;
        tsSL_GetIndoorForceStopStatusRequest	sGetIndoorForceStopStatusRequest;
        tsSL_GetIndoorErrorCodeRequest			sGetIndoorErrorCodeRequest;
        tsSL_GetIndoorFilterSignalRequest		sGetIndoorFilterSignalRequest;

		
        tsSL_SetIndoorOnOffModeRequest			sSetIndoorOnOffModeRequest;
        tsSL_SetIndoorWorkingModeRequest		sSetIndoorWorkingModeRequest;
        tsSL_SetIndoorSetingTempeRequest		sSetIndoorSetingTempeRequest;
        tsSL_SetIndoorVaneDirectionRequest		sSetIndoorVaneDirectionRequest;
        tsSL_SetIndoorBlowingRateRequest		sSetIndoorBlowingRateRequest;
        tsSL_SetIndoorFilterSignalRequest		sSetIndoorFilterSignalRequest;
    } uMessage ;
} tsSL_Msg;

#define SL_MSG_TX_SIZE		70

typedef struct
{
	uint8_t		isFull;
    uint8_t  	size;
    uint8_t  	head;
    uint8_t  	tail;
    tsSL_Msg*  	psSL_MsgBuffer;
} tsMsgRingBuffer;

teAdapterCMD_Status eACMD_InitSystem();
teAdapterCMD_Status eACMD_UpdateEndpointVisible();
/*****************************************Basic Operate**********************************************************/
teAdapterCMD_Status eACMD_AdapterCommandSend(uint8_t u8SlaveAddr,uint8_t u8FunctionCode,uint16_t u16RegisterAddr,uint8_t *pu8Data,uint8_t u8DataLen);
teAdapterCMD_Status eACMD_AdapterReadInputRegister(uint16_t u16RegisterAddr,uint16_t u16RegisterNum);
teAdapterCMD_Status eACMD_AdapterPresetSingleRegister(uint16_t u16RegisterAddr,uint16_t u16RegisterData);
teAdapterCMD_Status eACMD_AdapterPresetMultipleRegister(uint16_t u16RegisterAddr,uint16_t u16RegisterNum,uint8_t u8DataSize,uint8_t *pu8Data);

/****************************************************监控***********************************************/

teAdapterCMD_Status eACMD_GetAdapterStatus();							//获取适配器状态
teAdapterCMD_Status eACMD_GetIndoorStatus(uint8_t u8indoorAddr);		//获取单个室内机状态
teAdapterCMD_Status eACMD_GetIndoorUnitConnectionStatus();		//获取室内机连接状态
teAdapterCMD_Status eACMD_GetIndoorUnitCommunicateStatus();
teAdapterCMD_Status eACMD_GetIndoorUnitPerformanceInfomation(uint8_t u8indoorAddr);
teAdapterCMD_Status eACMD_ModifyAdapterSlaveAddress(uint8_t u8slaveAddress);


//获取内机信息可以一次获取完成,不需要单独获取
teAdapterCMD_Status eACMD_GetIndoorUnitStatusInfomation(uint8_t u8indoorAddr);
teAdapterCMD_Status eACMD_GetIndoorOnOffMode(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorAirDirection(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorAirQuantity(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorWorkingMode(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorSetTemperature(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorResetFilterSignal(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorInhaleTempe(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorForceStopStatus(uint8_t u8Addr);
teAdapterCMD_Status eACMD_GetIndoorFilterSignal(uint8_t u8Addr);


/****************************************************控制***********************************************/
teAdapterCMD_Status eACMD_SetIndoorOnOffMode(uint8_t u8Addr,teIndoorOnOff_Mode teOnOffMode);
teAdapterCMD_Status eACMD_SetIndoorAirDirection(uint8_t u8Addr,teIndooAir_Direction teAirDirection);
teAdapterCMD_Status eACMD_SetIndoorAirQuantity(uint8_t u8Addr,teIndooAir_Quantity teAirQuantity);
teAdapterCMD_Status eACMD_SetIndoorWorkingMode(uint8_t u8Addr,teIndoorWorking_Mode teWorkingMode);
teAdapterCMD_Status eACMD_SetIndoorSetTemperature(uint8_t u8Addr,int16_t i16Temperature);
teAdapterCMD_Status eACMD_SetIndoorResetFilterSignal(uint8_t u8Addr);

//上报错误码
teAdapterCMD_Status eACMD_ReportingErrorCode(uint8_t endpoint ,uint16_t u16errCode);

//上报endpoint改变属性
teAdapterCMD_Status eACMD_ReportingEndpointChanged();


teAdapterCMD_Status eACMD_GetInComingResponse(uint8_t *pu8Data,uint8_t *pu8DataLen);



#define RS485_CONTROL_PIN	0x03
void eACMD_EnableTx();
void eACMD_EnableRx();



#define SL_MAX_RETRY	1

extern teSL_MsgType 	teSL_CurrentREQ;		//串口当前请求的命令
extern teSL_MsgType 	teSL_CurrentRSP;		//串口当前等待的响应
extern teSL_Status		eSL_Status;	//串口是当前状态

extern tsMsgRingBuffer sMsgTxRingBuffer;			//串口TX循环队列
extern teAdapter Adapter;			//适配器设备
#endif /*_ADAPTER_COMMAND_H_*/




////////////////////////////////////////////////////////////////////////////
// Copyright 2016 Shenzhen ORVIBO Electronics Co., Ltd.
//
// Description:添加属性报告任务
//
// Auther :hxf
//
// data :20161101
////////////////////////////////////////////////////////////////////////////

#include "app/framework/include/af.h"
#include "attributeReporting.h"

#define MAX_REPORTABLE_ATTR 8   //属性等待上报最大缓存数(单属性上报)
#define MAX_PER_ATTR_BYTE   50  //单个属性属性值最大字节数
#define MAX_MULTI_ATTR_NUM  10  //多属性上报，一包中最大属性条数

//默认上报地址和端点 ，仅在未找到绑定表时有效
#define DEFAULT_REPORTING_ADDRESS     0x0000
#define DEFAULT_REPORTING_ENDPOINT    0x01

#define ATTRIBUTE_REPORTING_DEBUG  //是否开启本文件debug打印

//需要到“工程_endpoint_config.h”文件添加以下两个事件和相应处理函数
EmberEventControl attributeReportingEventControl;     //单属性处理事件
EmberEventControl MultiAttributeReportingEventControl;//多属性处理事件

//单属性上报缓存结构体
typedef struct
{
    uint8_t endpoint;
    EmberAfClusterId clusterId;
    EmberAfAttributeId attributeId;
    uint32_t delayTimeMs;
} attrReport_t;

//多属性上报缓存结构体
typedef struct
{
    uint8_t endpoint;
    EmberAfClusterId clusterId;
    uint8_t attributeNum;
    EmberAfAttributeId attributeId[10];
    uint32_t delayTimeMs;
} multiAttrReport_t;

multiAttrReport_t multiAttrReport;
attrReport_t attrReport[MAX_REPORTABLE_ATTR];
uint32_t recentlyReportTime = 0xffffffff; //最近一次上报需要延时时间

extern EmberApsFrame globalApsFrame;
extern uint8_t appZclBuffer[EMBER_AF_MAXIMUM_SEND_PAYLOAD_LENGTH];
extern uint16_t appZclBufferLen;
extern uint8_t disableDefaultResponse;

extern void zclBufferSetup(uint8_t frameType, uint16_t clusterId, uint8_t commandId);
extern void zclBufferAddWord(uint16_t word);
extern void zclBufferAddByte(uint8_t type);
extern void emAfApsFrameEndpointSetup(uint8_t srcEndpoint,uint8_t dstEndpoint);

void attributeReportingEventHandler(void);
void MultiAttributeReportingEventHandler(void);
//延时启动属性上报
bool addReportingSchedule(uint8_t endpoint,
                          EmberAfClusterId clusterId,
                          EmberAfAttributeId attributeId,
                          uint32_t delayTimeMs);


uint8_t returnMinTimeIndex();
bool decreaceReportTime(uint32_t timeMs);
void updataReportTime();

//单属性上报处理
void attributeReportingEventHandler(void)
{
    uint8_t i = 0;
    uint8_t minTimeIndex = 0xff;
    EmberStatus status;
    EmberAfAttributeType type;
    uint8_t size;
    uint8_t data[MAX_PER_ATTR_BYTE];
    disableDefaultResponse = 1;

    emberEventControlSetInactive(attributeReportingEventControl);
    updataReportTime();  //更新时间

    for(i = 0; i < MAX_REPORTABLE_ATTR; i++)
    {
        if(attrReport[i].endpoint != 0 && attrReport[i].delayTimeMs == 0)
        {
#ifdef ATTRIBUTE_REPORTING_DEBUG
            emberAfAppPrint("\r\n Index = %d",i);
            emberAfAppPrint("\r\n report ep %x cluster %4x attr %x",attrReport[i].endpoint, attrReport[i].clusterId, attrReport[i].attributeId);
#endif
            status = emberAfReadAttribute(attrReport[i].endpoint, // endpoint
                                          attrReport[i].clusterId,
                                          attrReport[i].attributeId,
                                          CLUSTER_MASK_SERVER,
                                          data,
                                          sizeof(data),
                                          &type);

            if (status != EMBER_ZCL_STATUS_SUCCESS)
            {
#ifdef ATTRIBUTE_REPORTING_DEBUG
                emberAfAppPrint("ERR: reading attribute %x", status);
#endif
                attrReport[i].endpoint = 0;   //属性读取出错端点恢复无效值
                return;
            }

            zclBufferSetup(ZCL_GLOBAL_COMMAND
                           | ZCL_FRAME_CONTROL_SERVER_TO_CLIENT,
                           attrReport[i].clusterId,
                           ZCL_REPORT_ATTRIBUTES_COMMAND_ID);

            zclBufferAddWord(attrReport[i].attributeId);
            zclBufferAddByte(type);

            size = (emberAfIsThisDataTypeAStringType(type)
                    ? emberAfStringLength(data) + 1
                    : emberAfGetDataSize(type));

#if (BIGENDIAN_CPU)
            if (isThisDataTypeSentLittleEndianOTA(type))
            {
                emberReverseMemCopy(appZclBuffer + appZclBufferLen, data, size);
            }
            else
            {
                MEMMOVE(appZclBuffer + appZclBufferLen, data, size);
            }
#else
            MEMMOVE(appZclBuffer + appZclBufferLen, data, size);
#endif
            appZclBufferLen += size;

            //优先使用绑定表地址发送
            status = emberAfSendUnicastToBindings(&globalApsFrame,
                                                  appZclBufferLen,
                                                  appZclBuffer);

            if(status != EMBER_SUCCESS)
            {
                //找不到绑定表地址使用ORB默认地址0
                emAfApsFrameEndpointSetup(attrReport[i].endpoint, DEFAULT_REPORTING_ENDPOINT);
                status = emberAfSendUnicast(EMBER_OUTGOING_DIRECT,
                                            DEFAULT_REPORTING_ADDRESS,
                                            &globalApsFrame,
                                            appZclBufferLen,
                                            appZclBuffer);
#ifdef ATTRIBUTE_REPORTING_DEBUG
                emberAfAppPrint("\r\n ORB addr report %x",status);
#endif
            }
            attrReport[i].endpoint = 0;   //上报完端点恢复无效值
        }
    }
    minTimeIndex = returnMinTimeIndex();

    if(minTimeIndex != 0xff)  //获得有效索引，继续运行定时事件
    {
#ifdef ATTRIBUTE_REPORTING_DEBUG
        emberAfAppPrint("\r\n attributeReportingEventControl time：%d ",attrReport[minTimeIndex].delayTimeMs);
#endif
        recentlyReportTime = attrReport[minTimeIndex].delayTimeMs;
        emEventControlSetDelayMS(&attributeReportingEventControl, attrReport[minTimeIndex].delayTimeMs);
    }
    else //表中全部执行完毕
    {
        recentlyReportTime = 0xffffffff;
    }
}

//多属性上报处理
void MultiAttributeReportingEventHandler(void)
{
    uint8_t i = 0;
    EmberStatus status;
    EmberAfAttributeType type;
    uint8_t size;
    uint8_t data[MAX_PER_ATTR_BYTE];
    disableDefaultResponse = 1;

    emberEventControlSetInactive(MultiAttributeReportingEventControl);
    zclBufferSetup(ZCL_GLOBAL_COMMAND
                   | ZCL_FRAME_CONTROL_SERVER_TO_CLIENT,
                   multiAttrReport.clusterId,
                   ZCL_REPORT_ATTRIBUTES_COMMAND_ID);

    for(i = 0; i < multiAttrReport.attributeNum; i++)
    {
#ifdef ATTRIBUTE_REPORTING_DEBUG
        emberAfAppPrint("\r\n multiAttrReport ep %x cluster %x attr %x",multiAttrReport.endpoint, multiAttrReport.clusterId, multiAttrReport.attributeId[i]);
#endif
        status = emberAfReadAttribute(multiAttrReport.endpoint, // endpoint
                                      multiAttrReport.clusterId,
                                      multiAttrReport.attributeId[i],
                                      CLUSTER_MASK_SERVER,
                                      data,
                                      sizeof(data),
                                      &type);

        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
#ifdef ATTRIBUTE_REPORTING_DEBUG
            emberAfAppPrint("ERR: reading attribute %x", status);
#endif
            multiAttrReport.endpoint = 0;   //属性读取出错端点恢复无效值
            return;
        }

        zclBufferAddWord(multiAttrReport.attributeId[i]);
        zclBufferAddByte(type);

        size = (emberAfIsThisDataTypeAStringType(type)
                ? emberAfStringLength(data) + 1
                : emberAfGetDataSize(type));

#if (BIGENDIAN_CPU)
        if (isThisDataTypeSentLittleEndianOTA(type))
        {
            emberReverseMemCopy(appZclBuffer + appZclBufferLen, data, size);
        }
        else
        {
            MEMMOVE(appZclBuffer + appZclBufferLen, data, size);
        }
#else
        MEMMOVE(appZclBuffer + appZclBufferLen, data, size);
#endif
        appZclBufferLen += size;

    }
    //优先使用绑定表地址发送
    status = emberAfSendUnicastToBindings(&globalApsFrame,
                                          appZclBufferLen,
                                          appZclBuffer);

    if(status != EMBER_SUCCESS)
    {
        //找不到绑定表地址使用ORB默认地址0
        emAfApsFrameEndpointSetup(attrReport[i].endpoint, DEFAULT_REPORTING_ENDPOINT);
        status = emberAfSendUnicast(EMBER_OUTGOING_DIRECT,
                                    DEFAULT_REPORTING_ADDRESS,
                                    &globalApsFrame,
                                    appZclBufferLen,
                                    appZclBuffer);
#ifdef ATTRIBUTE_REPORTING_DEBUG
        emberAfAppPrint("\r\n ORB addr report %x",status);
#endif
    }
}

///////////////////////////////////////////////////////////////////////////////
// 功能：添加单属性上报任务（非立即上报，需要系统循环后执行），如果添加的是相同属性
//       则更新之前属性上报时间
//
// 参数：endpoint - 源端点
//       clusterId - cluster ID
//       attributeId - 属性ID
//       delayTimeMs - 上报延时时间
//
// 返回值：TRUE-任务添加成功，FALSE-任务表已满
///////////////////////////////////////////////////////////////////////////////
bool addReportingSchedule(uint8_t endpoint,
                          EmberAfClusterId clusterId,
                          EmberAfAttributeId attributeId,
                          uint32_t delayTimeMs)
{
    uint8_t i = 0;
    uint8_t emptyIndex = 0xff;
    uint8_t MinTimeIndex = 0xff;
    uint32_t reportRemainingMS = 0;

    if(recentlyReportTime == 0xffffffff)  //上报表为空，直接添加
    {
#ifdef ATTRIBUTE_REPORTING_DEBUG
        emberAfAppPrint("\r\n add new report Time == %d ",delayTimeMs);
#endif
        attrReport[0].endpoint = endpoint;
        attrReport[0].clusterId = clusterId;
        attrReport[0].attributeId = attributeId;
        attrReport[0].delayTimeMs = delayTimeMs;
        recentlyReportTime = delayTimeMs;
        emEventControlSetDelayMS(&attributeReportingEventControl, attrReport[0].delayTimeMs);
        return TRUE;
    }
    else //上报表非空
    {
        for(i = 0; i < MAX_REPORTABLE_ATTR; i++)
        {
            //找到相同的一条属性未执行上报，更新上报时间
            if(attrReport[i].endpoint == endpoint &&
                    attrReport[i].clusterId == clusterId &&
                    attrReport[i].attributeId == attributeId)
            {
                reportRemainingMS = emEventControlGetRemainingMS(&attributeReportingEventControl);
                decreaceReportTime(recentlyReportTime - reportRemainingMS);
                attrReport[i].delayTimeMs = delayTimeMs;
                MinTimeIndex = returnMinTimeIndex();  //重新找到最小上报延时索引
                recentlyReportTime = attrReport[MinTimeIndex].delayTimeMs;
                emberEventControlSetInactive(attributeReportingEventControl);
                emEventControlSetDelayMS(&attributeReportingEventControl, attrReport[MinTimeIndex].delayTimeMs);
#ifdef ATTRIBUTE_REPORTING_DEBUG
                emberAfAppPrint("\r\n updata report Time == %d ",delayTimeMs);
#endif
                return TRUE;
            }
            //记录第一条空闲索引
            else if(attrReport[i].endpoint == 0 &&
                    emptyIndex == 0xff)
            {
                emptyIndex = i;
            }
        }
        if(emptyIndex != 0xff)
        {
#ifdef ATTRIBUTE_REPORTING_DEBUG
            emberAfAppPrint("\r\n add new report Time == %d ",delayTimeMs);
#endif
            //在第一个空闲位置添加一条新的属性上报
            attrReport[emptyIndex].endpoint = endpoint;
            attrReport[emptyIndex].clusterId = clusterId;
            attrReport[emptyIndex].attributeId = attributeId;
            reportRemainingMS = emEventControlGetRemainingMS(&attributeReportingEventControl);
            decreaceReportTime(recentlyReportTime - reportRemainingMS);
            attrReport[emptyIndex].delayTimeMs = delayTimeMs;
            MinTimeIndex = returnMinTimeIndex();  //重新找到最小上报延时索引
            recentlyReportTime = attrReport[MinTimeIndex].delayTimeMs;
#ifdef ATTRIBUTE_REPORTING_DEBUG
            emberAfAppPrint("\r\n recentlyReportTime == %d ",recentlyReportTime);
#endif
            emberEventControlSetInactive(attributeReportingEventControl);
            emEventControlSetDelayMS(&attributeReportingEventControl, attrReport[MinTimeIndex].delayTimeMs);
            return TRUE;
        }
    }
    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// 功能：添加多属性上报任务（支持立即上报，直接在添加处执行），只支持一条缓存，如果
//      添加多条任务则覆盖之前上报任务
//
// 参数：endpoint - 源端点
//       clusterId - cluster ID
//       attributeNum - 属性数量
//       attributeId[] - 属性ID列表
//       delayTimeMs - 上报延时时间
//       reportNow - 是否立即上报,TRUE:直接调用发送，不经过系统循环，可连续调用此函
//                  数上报属性;FALSE:不立即发送，在系统循环时发送
//
// 返回值：TRUE-任务添加成功，FALSE-任务表已满
///////////////////////////////////////////////////////////////////////////////
bool addMultiReportingSchedule(uint8_t endpoint,
                               EmberAfClusterId clusterId,
                               uint8_t attributeNum,
                               EmberAfAttributeId attributeId[],
                               uint32_t delayTimeMs,
                               bool reportNow)
{
    emberEventControlSetInactive(MultiAttributeReportingEventControl);
    multiAttrReport.endpoint = endpoint;
    multiAttrReport.clusterId = clusterId;
    if(attributeNum > MAX_MULTI_ATTR_NUM)
    {
        multiAttrReport.attributeNum = MAX_MULTI_ATTR_NUM;
    }
    multiAttrReport.attributeNum = attributeNum;
    MEMMOVE(multiAttrReport.attributeId, attributeId, multiAttrReport.attributeNum *sizeof(EmberAfAttributeId));
    multiAttrReport.delayTimeMs = delayTimeMs;

    if(reportNow == TRUE)  //立即调用属性上报
    {
        emberAfAppPrint(" \r\n MultiAttributeReportingEventHandler ");
        MultiAttributeReportingEventHandler();
    }
    else //延时上报
    {
        emberAfAppPrint(" \r\n MultiAttributeReporting SetDelayMS ");
        emEventControlSetDelayMS(&MultiAttributeReportingEventControl, multiAttrReport.delayTimeMs);
    }
    return TRUE;
}


//返回最小延时时间的索引，0xff为无效值
uint8_t returnMinTimeIndex()
{
    uint8_t i = 0;
    uint8_t minIndex = 0xff;
    uint32_t minTime = 0xffffffff;

    for(i = 0; i < MAX_REPORTABLE_ATTR; i++)
    {
        if(attrReport[i].endpoint != 0)
        {
            if(attrReport[i].delayTimeMs < minTime)
            {
                minTime = attrReport[i].delayTimeMs;
                minIndex = i;
            }
        }
    }
    return minIndex;
}

//上报延时表减去固定值ms
bool decreaceReportTime(uint32_t timeMs)
{
    uint8_t i = 0;
    uint8_t index = 0;

    index = returnMinTimeIndex();
    //上报表空
    if(index != 0xff)
    {
        return FALSE;
    }
    else
    {
        if(attrReport[index].delayTimeMs < timeMs)
        {
            return FALSE;
        }
    }
    for(i = 0; i < MAX_REPORTABLE_ATTR; i++)
    {
        if(attrReport[i].endpoint != 0)
        {
            attrReport[i].delayTimeMs -= timeMs;
        }
    }
    return TRUE;
}

//更新上报时间表
void updataReportTime()
{
    uint8_t i = 0;

    for(i = 0; i < MAX_REPORTABLE_ATTR; i++)
    {
        if(attrReport[i].endpoint != 0)
        {
            attrReport[i].delayTimeMs -= recentlyReportTime;
        }
    }
}


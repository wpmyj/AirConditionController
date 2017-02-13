////////////////////////////////////////////////////////////////////////////
// Copyright 2016 Shenzhen ORVIBO Electronics Co., Ltd.
//
// Description:������Ա�������
//
// Auther :hxf
//
// data :20161101
////////////////////////////////////////////////////////////////////////////

#include "app/framework/include/af.h"
#include "attributeReporting.h"

#define MAX_REPORTABLE_ATTR 8   //���Եȴ��ϱ���󻺴���(�������ϱ�)
#define MAX_PER_ATTR_BYTE   50  //������������ֵ����ֽ���
#define MAX_MULTI_ATTR_NUM  10  //�������ϱ���һ���������������

//Ĭ���ϱ���ַ�Ͷ˵� ������δ�ҵ��󶨱�ʱ��Ч
#define DEFAULT_REPORTING_ADDRESS     0x0000
#define DEFAULT_REPORTING_ENDPOINT    0x01

#define ATTRIBUTE_REPORTING_DEBUG  //�Ƿ������ļ�debug��ӡ

//��Ҫ��������_endpoint_config.h���ļ�������������¼�����Ӧ������
EmberEventControl attributeReportingEventControl;     //�����Դ����¼�
EmberEventControl MultiAttributeReportingEventControl;//�����Դ����¼�

//�������ϱ�����ṹ��
typedef struct
{
    uint8_t endpoint;
    EmberAfClusterId clusterId;
    EmberAfAttributeId attributeId;
    uint32_t delayTimeMs;
} attrReport_t;

//�������ϱ�����ṹ��
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
uint32_t recentlyReportTime = 0xffffffff; //���һ���ϱ���Ҫ��ʱʱ��

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
//��ʱ���������ϱ�
bool addReportingSchedule(uint8_t endpoint,
                          EmberAfClusterId clusterId,
                          EmberAfAttributeId attributeId,
                          uint32_t delayTimeMs);


uint8_t returnMinTimeIndex();
bool decreaceReportTime(uint32_t timeMs);
void updataReportTime();

//�������ϱ�����
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
    updataReportTime();  //����ʱ��

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
                attrReport[i].endpoint = 0;   //���Զ�ȡ����˵�ָ���Чֵ
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

            //����ʹ�ð󶨱��ַ����
            status = emberAfSendUnicastToBindings(&globalApsFrame,
                                                  appZclBufferLen,
                                                  appZclBuffer);

            if(status != EMBER_SUCCESS)
            {
                //�Ҳ����󶨱��ַʹ��ORBĬ�ϵ�ַ0
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
            attrReport[i].endpoint = 0;   //�ϱ���˵�ָ���Чֵ
        }
    }
    minTimeIndex = returnMinTimeIndex();

    if(minTimeIndex != 0xff)  //�����Ч�������������ж�ʱ�¼�
    {
#ifdef ATTRIBUTE_REPORTING_DEBUG
        emberAfAppPrint("\r\n attributeReportingEventControl time��%d ",attrReport[minTimeIndex].delayTimeMs);
#endif
        recentlyReportTime = attrReport[minTimeIndex].delayTimeMs;
        emEventControlSetDelayMS(&attributeReportingEventControl, attrReport[minTimeIndex].delayTimeMs);
    }
    else //����ȫ��ִ�����
    {
        recentlyReportTime = 0xffffffff;
    }
}

//�������ϱ�����
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
            multiAttrReport.endpoint = 0;   //���Զ�ȡ����˵�ָ���Чֵ
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
    //����ʹ�ð󶨱��ַ����
    status = emberAfSendUnicastToBindings(&globalApsFrame,
                                          appZclBufferLen,
                                          appZclBuffer);

    if(status != EMBER_SUCCESS)
    {
        //�Ҳ����󶨱��ַʹ��ORBĬ�ϵ�ַ0
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
// ���ܣ���ӵ������ϱ����񣨷������ϱ�����Ҫϵͳѭ����ִ�У��������ӵ�����ͬ����
//       �����֮ǰ�����ϱ�ʱ��
//
// ������endpoint - Դ�˵�
//       clusterId - cluster ID
//       attributeId - ����ID
//       delayTimeMs - �ϱ���ʱʱ��
//
// ����ֵ��TRUE-������ӳɹ���FALSE-���������
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

    if(recentlyReportTime == 0xffffffff)  //�ϱ���Ϊ�գ�ֱ�����
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
    else //�ϱ���ǿ�
    {
        for(i = 0; i < MAX_REPORTABLE_ATTR; i++)
        {
            //�ҵ���ͬ��һ������δִ���ϱ��������ϱ�ʱ��
            if(attrReport[i].endpoint == endpoint &&
                    attrReport[i].clusterId == clusterId &&
                    attrReport[i].attributeId == attributeId)
            {
                reportRemainingMS = emEventControlGetRemainingMS(&attributeReportingEventControl);
                decreaceReportTime(recentlyReportTime - reportRemainingMS);
                attrReport[i].delayTimeMs = delayTimeMs;
                MinTimeIndex = returnMinTimeIndex();  //�����ҵ���С�ϱ���ʱ����
                recentlyReportTime = attrReport[MinTimeIndex].delayTimeMs;
                emberEventControlSetInactive(attributeReportingEventControl);
                emEventControlSetDelayMS(&attributeReportingEventControl, attrReport[MinTimeIndex].delayTimeMs);
#ifdef ATTRIBUTE_REPORTING_DEBUG
                emberAfAppPrint("\r\n updata report Time == %d ",delayTimeMs);
#endif
                return TRUE;
            }
            //��¼��һ����������
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
            //�ڵ�һ������λ�����һ���µ������ϱ�
            attrReport[emptyIndex].endpoint = endpoint;
            attrReport[emptyIndex].clusterId = clusterId;
            attrReport[emptyIndex].attributeId = attributeId;
            reportRemainingMS = emEventControlGetRemainingMS(&attributeReportingEventControl);
            decreaceReportTime(recentlyReportTime - reportRemainingMS);
            attrReport[emptyIndex].delayTimeMs = delayTimeMs;
            MinTimeIndex = returnMinTimeIndex();  //�����ҵ���С�ϱ���ʱ����
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
// ���ܣ���Ӷ������ϱ�����֧�������ϱ���ֱ������Ӵ�ִ�У���ֻ֧��һ�����棬���
//      ��Ӷ��������򸲸�֮ǰ�ϱ�����
//
// ������endpoint - Դ�˵�
//       clusterId - cluster ID
//       attributeNum - ��������
//       attributeId[] - ����ID�б�
//       delayTimeMs - �ϱ���ʱʱ��
//       reportNow - �Ƿ������ϱ�,TRUE:ֱ�ӵ��÷��ͣ�������ϵͳѭ�������������ô˺�
//                  ���ϱ�����;FALSE:���������ͣ���ϵͳѭ��ʱ����
//
// ����ֵ��TRUE-������ӳɹ���FALSE-���������
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

    if(reportNow == TRUE)  //�������������ϱ�
    {
        emberAfAppPrint(" \r\n MultiAttributeReportingEventHandler ");
        MultiAttributeReportingEventHandler();
    }
    else //��ʱ�ϱ�
    {
        emberAfAppPrint(" \r\n MultiAttributeReporting SetDelayMS ");
        emEventControlSetDelayMS(&MultiAttributeReportingEventControl, multiAttrReport.delayTimeMs);
    }
    return TRUE;
}


//������С��ʱʱ���������0xffΪ��Чֵ
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

//�ϱ���ʱ���ȥ�̶�ֵms
bool decreaceReportTime(uint32_t timeMs)
{
    uint8_t i = 0;
    uint8_t index = 0;

    index = returnMinTimeIndex();
    //�ϱ����
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

//�����ϱ�ʱ���
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


// Copyright 2016 Silicon Laboratories, Inc.
#ifndef __ATTRIBUTE_REPORT_HEADER__
#define __ATTRIBUTE_REPORT_HEADER__

///////////////////////////////////////////////////////////////////////////////
// ���ܣ���ӵ������ϱ����񣨷������ϱ�����Ҫϵͳѭ����ִ�У��������ӵ�����ͬ
//       ���������֮ǰ�����ϱ�ʱ��
//
// ������endpoint - Դ�˵�
//       clusterId - cluster ID
//       attributeId - ����ID
//       delayTimeMs - �ϱ���ʱʱ��
//
// ����ֵ��TRUE-������ӳɹ���FALSE-���������
///////////////////////////////////////////////////////////////////////////////
extern bool addReportingSchedule(uint8_t endpoint,
                          EmberAfClusterId clusterId,
                          EmberAfAttributeId attributeId,
                          uint32_t delayTimeMs);

///////////////////////////////////////////////////////////////////////////////
// ���ܣ���Ӷ������ϱ�����֧�������ϱ�����Ҫϵͳѭ����ִ�У��������ӵ�����
//       ͬ���������֮ǰ�����ϱ�ʱ��
//
// ������endpoint - Դ�˵�
//       clusterId - cluster ID
//       attributeNum - ��������
//       attributeId[] - ����ID�б�
//       delayTimeMs - �ϱ���ʱʱ��
//       reportNow - �Ƿ������ϱ�,TRUE:ֱ�ӵ��÷��ͣ�������ϵͳѭ�������������ô˺�
//                  ���ϱ�����;FALSE:����������
//
// ����ֵ��TRUE-������ӳɹ���FALSE-���������
///////////////////////////////////////////////////////////////////////////////
extern bool addMultiReportingSchedule(uint8_t endpoint,
                                EmberAfClusterId clusterId,
                                uint8_t attributeNum,
                                EmberAfAttributeId attributeId[],
                                uint32_t delayTimeMs,
                                bool reportNow);
#endif
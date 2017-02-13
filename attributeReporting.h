// Copyright 2016 Silicon Laboratories, Inc.
#ifndef __ATTRIBUTE_REPORT_HEADER__
#define __ATTRIBUTE_REPORT_HEADER__

///////////////////////////////////////////////////////////////////////////////
// 功能：添加单属性上报任务（非立即上报，需要系统循环后执行），如果添加的是相同
//       属性则更新之前属性上报时间
//
// 参数：endpoint - 源端点
//       clusterId - cluster ID
//       attributeId - 属性ID
//       delayTimeMs - 上报延时时间
//
// 返回值：TRUE-任务添加成功，FALSE-任务表已满
///////////////////////////////////////////////////////////////////////////////
extern bool addReportingSchedule(uint8_t endpoint,
                          EmberAfClusterId clusterId,
                          EmberAfAttributeId attributeId,
                          uint32_t delayTimeMs);

///////////////////////////////////////////////////////////////////////////////
// 功能：添加多属性上报任务（支持立即上报，需要系统循环后执行），如果添加的是相
//       同属性则更新之前属性上报时间
//
// 参数：endpoint - 源端点
//       clusterId - cluster ID
//       attributeNum - 属性数量
//       attributeId[] - 属性ID列表
//       delayTimeMs - 上报延时时间
//       reportNow - 是否立即上报,TRUE:直接调用发送，不经过系统循环，可连续调用此函
//                  数上报属性;FALSE:不立即发送
//
// 返回值：TRUE-任务添加成功，FALSE-任务表已满
///////////////////////////////////////////////////////////////////////////////
extern bool addMultiReportingSchedule(uint8_t endpoint,
                                EmberAfClusterId clusterId,
                                uint8_t attributeNum,
                                EmberAfAttributeId attributeId[],
                                uint32_t delayTimeMs,
                                bool reportNow);
#endif
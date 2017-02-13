/**
******************************************************************************
* @file    RingBufferUtils.h 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This header contains function prototypes called by ring buffer 
*          operation
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, MXCHIP Inc. SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2014 MXCHIP Inc.</center></h2>
******************************************************************************
*/ 

#ifndef __RingBufferUtils_h__
#define __RingBufferUtils_h__

//#include "Common.h"
#include "app/framework/include/af.h"
#include "stack/include/ember.h"
#include "app/framework/util/util.h"
#include "adapter-command.h"


#ifndef MIN
#define MIN(x,y)  ((x) < (y) ? (x) : (y))
#endif /* ifndef MIN */

bool ring_buffer_init( tsMsgRingBuffer* psMsgRingBuffer, const tsSL_Msg* psMsgBuffer, uint32_t size );
bool ring_buffer_deinit( tsMsgRingBuffer* psMsgRingBuffer );

uint32_t ring_buffer_free_space( tsMsgRingBuffer* psMsgRingBuffer );

uint32_t ring_buffer_used_space( tsMsgRingBuffer* psMsgRingBuffer );

uint8_t ring_buffer_get_data( tsMsgRingBuffer* psMsgRingBuffer, tsSL_Msg** sSL_Msg);

uint8_t ring_buffer_del_data( tsMsgRingBuffer* psMsgRingBuffer);

uint8_t ring_buffer_consume( tsMsgRingBuffer* psMsgRingBuffer, uint32_t bytes_consumed );

uint32_t ring_buffer_write( tsMsgRingBuffer* psMsgRingBuffer, const tsSL_Msg* sSL_Msg);

#endif // __RingBufferUtils_h__



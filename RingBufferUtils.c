/**
******************************************************************************
* @file    RingBufferUtils.c
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file contains function called by ring buffer operation
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

#include "RingBufferUtils.h"




/****************************************************************************
* Function	: ring_buffer_init
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
bool ring_buffer_init( tsMsgRingBuffer* psMsgRingBuffer, const tsSL_Msg* psMsgBuffer, uint32_t size )
{
    psMsgRingBuffer->psSL_MsgBuffer     = (tsSL_Msg*)psMsgBuffer;
    psMsgRingBuffer->size       = size;
    psMsgRingBuffer->head       = 0;
    psMsgRingBuffer->tail       = 0;
    psMsgRingBuffer->isFull		= 0;
    return 0;
}



/****************************************************************************
* Function	: ring_buffer_deinit
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
bool ring_buffer_deinit( tsMsgRingBuffer* psMsgRingBuffer )
{
    //UNUSED_PARAMETER(ring_buffer);
    (void)psMsgRingBuffer;
    return 0;
}


/****************************************************************************
* Function	: ring_buffer_free_space
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
uint32_t ring_buffer_free_space( tsMsgRingBuffer* psMsgRingBuffer )
{
    uint8_t free_size = 0;
    if(psMsgRingBuffer->isFull)
        return free_size;

    uint32_t tail_to_end = psMsgRingBuffer->size - psMsgRingBuffer->tail;
    free_size = ((tail_to_end + psMsgRingBuffer->head) % psMsgRingBuffer->size);
    if(free_size == 0)
        free_size = psMsgRingBuffer->size;

    return free_size;
}


/****************************************************************************
* Function	: ring_buffer_used_space
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
uint32_t ring_buffer_used_space( tsMsgRingBuffer* psMsgRingBuffer )
{
    uint8_t used_size = psMsgRingBuffer->size;
    if(psMsgRingBuffer->isFull)
        return used_size;

    uint32_t head_to_end = psMsgRingBuffer->size - psMsgRingBuffer->head;
    used_size = ((head_to_end + psMsgRingBuffer->tail) % psMsgRingBuffer->size);

    return used_size;
}


/****************************************************************************
* Function	: ring_buffer_get_data
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
uint8_t ring_buffer_get_data( tsMsgRingBuffer* psMsgRingBuffer, tsSL_Msg** sSL_Msg)
{
    if(ring_buffer_used_space(psMsgRingBuffer))
    {
        *sSL_Msg = &(psMsgRingBuffer->psSL_MsgBuffer[psMsgRingBuffer->head]);
        return 0;
    }
	else
	{
		emberAfAppPrintln("buffer empty");		
	}

    return -1;
}


/****************************************************************************
* Function	: ring_buffer_del_data
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
uint8_t ring_buffer_del_data( tsMsgRingBuffer* psMsgRingBuffer)
{
    if(ring_buffer_used_space(psMsgRingBuffer))
    {
        psMsgRingBuffer->head ++;
        psMsgRingBuffer->head = (psMsgRingBuffer->head % psMsgRingBuffer->size);
        return 0;
    }
	else
	{
		emberAfAppPrintln("buffer empty1");				
	}
    return -1;
}



/****************************************************************************
* Function	: ring_buffer_consume
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
uint8_t ring_buffer_consume( tsMsgRingBuffer* psMsgRingBuffer, uint32_t bytes_consumed )
{
    psMsgRingBuffer->head = (psMsgRingBuffer->head + bytes_consumed) % psMsgRingBuffer->size;
    return 0;
}


/****************************************************************************
* Function	: ring_buffer_write
* Description	:
* Input Para	:
* Output Para	:
* Return Value:
****************************************************************************/
uint32_t ring_buffer_write( tsMsgRingBuffer* psMsgRingBuffer, const tsSL_Msg* sSL_Msg )
{
	if(psMsgRingBuffer->isFull)
		return 0;
    /* Calculate the maximum amount we can copy */
    uint32_t amount_to_copy = 1;

    /* Copy as much as we can until we fall off the end of the buffer */
    memcpy(&psMsgRingBuffer->psSL_MsgBuffer[psMsgRingBuffer->tail], sSL_Msg, sizeof(tsSL_Msg));

    /* Update the tail */
    psMsgRingBuffer->tail = (psMsgRingBuffer->tail + 1) % psMsgRingBuffer->size;

	if(psMsgRingBuffer->tail == psMsgRingBuffer->head)
		psMsgRingBuffer->isFull = TRUE;
	
    return amount_to_copy;
}


void ring_buffer_display(tsMsgRingBuffer* psMsgRingBuffer)
{
	uint8_t i = 0;
	for( i = 0;i < psMsgRingBuffer->size;i++)
		emberAfAppPrintln("eMsgType:%d",psMsgRingBuffer->psSL_MsgBuffer[i].eMsgType);				
}


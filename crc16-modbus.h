/***************************************************************************************
****************************************************************************************
* FILE		: crc16-modbus.h
* Description	: 
*			  
* Copyright (c) 2016 by ORVIBO. All Rights Reserved.
* 
* History:
* Version		Name       		Date			Description
   0.1		sven	2016/11/03	Initial Version
   
****************************************************************************************
****************************************************************************************/
#ifndef _CRC16_MODBUS_H_
#define _CRC16_MODBUS_H_
#include "app/framework/include/af.h"

uint16_t CRC16 (const uint8_t *nData, uint16_t wLength);


#endif /*_CRC16_MODBUS_H_*/




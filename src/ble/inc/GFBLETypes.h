/*
 * Copyright 2017, OYMotion Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */
#ifndef __GF_TYPES_H__
#define __GF_TYPES_H__
#pragma once

#ifdef WIN32
#include <Windows.h>
#endif

#include "gfTypes.h"
using namespace gf;

typedef void				GF_VOID;
typedef GF_INT				GF_BOOL;
#define GF_FALSE			((GF_BOOL)0)
#define GF_TRUE				(GF_BOOL)(!(GF_FALSE))

#define GF_OK				((GF_INT)0)
#define GF_FAIL				((GF_INT)1)
#define GF_SUCCEEDED		(GF_OK == status)

#define BT_ADDRESS_SIZE		6
#define GF_AUTH_FAIL		0x1000

typedef enum {
	GF_ERR_CODE_SCAN_BUSY = 0x60,
	GF_ERR_CODE_NO_RESOURCE = 0x61,
} GF_ERROR_CODE;

#define BLE_DEVICE_NAME_LENGTH 30
struct GF_BLEDevice{
	GF_UINT8   addr_type;
	GF_UINT8	addr[BT_ADDRESS_SIZE];
	GF_CHAR	dev_name[BLE_DEVICE_NAME_LENGTH];
	GF_UINT8   rssi;
	GF_BLEDevice(GF_UINT8 bytes[], GF_UINT8 type)
	{
		for (int i = BT_ADDRESS_SIZE - 1; i >= 0; --i) {
			this->addr[i] = bytes[i];
		}
		addr_type = type;
	}

};

#define	LOGTYPE_CONSOLE 1
#define	LOGTYPE_MFC     2
#define	LOGTYPE_FILE    3
#define	LOGTYPE_MAX     3

class GF_CCallBack
{
public:
	GF_CCallBack(GF_UINT32 eventmask, GF_UINT8 index) :mEventMask(eventmask), mIndex(index){}
	virtual GF_STATUS OnDeviceFound(GF_BLEDevice new_device) = 0;
	virtual GF_STATUS OnEvent(GF_UINT32 event, GF_PUINT8 data, GF_UINT16 length) = 0;

	GF_UINT32 GetEventMask()
	{
		return mEventMask;
	}

	GF_UINT8 GetIndex()
	{
		return mIndex;
	}

private:
	GF_UINT32 mEventMask;//which event is intent to process.
	GF_UINT8 mIndex;
};

typedef enum {
	State_Idle = 0,
	State_Scanning = 1,
	State_Connecting = 2,
}GF_HubState;

struct GF_ConnectedDevice
{
	GF_UINT8 address_type;
	GF_UINT8 address[BT_ADDRESS_SIZE];
	GF_UINT16 handle;
	GF_UINT16 conn_int;
	GF_UINT16 superTO;
	GF_UINT16 slavelatency;
	GF_UINT16 MTUsize;
};

typedef enum {
	ProtocolType_SimpleProfile = 0,
	ProtocolType_DataProtocol = 1,
	ProtocolType_OADService = 2,
	ProtocolType_Invalid = 0xff,
}GF_DeviceProtocolType;

/*Define Control Response Status for gForce Data Protocol*/
#define CONTROL_COMMAND_RESPONSE_OK 0X00

/*Define Control Command Type for gForce Data Protocol*/
#define GET_PROTOCOL_VERSION 	0x00
#define GET_FEATURE_MAP 		0x01
#define GET_DEVICE_NAME 		0x02
#define GET_MODEL_NUMBER 		0x03
#define GET_SERIAL_NUMBER 		0x04
#define GET_HARDWARE_REVISION   0x05
#define GET_FIRMWARE_REVISION   0x06
#define GET_MANUFACTURER_NAME   0X07
#define GET_BATTERY_LEVEL 		0X08
#define GET_TEMPERATURE_LEVEL   0X09

#define POWER_OFF_REQUEST 		0X1D

#define GFORCE_ATT_RESPONSE_LENGTH_OFFSET 3
#define GFORCE_ATT_RESPONSE_HANDLE_OFFSET 4
#define GFORCE_ATT_RESPONSE_DATA_OFFSET   6
/*Define UUID for Service of gForce Data Protocol*/
#define UUID_128_LENGTH 16
const GF_UINT16 GFORCE_SIMPLE_PROTOCOL_SERVICE_UUID = 0xFFF0;
const GF_UINT8 GFORCE_DATA_PROTOCOL_SERVICE_UUID[UUID_128_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0xD0, 0xFF, 0x00, 0xF0 };
const GF_UINT8 GFORCE_OAD_SERVICE_UUID[UUID_128_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0xC0, 0xFF, 0x00, 0xF0 };
const GF_UINT8 GFORCE_DATA_PROTOCOL_CHRAC_CONTROL_COMMAND_UUID[UUID_128_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0xE1, 0xFF, 0x00, 0xF0 };
const GF_UINT8 GFORCE_DATA_PROTOCOL_CHRAC_DATA_NOTIFY_UUID[UUID_128_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0xE2, 0xFF, 0x00, 0xF0 };
const GF_UINT8 GFORCE_OAD_IDENTIFY_CHAR_UUID[UUID_128_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0xC1, 0xFF, 0x00, 0xF0 };
const GF_UINT8 GFORCE_OAD_BLOCK_CHAR_UUID[UUID_128_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0xC2, 0xFF, 0x00, 0xF0 };
const GF_UINT8 GFORCE_OAD_FAST_CHAR_UUID[UUID_128_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x40, 0x51, 0x04, 0xC3, 0xFF, 0x00, 0xF0 };
#endif
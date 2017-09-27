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
#ifndef __REMOTEDEVICE_H__
#define __REMOTEDEVICE_H__

#include "GFBLETypes.h"
#include "LogPrint.h"
#include <NpiInterface.h>
#include "GattClient.h"
#include "Database.h"

#define MODUAL_TAG_RD "Remote Device"

typedef struct
{
	GF_UINT16 event;
	GF_UINT16 length;
	GF_UINT8 data[200];
}MESSAGE;

typedef enum
{
	GF_DEVICE_STATE_IDLE = 0,
	GF_DEVICE_STATE_W4CONN = 1,
	GF_DEVICE_STATE_W4SECU = 2,
	GF_DEVICE_STATE_GATT_PRI_SVC = 3,
	GF_DEVICE_STATE_GATT_INC_SVC = 4,
	GF_DEVICE_STATE_GATT_DIS_CHARC = 5,
	GF_DEVICE_STATE_GATT_READ_CHARC_VALUE = 6,
	GF_DEVICE_STATE_GATT_READ_CHARC_DESCRIPTOR_VALUE = 7,
	GF_DEVICE_STATE_CONNECTED = 8,
} GF_DEVICE_STATE;

typedef enum
{
	GF_DEVICE_EVENT_DEVICE_CONNECTED = WM_USER + 0x500,
	GF_DEVICE_EVENT_SLAVE_SECURY_REQUEST = WM_USER + 0x501,
	GF_DEVICE_EVENT_AUTH_COMPLETE = WM_USER + 0x502,
	GF_DEVICE_EVENT_BOND_COMPLETE = WM_USER + 0x503,
	GF_DEVICE_EVENT_LINK_PARA_UPDATE = WM_USER + 0x504,
	GF_DEVICE_EVENT_ATT_READ_BY_GRP_TYPE_MSG = WM_USER + 0x505,
	GF_DEVICE_EVENT_ATT_READ_BY_TYPE_MSG = WM_USER + 0x506,
	GF_DEVICE_EVENT_ATT_READ_RESP_MSG = WM_USER + 0x507,
	GF_DEVICE_EVENT_ATT_READ_BLOB_RESP_MSG = WM_USER + 0x508,
	GF_DEVICE_EVENT_ATT_ERROR_MSG = WM_USER + 0x509,
	GF_DEVICE_EVENT_ATT_READ_BY_INFO_MSG = WM_USER + 0x510,
	GF_DEVICE_EVENT_ATT_EXCHANGE_MTU_MSG = WM_USER + 0x511,
	GF_DEVICE_EVENT_ATT_WRITE_MSG = WM_USER + 0x512,
	GF_DEVICE_EVENT_EVICE_DISCONNECTED = WM_USER + 0x513,
	GF_DEVICE_EVENT_INVALID = WM_USER + 0x599,
} GF_DEVICE_EVENT;

typedef enum
{
	GF_DEVICE_ROLE_CENTRAL = 0,
	GF_DEVICE_ROLE_PERIPHERAL = 1,
}GF_DEVICE_ROLE;

#define EVENT_DEVICE_CCONNECTED_STATUS 0
#define EVENT_DEVICE_CCONNECTED_CONN_HANDLE_OFFSET 8
#define EVENT_DEVICE_CCONNECTED_ROLE_OFFSET (EVENT_DEVICE_CCONNECTED_CONN_HANDLE_OFFSET + 2)
#define EVENT_DEVICE_CCONNECTED_CONN_INTERVAL_OFFSET (EVENT_DEVICE_CCONNECTED_ROLE_OFFSET + 1)
#define EVENT_DEVICE_CCONNECTED_SLAVE_LATENCY_OFFSET (EVENT_DEVICE_CCONNECTED_CONN_INTERVAL_OFFSET + 2)
#define EVENT_DEVICE_CCONNECTED_CONN_TIMEOUT_OFFSET (EVENT_DEVICE_CCONNECTED_SLAVE_LATENCY_OFFSET + 2)
#define EVENT_DEVICE_CCONNECTED_CLOCK_ACCURACY_OFFSET (EVENT_DEVICE_CCONNECTED_CONN_TIMEOUT_OFFSET + 2)

#define EVENT_SLAVE_SECURY_REQUEST_STATUS_OFFSET 0
#define EVENT_SLAVE_SECURY_REQUEST_HANDLE_OFFSET (EVENT_SLAVE_SECURY_REQUEST_STATUS_OFFSET + 1)
#define EVENT_SLAVE_SECURY_REQUEST_ADDRESS_OFFSET (EVENT_SLAVE_SECURY_REQUEST_HANDLE_OFFSET + 2)
#define EVENT_SLAVE_SECURY_REQUEST_AUTH_TYPE_OFFSET (EVENT_SLAVE_SECURY_REQUEST_ADDRESS_OFFSET + 6)

#define EVENT_AUTH_COMPLETE_STATUS_OFFSET 0
#define EVENT_AUTH_COMPLETE_ENC_ENABLE_OFFSET 4
#define EVENT_AUTH_COMPLETE_KEYSIZE_OFFSET 5
#define EVENT_AUTH_COMPLETE_LTK_OFFSET 34
#define EVENT_AUTH_COMPLETE_DIV_OFFSET 50
#define EVENT_AUTH_COMPLETE_RAND_OFFSET (EVENT_AUTH_COMPLETE_DIV_OFFSET+2)

#define EVENT_BOND_COMPLETE_STATUS_OFFSET 0

#define EVENT_LINK_PARA_UPDATE_STATUS_OFFSET 0
#define EVENT_LINK_PARA_UPDATE_INTERVEL_OFFSET 3
#define EVENT_LINK_PARA_UPDATE_LATENCY_OFFSET (EVENT_LINK_PARA_UPDATE_INTERVEL_OFFSET + 2)
#define EVENT_LINK_PARA_UPDATE_TIMEOUT_OFFSET (EVENT_LINK_PARA_UPDATE_LATENCY_OFFSET + 2)

#define INVALID_HANDLE 0xFFFF

#define EVENT_ATT_EXCHANGE_MTU_MSG_STATUS_OFFSET 0
#define EVENT_ATT_EXCHANGE_MTU_MSG_LEN_OFFSET (EVENT_ATT_EXCHANGE_MTU_MSG_STATUS_OFFSET + 3)
#define EVENT_ATT_EXCHANGE_MTU_MSG_DATA_OFFSET (EVENT_ATT_EXCHANGE_MTU_MSG_LEN_OFFSET + 1)

class GF_CCONTROLCOMMANDQUEUE;

class GF_CRemoteDevice:public Runnable
//class GF_CRemoteDevice
{
public:
	GF_CRemoteDevice(GF_CNpiInterface* minterface, GF_UINT8 addr_type, GF_PUINT8 address, GF_CCallBack* callback);
	~GF_CRemoteDevice();
	GF_VOID Run();
	GF_UINT16 GetHandle()
	{
		return mHandle;
	}

	GF_UINT16 GetConnectInterval()
	{
		return mConnInternal;
	}

	GF_UINT16 GetSupervisionTimeout()
	{
		return mConnTimeout;
	}

	GF_UINT16 GetSlaveLatency()
	{
		return mSlaveLatency;
	}

	GF_UINT16 GetMTUSize()
	{
		return mMTUSize;
	}

	GF_DEVICE_STATE GetState()
	{
		return mState;
	}

	GF_DeviceProtocolType GetProtocolType()
	{
		return mDeviceProtocolType;
	}

	GF_UINT16 GetControlCommandHandle()
	{
		return mControlCommandHandle;
	}

	GF_UINT16 GetOADIdentifyHandle()
	{
		return mOADIdenfityHandle;
	}
	GF_UINT16 GetOADBlockHandle()
	{
		return mOADBlockHandle;
	}
	GF_UINT16 GetOADFastHandle()
	{
		return mOADFastHandle;
	}
	GF_VOID Init();
	GF_VOID DeInit();

	GF_STATUS ProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS IdleStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS W4ConnStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length); 
	GF_STATUS W4SecuStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS W4GattPriSvcStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS W4GattIncSvcStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS W4GattDisCharcStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS W4GattReadCharcValueStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS W4GattReadCharcDesValueStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS DeviceConnectedStateProcessMessage(GF_DEVICE_EVENT event, GF_PUINT8 data, GF_UINT16 length);

	GF_STATUS StartDiscoveryCharacteristic();
	GF_STATUS NextCharacterateristic();
	GF_STATUS DiscoveryDescriptor();

	GF_STATUS ProcessCharacteristicConfiguration(GF_UINT8 currentprisvc, GF_UINT8 currentchrac, GF_UINT8 currentchracdes);

	GF_STATUS OnDeviceConnected(GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS AuthenticationComplete(GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS BondComplete(GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS ConnectionParameterUpdated(GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS ConnectionParameterUpdateRequest(GF_UINT16 conn_interval_min, GF_UINT16 conn_interval_max, GF_UINT16 slave_latence, GF_UINT16 supervision_timeout);
	GF_STATUS WriteCharacteristicValue(GF_UINT16 attribute_handle, GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS ReadCharacteristicValue(GF_UINT16 attribute_handle);
	GF_STATUS SendControlCommand(GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS SendControlCommandInternal(GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS SendOADIdentify(GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS SendOADBlock(GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS SendOADBlockFast(GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS SendOADIdentifyInternal(GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS SendOADBlockInternal(GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS SendOADFastInternal(GF_UINT8 data_length, GF_PUINT8 data);

	GF_STATUS ExchangeMTUSize(GF_UINT16 mtu_size);
	GF_STATUS ExchangeMTUSizeInternal(GF_UINT16 mtu_size);
	GF_UINT8  mAddrType;
	GF_UINT8  mAddr[BT_ADDRESS_SIZE];
	GF_UINT8 mLTK[16];
	GF_UINT16 mDIV;
	GF_UINT8 mRAND[8];
	GF_UINT8 mKeySize;

private:
	GF_VOID CheckProtocolType();

	GF_CHAR   mDevName[BLE_DEVICE_NAME_LENGTH];
	GF_UINT16 mHandle; //connection handle
	GF_DEVICE_ROLE mConnRole;
	GF_UINT16 mConnInternal;  //connection interval
	GF_UINT16 mSlaveLatency;
	GF_UINT16 mConnTimeout;
	GF_UINT8 mClockAccuracy;

	GF_CCallBack* mCallback;
	GF_CNpiInterface *mInterface;
	GF_DEVICE_STATE mState;
	GF_CPCHAR mTag;

	CThread* mThread;
	GF_BOOL mThreadRunning;
	HANDLE mThreadEvent;

	list<MESSAGE*> mMessage;
	CRITICAL_SECTION mMutex;

	GF_CService mService;
	GF_BOOL mNeedSaveService;
	GF_BOOL mNeedSaveLTK;
	GF_UINT16 mMTUSize;

	GF_UINT8 mCurrentPrimaryServiceIndex;
	GF_UINT8 mCurrentCharactericticIndex;
	GF_UINT8 mCurrentCharactericticDescriptorIndex;
	//database
	GF_CDatabase* mDatabase;

	GF_UINT8 N;

	/*indicate the protocpl type the device supported. */
	GF_DeviceProtocolType mDeviceProtocolType;
	GF_UINT16 mControlCommandHandle;
	//OAD char handle
	GF_UINT16 mOADIdenfityHandle;
	GF_UINT16 mOADBlockHandle;
	GF_UINT16 mOADFastHandle;
	GF_CCONTROLCOMMANDQUEUE* mCommandQueue;
};

#define MAX_COMMAND_PAYLOAD 50
#define GATT_TYPE_EXCHANGE_MTU_SIZE 0
#define GATT_TYPE_SEND_CONTROL_COMMAND 1
//OAD Operate
#define GATT_TYPE_SEND_OAD_IDENTIFY    2
#define GATT_TYPE_SEND_OAD_BLOCK       3
#define GATT_TYPE_SEND_OAD_FAST        4
class GF_CGATTCommand
{
public:
	GF_CGATTCommand(GF_UINT8 commandtype, GF_UINT8 length, GF_PUINT8 parameter)
	{
		type = commandtype;
		dataLength = length;
		if (dataLength != 0)
		{
			data = (GF_PUINT8)new GF_UINT8[length];
			memcpy(data, parameter, length);
		}
	}

	~GF_CGATTCommand()
	{
		if (dataLength != 0)
		{
			delete data;
		}
	}

	GF_UINT8 getCommandType()
	{
		return type;
	}

	GF_UINT8 getDataLength()
	{
		return dataLength;
	}

	GF_PUINT8 getData()
	{
		return data;
	}
private:
	GF_UINT8 type;
	GF_UINT8 dataLength;
	GF_PUINT8 data;
};

class GF_CCONTROLCOMMANDQUEUE
{
public:
	GF_CCONTROLCOMMANDQUEUE(GF_CRemoteDevice* device)
	{
		mDevice = device;
		mBusy = GF_FALSE;
		InitializeCriticalSection(&mCommandMutex);
	}

	~GF_CCONTROLCOMMANDQUEUE()
	{
		EnterCriticalSection(&mCommandMutex);
		while (mCommand.size() != 0)
		{
			/*means pending command needs to sent out.*/
			GF_CGATTCommand* command = mCommand.front();
			mCommand.pop_front();
			delete command;
		}
		LeaveCriticalSection(&mCommandMutex);
	}

	void Enqueue(GF_UINT8 type, GF_UINT8 data_length, GF_PUINT8 data)
	{
		EnterCriticalSection(&mCommandMutex);
		if (mBusy != GF_TRUE)
		{
			/*no command is ongoing, we can send command to remote devive directly.*/
			if (GATT_TYPE_EXCHANGE_MTU_SIZE == type)
			{
				GF_UINT16 mtu = data[0] | (data[1] << 8);
				mDevice->ExchangeMTUSizeInternal(mtu);
			}
			else if (GATT_TYPE_SEND_CONTROL_COMMAND == type)
			{
				mDevice->SendControlCommandInternal(data_length, data);
			}
			else if (GATT_TYPE_SEND_OAD_IDENTIFY == type)
			{
				mDevice->SendOADIdentifyInternal(data_length, data);
			}
			else if (GATT_TYPE_SEND_OAD_BLOCK == type)
			{
				mDevice->SendOADBlockInternal(data_length, data);
			}
			mBusy = GF_TRUE;
		}
		else
		{
			/*command is ongoing, we need to enqueue data to list, can not sent them out until last command is finished.*/
			GF_CGATTCommand* command = new GF_CGATTCommand(type, data_length, data);
			mCommand.push_back(command);
		}
		if (GATT_TYPE_SEND_OAD_FAST == type)
		{
			mDevice->SendOADFastInternal(data_length, data);
		}
		//printf("Enqueue with mBusy = %d， queue size = %d \n", mBusy, mCommand.size());
		LeaveCriticalSection(&mCommandMutex);
	}

	void Dequeue()
	{
		EnterCriticalSection(&mCommandMutex);
		if (mCommand.size() != 0)
		{
			/*means pending command needs to sent out.*/
			GF_CGATTCommand* command = mCommand.front();
			if (GATT_TYPE_EXCHANGE_MTU_SIZE == command->getCommandType())
			{
				GF_UINT16 mtu = command->getData()[0] | (command->getData()[1] << 8);
				mDevice->ExchangeMTUSizeInternal(mtu);
			}
			else if (GATT_TYPE_SEND_CONTROL_COMMAND == command->getCommandType())
			{
				mDevice->SendControlCommandInternal(command->getDataLength(), command->getData());
			}
			else if (GATT_TYPE_SEND_OAD_IDENTIFY == command->getCommandType())
			{
				mDevice->SendOADIdentifyInternal(command->getDataLength(), command->getData());
			}
			else if (GATT_TYPE_SEND_OAD_BLOCK == command->getCommandType())
			{
				mDevice->SendOADBlockInternal(command->getDataLength(), command->getData());
			}
			mCommand.pop_front();
			//printf("Dequeue with mBusy = %d， queue size = %d \n", mBusy, mCommand.size());
			delete command;
		}
		else
		{
			mBusy = GF_FALSE;
		}
		LeaveCriticalSection(&mCommandMutex);
	}

private:
	list<GF_CGATTCommand*> mCommand;
	GF_BOOL mBusy;
	GF_CRemoteDevice* mDevice;
	CRITICAL_SECTION mCommandMutex;
};
#endif
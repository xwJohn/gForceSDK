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
#ifndef __ADAPTERMANAGER_H__
#define __ADAPTERMANAGER_H__
#include "GFBLETypes.h"
#include "AdapterManagerInterface.h"
#include <list>
#include <mutex>
using namespace std;

class GF_CDiscoveryService;
class GF_CClientCallback;
class GF_CRemoteDevice;
class GF_CNpiInterface;
class GF_CClientCallback;
class GF_CCallBack;

class GF_CAdapterManager : public GF_CCallBack, public GF_CAdapterManagerInterface
{
public:
	GF_STATUS Init(GF_UINT8 com_num, GF_UINT8 log_type);
	GF_STATUS Deinit();
	GF_STATUS StartScan(GF_UINT8 RSSI_Threshold);
	GF_STATUS StopScan();
	GF_STATUS Connect(GF_PUINT8 addr, UINT8 addr_type, GF_BOOL is_direct_conn);
	GF_STATUS CancelConnect(GF_PUINT8 addr, GF_UINT8 addr_type);
	GF_STATUS Disconnect(GF_UINT16 handle);
	GF_STATUS RegisterClientCallback(GF_CClientCallback* callback)
	{
		mClientCallback = callback;
		return GF_OK;
	}

	GF_STATUS UnregisterClientCallback()
	{
		mClientCallback = NULL;
		return GF_OK;
	}

	GF_STATUS ConfigMtuSize(GF_UINT16 conn_handle, GF_UINT16 MTU_Size);
	GF_STATUS ConnectionParameterUpdate(GF_UINT16 conn_handle, GF_UINT16 conn_interval_min, GF_UINT16 conn_interval_max, GF_UINT16 slave_latence, GF_UINT16 supervision_timeout);
	GF_STATUS WriteCharacteristic(GF_UINT16 conn_handle, GF_UINT16 attribute_handle, GF_UINT8 data_length, GF_PUINT8 data);
	GF_STATUS ReadCharacteristic(GF_UINT16 conn_handle, GF_UINT16 attribute_handle);

    GF_HubState GetHubState();
	GF_UINT8 GetConnectedDeviceNum();
	/*connected_device is output result*/
	GF_STATUS GetConnectedDeviceByIndex(GF_UINT8 index, GF_ConnectedDevice* connected_device);

	GF_STATUS OnEvent(GF_UINT32 event, GF_PUINT8 data, GF_UINT16 length);
	GF_STATUS OnDeviceFound(GF_BLEDevice new_device);
	GF_CAdapterManagerInterface* GetInstance();

	GF_DeviceProtocolType GetDeviceProtocolSupported(GF_UINT16 conn_handle);
	GF_STATUS SendControlCommand(GF_UINT16 conn_handle, ProfileCharType type, GF_UINT8 data_length, GF_PUINT8 data);
	
protected:
	GF_CAdapterManager();
	~GF_CAdapterManager();
	friend class GF_CAdapterManagerInterface;
private:
	static GF_CAdapterManager* mInstance;
	GF_STATUS ConnectComplete(GF_UINT16 handle);
	GF_STATUS ScanFinished();
	GF_STATUS OnConnectEvent(GF_PUINT8 data, GF_UINT16 length);
	GF_CRemoteDevice* GetConnectedDeviceByHandle(GF_UINT16 handle);
	GF_CRemoteDevice* GetDisconnectingDeviceByHandle(GF_UINT16 handle);
	GF_STATUS mIsConnecting;
	GF_STATUS mIsScanning;
	GF_CNpiInterface* mInterface;
	GF_CPCHAR mTag;
	GF_CDiscoveryService* mDS;

	list<GF_CRemoteDevice*> mAvailabeDevice;

	std::mutex m_ConnectingDeviceMutex;
	list<GF_CRemoteDevice*> mConnectingDevice;

	std::mutex mConnectedDeviceMutex;
	list<GF_CRemoteDevice*> mConnectedDevice;

	std::mutex mDisconnectingDeviceMutex;
	list<GF_CRemoteDevice*> mDisconnectingDevice;
	GF_CRemoteDevice* mCancelConnectingDevice;
	GF_CClientCallback* mClientCallback;
};
#endif
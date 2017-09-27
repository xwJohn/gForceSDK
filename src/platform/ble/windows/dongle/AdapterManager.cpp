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
#include "AdapterManager.h"
#include "RemoteDevice.h"
#include "DiscoveryService.h"
#include "ClientCallbackInterface.h"
#include "NpiInterface.h"
#include <LogPrint.h>
#include <assert.h>
#include <mutex>

#define MODUAL_TAG_AM "AdaperManager"

#define ADAPTER_MANAGER_ATT_EVENT  (EVENT_MASK_LINK_PARA_UPDATE_MSG | EVENT_MASK_DEVICE_CONNECTED | EVENT_MASK_INTERNAL_SCAN_FINISHED | EVENT_MASK_ATT_WRITE_MSG | EVENT_MASK_ATT_EXCHANGE_MTU_MSG | EVENT_MASK_ATT_READ_BLOB_RESP_MSG | EVENT_MASK_ATT_NOTI_MSG | EVENT_MASK_ATT_READ_RESP_MSG | EVENT_MASK_ATT_ERROR_MSG | EVENT_MASK_ATT_READ_BY_GRP_TYPE_MSG | EVENT_MASK_ATT_READ_BY_TYPE_MSG | EVENT_MASK_ATT_READ_BY_INFO_MSG)
#define ADAPTER_MANAGER_EVENT (0x0100FD | ADAPTER_MANAGER_ATT_EVENT)
static GF_CAdapterManager* SingleInstance = NULL;
std::mutex SingleInstanceMutex;

GF_CAdapterManager::GF_CAdapterManager() :GF_CCallBack(ADAPTER_MANAGER_EVENT, ADAPTER_MANAGER_CALLBACK_INDEX)
{
	mInterface = new GF_CNpiInterface;
	mTag = MODUAL_TAG_AM;
	mDS = new GF_CDiscoveryService(mInterface, this);
	mIsConnecting = GF_FALSE;
	mIsScanning = GF_FALSE;
	//mInitialized = GF_TRUE;
	mCancelConnectingDevice = NULL;
}

GF_CAdapterManager::~GF_CAdapterManager()
{
	delete mDS;
	delete mInterface;
	mInterface = NULL;
	mDS = NULL;
}

GF_CAdapterManagerInterface* GF_CAdapterManagerInterface::GetInstance()
{
	if (SingleInstance == NULL)
	{
		SingleInstanceMutex.lock();
		if (SingleInstance == NULL)
		{
			SingleInstance = new GF_CAdapterManager;
		}
		SingleInstanceMutex.unlock();
	}

	return SingleInstance;
}

GF_STATUS GF_CAdapterManager::Init(GF_UINT8 com_num, GF_UINT8 log_type)
{
	LOGDEBUG(mTag, "Init... \n");
	GF_STATUS result = GF_FAIL;

	if (mInterface != NULL)
	{
		result = mInterface->Init(com_num, log_type);
	}
	else
	{
		return result;
	}
	
	if (result == GF_OK)
	{
		//register to receive event form NIF
		mInterface->RegisterCallback(this);
		mInterface->InitDevice();
	}
	else
	{
		return result;
	}
	
	if (mDS != NULL)
	{
		result = mDS->Init();
	}

	do{
		std::lock_guard<std::mutex> lg(m_ConnectingDeviceMutex);
		mConnectingDevice.clear();
	}while(0);

	do{
		std::lock_guard<std::mutex> lg(mConnectedDeviceMutex);
		mConnectedDevice.clear();
	}while(0);

	do
	{
		std::lock_guard<std::mutex> lg(mDisconnectingDeviceMutex);
		mDisconnectingDevice.clear();
	}while(0);

	return result;
}

GF_STATUS GF_CAdapterManager::Deinit()
{
	LOGDEBUG(mTag, "Deinit... \n");
	/*timeout is 50 * 100 = 5s*/
	UINT8 delay = 50;
	GF_STATUS result = GF_FAIL;
	LOGDEBUG(mTag, "before mConnectedDevice.size() = %d \n", GetConnectedDeviceNum());
	LOGDEBUG(mTag, "before mDisconnectingDevice.size() = %d \n", mDisconnectingDevice.size());
	/*wait for device disconnected if we have initial disconnect process avoid meory leak.*/
	while (delay > 0 && mDisconnectingDevice.size())
	{
		LOGDEBUG(mTag, "delay = %d \n", delay);
		if(mDisconnectingDevice.size() != 0)
		{
			Sleep(100);
		}
		else
		{
			break;
		}
		delay--;
	}

	if (mDS != NULL)
	{
		result = mDS->Deinit();
	}

	if (mInterface != NULL)
	{
		result = mInterface->UnRegisterCallback(this);
	}

	if (mInterface != NULL)
	{
		result = mInterface->Deinit();
	}
	return result;
}

/*RSSI_Threshold is used to filter device by RSSI
* if RSSI_Threshold = 0, all device will be sent to client,
* if RSSI_Threshold = x, only device with rssi > x will be sent to client.
*/
GF_STATUS GF_CAdapterManager::StartScan(GF_UINT8 RSSI_Threshold)
{
	/*cannot start to scaning while scanning or connecting is on the way.*/
	if (mIsScanning == GF_TRUE || mIsConnecting == GF_TRUE)
	{
		return GF_ERR_CODE_SCAN_BUSY;
	}

	if (mDS != NULL)
	{
		//start to scan
		mIsScanning = GF_TRUE;
		return mDS->StartScan(RSSI_Threshold);
	}
	else
	{
		return GF_ERR_CODE_NO_RESOURCE;
	}
}

GF_STATUS GF_CAdapterManager::StopScan()
{
	if (mDS != NULL)
	{
		//start to scan
		mIsScanning = GF_FALSE;
		return mDS->StopScan();
	}
	else
	{
		return GF_FAIL;
	}
}

GF_STATUS GF_CAdapterManager::OnDeviceFound(GF_BLEDevice new_device)
{
	GF_STATUS result = GF_OK;
	LOGDEBUG(mTag, "OnNewDeviceFound... \n");
	if (mClientCallback != NULL)
	{
		mClientCallback->onScanResult(&new_device);
	}
	return result;
}

GF_STATUS GF_CAdapterManager::ScanFinished()
{
	GF_STATUS result = GF_FAIL;
	LOGDEBUG(mTag, "OnScanFinished... \n");
	mIsScanning = GF_FALSE;

	if (mClientCallback != NULL)
	{
		mClientCallback->onScanFinished();
	}
	return result;
}

GF_HubState GF_CAdapterManager::GetHubState()
{
	if(mIsConnecting == GF_TRUE)
	{
		return State_Connecting;
	}
	else if(mIsScanning == GF_TRUE)
	{
		return State_Scanning;
	}
	else
	{
		return State_Idle;
	}
}

GF_UINT8 GF_CAdapterManager::GetConnectedDeviceNum()
{
	GF_UINT8 num;
	std::lock_guard<std::mutex> lg(mConnectedDeviceMutex);
	num = mConnectedDevice.size();

	return num;
}

GF_STATUS GF_CAdapterManager::GetConnectedDeviceByIndex(GF_UINT8 index, GF_ConnectedDevice* connected_device)
{
	GF_CRemoteDevice* device = NULL;
	list<GF_CRemoteDevice*>::iterator ii;
	if(index >= GetConnectedDeviceNum())
	{
		return GF_FAIL;
	}

	do
	{
		std::lock_guard<std::mutex> lg(mConnectedDeviceMutex);
		ii = mConnectedDevice.begin();
		while(index > 0)
		{
			ii++;
			index--;
		}
		device = (*ii);
	}while(0);

	if (device != NULL && connected_device != NULL)
	{
		connected_device->address_type = device->mAddrType;
		memcpy(connected_device->address, device->mAddr, BT_ADDRESS_SIZE);
		connected_device->handle = device->GetHandle();
		connected_device->conn_int = device->GetConnectInterval();
		connected_device->slavelatency = device->GetSlaveLatency();
		connected_device->superTO = device->GetSupervisionTimeout();
		connected_device->MTUsize = device->GetMTUSize();

		return GF_OK;
	}
	else
	{
		return GF_FAIL;
	}
}

GF_STATUS GF_CAdapterManager::Connect(GF_PUINT8 addr, UINT8 addr_type, GF_BOOL is_direct_conn)
{
	LOGDEBUG(mTag, "start to Connect... \n");
	GF_STATUS result = GF_FAIL;
	if (is_direct_conn == GF_TRUE && mIsConnecting == GF_TRUE)
	{
		return GF_FAIL;
	}

	if (mIsScanning == GF_TRUE)
	{
		return GF_FAIL;
	}

	GF_CRemoteDevice *device = new GF_CRemoteDevice(mInterface, addr_type, addr, this);
	assert(device);

	if (mInterface != NULL)
	{
		if (is_direct_conn == GF_TRUE)
		{
			std::lock_guard<std::mutex> lg(m_ConnectingDeviceMutex);
			result = mInterface->Connect(addr, addr_type, GF_FALSE);
			mConnectingDevice.push_front(device);
			mIsConnecting = GF_TRUE;
		}
	}
	return result;
}

GF_STATUS GF_CAdapterManager::CancelConnect(GF_PUINT8 addr, GF_UINT8 addr_type)
{
	LOGDEBUG(mTag, "start to CancelConnect... \n");
	GF_STATUS result = GF_FAIL;
	list<GF_CRemoteDevice*>::iterator ii;

	LOGDEBUG(mTag, "mConnectingDevice.size() = %d \n", mConnectingDevice.size());
	for (ii = mConnectingDevice.begin(); ii != mConnectingDevice.end();)
	{
		if (((*ii)->mAddrType == addr_type) && memcmp((*ii)->mAddr, addr, BT_ADDRESS_SIZE) == 0)
		{
			mCancelConnectingDevice = (*ii);
			do
			{
				std::lock_guard<std::mutex> lg(m_ConnectingDeviceMutex);
				ii = mConnectingDevice.erase(ii);
			} while (0);
			break;
		}
		else
		{
			ii++;
		}
	}
	LOGDEBUG(mTag, "mConnectingDevice.size() = %d \n", mConnectingDevice.size());

	if (mInterface != NULL)
	{
		result = mInterface->Disconnect(0xFFFE);
		mIsConnecting = GF_FALSE;
		result = GF_OK;
	}

	return result;
}

GF_STATUS GF_CAdapterManager::Disconnect(GF_UINT16 handle)
{
	GF_STATUS result = GF_FAIL;
	list<GF_CRemoteDevice*>::iterator ii;
	LOGDEBUG(mTag, "start to disconnect device... \n");
	GF_CRemoteDevice* device = GetConnectedDeviceByHandle(handle);
	if (device == NULL)
	{
		return result;
	}
	LOGDEBUG(mTag, "before mConnectedDevice.size() = %d \n", GetConnectedDeviceNum());
	LOGDEBUG(mTag, "before mDisconnectingDevice.size() = %d \n", mDisconnectingDevice.size());
	/*remote device thread exit when try to disconnect connection.*/
	for (ii = mConnectedDevice.begin(); ii != mConnectedDevice.end(); )
	{
		if ((*ii)->GetHandle() == handle)
		{
			(*ii)->DeInit();
			do{
				std::lock_guard<std::mutex> lg(mDisconnectingDeviceMutex);
				mDisconnectingDevice.push_front(*ii);
			}while(0);

			do{
				std::lock_guard<std::mutex> lg(mConnectedDeviceMutex);
				mConnectedDevice.erase(ii++);
			}while(0);
		}
		else
		{
			++ii;
		}
	}
	LOGDEBUG(mTag, "after mConnectedDevice.size() = %d \n", GetConnectedDeviceNum());
	LOGDEBUG(mTag, "after mDisconnectingDevice.size() = %d \n", mDisconnectingDevice.size());

	if (mInterface != NULL)
	{
		result = mInterface->Disconnect(handle);
	}
	return result;
}

GF_DeviceProtocolType GF_CAdapterManager::GetDeviceProtocolSupported(GF_UINT16 conn_handle)
{
	GF_DeviceProtocolType ProtocolType = ProtocolType_Invalid;
	LOGDEBUG(mTag, "GetDeviceProtocolSupported \n");
	GF_CRemoteDevice* device = GetConnectedDeviceByHandle(conn_handle);
	if (device != NULL)
	{
		ProtocolType = device->GetProtocolType();
	}

	return ProtocolType;
}

GF_STATUS GF_CAdapterManager::SendControlCommand(GF_UINT16 conn_handle, ProfileCharType type, GF_UINT8 data_length, GF_PUINT8 data)
{
	GF_STATUS result = GF_FAIL;
	LOGDEBUG(mTag, "SendControlCommand \n");
	GF_CRemoteDevice* device = GetConnectedDeviceByHandle(conn_handle);
	if (device != NULL)
	{
		switch(type)
		{
			case ProfileCharType::PROF_DATA_CMD:
				result = device->SendControlCommand(data_length, data);
				break;
			case ProfileCharType::PROF_OAD_IDENTIFY:
				result = device->SendOADIdentify(data_length, data);
				break;
			case ProfileCharType::PROF_OAD_BLOCK:
				result = device->SendOADBlock(data_length, data);
				break;
			case ProfileCharType::PROF_OAD_FAST:
				result = device->SendOADBlockFast(data_length, data);
				break;
			default:
				break;
		}
	}

	return result;
}

GF_STATUS GF_CAdapterManager::OnConnectEvent(GF_PUINT8 data, GF_UINT16 length)
{
	GF_STATUS result = GF_FAIL;
	list<GF_CRemoteDevice*>::iterator ii;
	GF_UINT8 status = data[0];
	GF_UINT8 addr_type = data[1];
	GF_CRemoteDevice* device = NULL;

	if (status != GF_OK)
	{
		LOGDEBUG(mTag, "OnConnectEvent with error status = %d \n", status);
		LOGDEBUG(mTag, "mConnectingDevice.size() = %d \n", mConnectingDevice.size());
		mIsConnecting = GF_FALSE;
		GF_ConnectedDevice connecteddevice;
		memset(&connecteddevice, 0, sizeof(connecteddevice));
		connecteddevice.address_type = mCancelConnectingDevice->mAddrType;
		memcpy(connecteddevice.address, mCancelConnectingDevice->mAddr, BT_ADDRESS_SIZE);
		connecteddevice.handle = INVALID_HANDLE;
		/*status != GF_OK means connect to remote device canceled by the caller.*/
		mClientCallback->onDeviceConnected(status, &connecteddevice);

		if (mCancelConnectingDevice != NULL)
		{
			delete mCancelConnectingDevice;
		}
		return GF_FAIL;
	}

	LOGDEBUG(mTag, "OnConnectEvent LOGDEBUG with length = %d \n" , length);

	for (GF_UINT16 i = 0; i < length; i++)
	{
		LOGDEBUG(mTag, "the data of [%d]th bytes is 0x%02x \n", i, data[i]);
	}
	LOGDEBUG(mTag, "mConnectingDevice.size() = %d \n", mConnectingDevice.size());
	
	for (ii = mConnectingDevice.begin(); ii != mConnectingDevice.end();)
	{
		LOGDEBUG(mTag, "mConnectedDevice.size() = %d \n", mConnectedDevice.size());
		if (((*ii)->mAddrType == addr_type) && memcmp((*ii)->mAddr, data + 2, BT_ADDRESS_SIZE) == 0)
		{
			(*ii)->Init();

			do{
				std::lock_guard<std::mutex> lg(mConnectedDeviceMutex);
				mConnectedDevice.push_front(*ii);
			}while(0);

			(*ii)->ProcessMessage(GF_DEVICE_EVENT_DEVICE_CONNECTED, data, length);

			do{
				std::lock_guard<std::mutex> lg(m_ConnectingDeviceMutex);
				ii = mConnectingDevice.erase(ii);
			} while (0);
			mIsConnecting = GF_FALSE;
		}
		else
		{
			++ii;
		}
	}

	return result;
}

GF_CRemoteDevice* GF_CAdapterManager::GetConnectedDeviceByHandle(GF_UINT16 handle)
{
	list<GF_CRemoteDevice*>::iterator ii;
	std::lock_guard<std::mutex> lg(mConnectedDeviceMutex);
	for (ii = mConnectedDevice.begin(); ii != mConnectedDevice.end(); ii++)
	{
		if (((*ii)->GetHandle()) == handle)
		{
			return (*ii);
		}
	}

	return NULL;
}

GF_CRemoteDevice* GF_CAdapterManager::GetDisconnectingDeviceByHandle(GF_UINT16 handle)
{
	list<GF_CRemoteDevice*>::iterator ii;
	std::lock_guard<std::mutex> lg(mDisconnectingDeviceMutex);
	for (ii = mDisconnectingDevice.begin(); ii != mDisconnectingDevice.end(); ii++)
	{
		if (((*ii)->GetHandle()) == handle)
		{
			return (*ii);
		}
	}

	return NULL;
}


GF_STATUS  GF_CAdapterManager::ConfigMtuSize(GF_UINT16 conn_handle, GF_UINT16 MTU_Size)
{
	GF_CRemoteDevice* device = GetConnectedDeviceByHandle(conn_handle);
	if (device != NULL)
	{
		return device->ExchangeMTUSize(MTU_Size);
	}
	else
	{
		return GF_FAIL;
	}
}

GF_STATUS GF_CAdapterManager::ConnectionParameterUpdate(GF_UINT16 conn_handle, GF_UINT16 conn_interval_min, GF_UINT16 conn_interval_max, GF_UINT16 slave_latence, GF_UINT16 supervision_timeout)
{
	GF_CRemoteDevice* device = GetConnectedDeviceByHandle(conn_handle);
	if (device != NULL)
	{
		return device->ConnectionParameterUpdateRequest(conn_interval_min, conn_interval_max, slave_latence, supervision_timeout);
	}
	else
	{
		return GF_FAIL;
	}

}

GF_STATUS GF_CAdapterManager::WriteCharacteristic(GF_UINT16 conn_handle, GF_UINT16 attribute_handle, GF_UINT8 data_length, GF_PUINT8 data)
{
	GF_CRemoteDevice* device = GetConnectedDeviceByHandle(conn_handle);
	if (device != NULL)
	{
		return device->WriteCharacteristicValue(attribute_handle, data_length, data);
	}
	else
	{
		return GF_FAIL;
	}
}

GF_STATUS GF_CAdapterManager::ReadCharacteristic(GF_UINT16 conn_handle, GF_UINT16 attribute_handle)
{
	GF_CRemoteDevice* device = GetConnectedDeviceByHandle(conn_handle);
	if (device != NULL)
	{
		return device->ReadCharacteristicValue(attribute_handle);
	}
	else
	{
		return GF_FAIL;
	}
}

GF_STATUS GF_CAdapterManager::OnEvent(GF_UINT32 event, GF_PUINT8 data, GF_UINT16 length)
{
	GF_STATUS result = GF_OK;
	GF_DEVICE_EVENT message = GF_DEVICE_EVENT_INVALID;
	list<GF_CRemoteDevice*>::iterator ii;
	GF_UINT8 handle_offset;

	switch(event)
	{
		case EVENT_SERIAL_PORT_NOT_AVAILABLE:
		{
			if (mClientCallback != NULL)
			{
				mClientCallback->onComDestory();
			}
			return GF_OK;
		}
		case EVENT_MASK_ATT_EXCHANGE_MTU_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_EXCHANGE_MTU_MSG;
			handle_offset = 1;
			break;
		}
		case EVENT_MASK_ATT_READ_BY_GRP_TYPE_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_READ_BY_GRP_TYPE_MSG;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_ATT_READ_BY_TYPE_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_READ_BY_TYPE_MSG;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_ATT_ERROR_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_ERROR_MSG;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_ATT_READ_RESP_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_READ_RESP_MSG;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_ATT_READ_BLOB_RESP_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_READ_BLOB_RESP_MSG;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_ATT_READ_BY_INFO_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_READ_BY_INFO_MSG;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_ATT_WRITE_MSG:
		{
			message = GF_DEVICE_EVENT_ATT_WRITE_MSG;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_ATT_NOTI_MSG:
		{
#if 0
			LOGDEBUG(mTag, "--------->notification received! \n");
			for (GF_UINT16 i = 0; i < length; i++)
			{
				LOGDEBUG(mTag, "the notificationdata of [%d]th bytes is 0x%02x \n", i, data[i]);
			}
#endif
			handle_offset = 1;
			GF_UINT16 handle = data[handle_offset] + (data[handle_offset + 1] << 8);
			GF_CRemoteDevice* device = GetConnectedDeviceByHandle(handle);
			GF_UINT16 atttibute_handle;

			if (device != NULL)
			{
				GF_DEVICE_STATE state = device->GetState();
				if (state == GF_DEVICE_STATE_CONNECTED && mClientCallback != NULL)
				{
					/*for notification from device that support data protocol*/
					if (ProtocolType_DataProtocol == device->GetProtocolType())
					{
						atttibute_handle = data[GFORCE_ATT_RESPONSE_HANDLE_OFFSET] + (data[GFORCE_ATT_RESPONSE_HANDLE_OFFSET + 1] << 8);
						GF_UINT8 length = data[GFORCE_ATT_RESPONSE_LENGTH_OFFSET];
						LOGDEBUG(mTag, "--------->notification atttibute_handle = %x! \n", atttibute_handle);
						/*for control command response*/
						if ( atttibute_handle == device->GetControlCommandHandle())
						{
							mClientCallback->onControlResponseReceived(handle, length - 2, data + GFORCE_ATT_RESPONSE_DATA_OFFSET);
						}
						else
						{
							mClientCallback->onNotificationReceived(handle, length, data + 4);
						}
					}
					/*for notification from device that support oad protocol*/
					else if (ProtocolType_OADService == device->GetProtocolType())
					{
						atttibute_handle = data[GFORCE_ATT_RESPONSE_HANDLE_OFFSET] + (data[GFORCE_ATT_RESPONSE_HANDLE_OFFSET + 1] << 8);
						GF_UINT8 length = data[GFORCE_ATT_RESPONSE_LENGTH_OFFSET];
						LOGDEBUG(mTag, "--------->notification atttibute_handle = %x! \n", atttibute_handle);
						/*for OAD Char response*/
						if ( atttibute_handle == device->GetOADIdentifyHandle())
						{
							/*if this char notify received that means oad failed*/
							mClientCallback->onOADFailedReceived(handle);
						}
						else if (atttibute_handle == device->GetOADBlockHandle())
						{
							/*if fast mode supported,ignore this char notify*/
							if(INVALID_HANDLE == device->GetOADFastHandle())
							{
								mClientCallback->onOADBlockRequestReceived(handle, length - 2, data + GFORCE_ATT_RESPONSE_DATA_OFFSET);
							}
						}
						else
						{
							mClientCallback->onOADFastRequestReceived(handle, length - 2, data + GFORCE_ATT_RESPONSE_DATA_OFFSET);
						}
					}
					else
					{
						mClientCallback->onNotificationReceived(handle, length-3, data + 3);
					}
				}
			}
			break;
		}

		case EVENT_MASK_SLAVE_REQUESTED_SECURITY_MSG:
		{
			message = GF_DEVICE_EVENT_SLAVE_SECURY_REQUEST;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_AUTH_COMPLETE_MSG:
		{
			message = GF_DEVICE_EVENT_AUTH_COMPLETE;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_BOND_COMPLETE_MSG:
		{
			message = GF_DEVICE_EVENT_BOND_COMPLETE;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_LINK_PARA_UPDATE_MSG:
		{
			message = GF_DEVICE_EVENT_LINK_PARA_UPDATE;
			handle_offset = 1;
			break;
		}

		case EVENT_MASK_GAP_LINK_ESTABLISHED_MSG:
		{
			OnConnectEvent(data, length);
			return GF_OK;
		}

		case EVENT_MASK_INTERNAL_SCAN_FINISHED:
		{
			ScanFinished();
			break;
		}

		case EVENT_MASK_GAP_LINK_TERMINATED_MSG:
		{
			handle_offset = 1;
			GF_UINT16 handle = data[handle_offset] + (data[handle_offset + 1] << 8);
			GF_UINT8 reason = data[3];
			LOGDEBUG(mTag, "EVENT_MASK_GAP_LINK_TERMINATED_MSG handle = %d\n", handle);
			GF_CRemoteDevice* device = GetConnectedDeviceByHandle(handle);
			/*connected state -> disconnected state*/
			if (device != NULL)
			{
				/*remove device from connected list.*/
				for (ii = mConnectedDevice.begin(); ii != mConnectedDevice.end(); )
				{
					if ((*ii)->GetHandle() == handle)
					{
						(*ii)->DeInit();
						do
						{
							std::lock_guard<std::mutex> lg(mConnectedDeviceMutex);
							mConnectedDevice.erase(ii++);
						}while(0);
					}
					else
					{
						++ii;
					}
				}
			}
			/*disconnecting state -> disconnected state*/
			else
			{
				device = GetDisconnectingDeviceByHandle(handle);
				if (device != NULL)
				{
					/*remove device from disconnecting list.*/
					LOGDEBUG(mTag, "EVENT_MASK_GAP_LINK_TERMINATED_MSG found in disconnecting device list\n");
					for (ii = mDisconnectingDevice.begin(); ii != mDisconnectingDevice.end(); )
					{
						if ((*ii)->GetHandle() == handle)
						{
							do{
								std::lock_guard<std::mutex> lg(mDisconnectingDeviceMutex);
								mDisconnectingDevice.erase(ii++);
							}while(0);
						}
						else
						{
							++ii;
						}
					}

				}
				else
				{
					LOGERROR(mTag, "handle = %d mapped to none device! \n", handle);
				}
			}

			if (device != NULL)
			{
				/*send message to upper level*/
				LOGDEBUG(mTag, "EVENT_MASK_GAP_LINK_TERMINATED_MSG delete device \n");
				GF_ConnectedDevice disconnecteddevice;
				memset(&disconnecteddevice, 0, sizeof(disconnecteddevice));
				disconnecteddevice.address_type = device->mAddrType;
				memcpy(disconnecteddevice.address, device->mAddr, BT_ADDRESS_SIZE);
				disconnecteddevice.handle = device->GetHandle();
				if (mClientCallback != NULL)
				{
					mClientCallback->onDeviceDisconnected(GF_OK, &disconnecteddevice, reason);
				}
				delete device;
			}
			break;
		}

		default:
			break;
	}

	if (message != GF_DEVICE_EVENT_INVALID)
	{
		LOGDEBUG(mTag, "OnEvent with length = %d \n", length);
		for (GF_UINT16 i = 0; i < length; i++)
		{
			//LOGDEBUG(mTag, "the data of [%d]th bytes is 0x%02x \n", i, data[i]);
		}

		GF_UINT16 handle = data[handle_offset] + (data[handle_offset + 1] << 8);
		GF_CRemoteDevice* device = GetConnectedDeviceByHandle(handle);
		if (device != NULL)
		{
			LOGDEBUG(mTag, "call ProcessMessage \n");
			device->ProcessMessage(message, data, length);

			if (event == EVENT_MASK_ATT_EXCHANGE_MTU_MSG)
			{
				GF_DEVICE_STATE state = device->GetState();
				if (state == GF_DEVICE_STATE_CONNECTED && mClientCallback != NULL)
				{
					mClientCallback->onMTUSizeChanged(data[0], handle, device->GetMTUSize());
				}
			}
			else if (event == EVENT_MASK_LINK_PARA_UPDATE_MSG)
			{
				GF_DEVICE_STATE state = device->GetState();
				if (state == GF_DEVICE_STATE_CONNECTED && mClientCallback != NULL)
				{
					mClientCallback->onConnectionParmeterUpdated(data[0], handle, device->GetConnectInterval(), device->GetSupervisionTimeout(), device->GetSlaveLatency());
				}
			}
			else if (event == EVENT_MASK_ATT_READ_RESP_MSG)
			{
				GF_DEVICE_STATE state = device->GetState();
				if (state == GF_DEVICE_STATE_CONNECTED && mClientCallback != NULL)
				{
					mClientCallback->onCharacteristicValueRead(data[0], handle, data[GF_ATT_READ_VALUE_RESP_LEN_OFFSET], data + GF_ATT_READ_VALUE_RESP_DATA_OFFSET);
				}
			}
		}

	} 
	else
	{
		if (event == EVENT_MASK_GAP_STATUS_MSG)
		{
			LOGDEBUG(mTag, "EVENT_MASK_GAP_STATUS_MSG with length = %d \n", length);
			for (GF_UINT16 i = 0; i < length; i++)
			{
				LOGDEBUG(mTag, "the data of [%d]th bytes is 0x%02x \n", i, data[i]);
			}
		}
		else if (event == EVENT_MASK_DEVICE_CONNECTED)
		{
			GF_UINT16 handle = data[0] + (data[1] << 8);
			GF_CRemoteDevice* device = GetConnectedDeviceByHandle(handle);
			if (mClientCallback != NULL && device != NULL)
			{
				GF_ConnectedDevice connecteddevice;
				connecteddevice.address_type = device->mAddrType;
				memcpy(connecteddevice.address, device->mAddr, BT_ADDRESS_SIZE);
				connecteddevice.handle = device->GetHandle();
				connecteddevice.conn_int = device->GetConnectInterval();
				connecteddevice.slavelatency = device->GetSlaveLatency();
				connecteddevice.superTO = device->GetSupervisionTimeout();
				connecteddevice.MTUsize = device->GetMTUSize();
				mClientCallback->onDeviceConnected(GF_OK, &connecteddevice);
			}
		}
	}

	return result;
}


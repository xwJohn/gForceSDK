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
#include "LogUtils.h"
#include "BLEHub.h"
#include "HubListener.h"
#include "Utils.h"
#include "GFBLETypes.h"

#ifdef BLECOMMAND_INTERVAL_ENABLED
#ifndef WIN32
#include <unistd.h>
#endif
#endif

using namespace gf;

BLEHub::BLEHub(const tstring& sIdentifier)
	: mNotifHelper(*this)
	, mWorkMode(WorkMode::Polling)
{
	GF_LOGD("BLEHub++: identfiler: %s", utils::tostring(sIdentifier).c_str());
}


BLEHub::~BLEHub()
{
	mAM = nullptr;
	GF_LOGD("BLEHub--");
}

GF_UINT32 BLEHub::executeCommand(gfsPtr<HubMsg> msg)
{
	msg->executed = false;
	unique_lock<mutex> lock;
	try {
		lock = unique_lock<mutex>(mTaskMutex);
	}
	catch (const system_error& e) {
		// duplicate lock in the same thread.
		// try polling mode instead.
		GF_LOGD("%s: system_error with code %d, meaning %s",
			__FUNCTION__, e.code().value(), e.what());
		throw e;
		return 0;
	}
	mMsgQ.push(msg);
	msg->syncCallCond.wait(lock, [msg]()->bool { return msg->executed; });
	return msg->ret;
}
void BLEHub::commandTask()
{
	bool loop = true;
	while (loop)
	{
		auto msg = mMsgQ.pop();
		if (nullptr == msg)
		{
			GF_LOGD("Wake me up and exit!");
			break;
		}
		{
			lock_guard<mutex> lock(mTaskMutex);
			msg->ret = msg->fun();
		}
		GF_LOGD("Msg received and executed: ret = %u", msg->ret);
		msg->executed = true;
		msg->syncCallCond.notify_all();
	}
}

// module management
GF_RET_CODE BLEHub::init(GF_UINT8 comPort)
{
	GF_RET_CODE ret = GF_RET_CODE::GF_SUCCESS;
	GF_LOGD(__FUNCTION__);

	lock_guard<mutex> lock(const_cast<mutex&>(mTaskMutex));
	if (nullptr != mAM) {
		return ret;
	}
	mAM = GF_CAdapterManagerInterface::GetInstance();
	if (nullptr == mAM) {
		GF_LOGD("Memory insurfficient.");
		return GF_RET_CODE::GF_ERROR_NO_RESOURCE;
	}
	if (GF_OK != mAM->Init(comPort, LOGTYPE_MFC)) {
		GF_LOGD("Hub init failed.");
		mAM = nullptr;
		// TODO: check return value for reason
		return GF_RET_CODE::GF_ERROR;
	}

	mAM->RegisterClientCallback(this);

	mCommandThread = thread(std::mem_fn(&BLEHub::commandTask), this);

	return ret;
}

GF_RET_CODE BLEHub::deinit()
{
	GF_RET_CODE ret = GF_RET_CODE::GF_SUCCESS;
	GF_LOGD(__FUNCTION__);

	if (nullptr == mAM)
		return ret;

	// get dongle status and stop scan
	if (HubState::Scanning == getState())
	{
		stopScan();
	}

	// stop receiving notif from AM
	mAM->UnregisterClientCallback();

	unique_lock<mutex> lock(mTaskMutex);
	// use a new set to handle all devices state change
	auto devices = mDisconnDevices;
	devices.insert(mConnectedDevices.begin(), mConnectedDevices.end());
	lock.unlock();
	for (auto& itor : devices)
	{
		if (DeviceConnectionStatus::Disconnected != itor->getConnectionStatus())
			itor->disconnect();
	}
	// send empty message to exit command thread
	gfsPtr<HubMsg> emptyEvent;
	mMsgQ.push(emptyEvent);
	if (mCommandThread.joinable())
	{
		mCommandThread.join();
	}

	lock.lock();
	mMsgQ.clear();
	mDisconnDevices.clear();
	mConnectedDevices.clear();
	for (auto& itor : devices)
	{
		mNotifHelper.onDeviceDiscard(itor);
	}
	devices.clear();

	if (GF_OK != mAM->Deinit()) {
		GF_LOGE("Hub deinit failed.");
	}
	mAM = nullptr;

	lock_guard<mutex> locklistener(mMutexListeners);
	mListeners.clear();

	return ret;
}

HubState BLEHub::getState()
{
	HubState ret = HubState::Unknown;
	if (nullptr == mAM) {
		return ret;
	}
	GF_HubState state = static_cast<GF_HubState>(executeCommand(make_shared<HubMsg>([this]()
	{
		return static_cast<GF_UINT32>(mAM->GetHubState());
	})));

	switch (state)
	{
	case State_Idle:
		ret = HubState::Idle;
		break;
	case State_Scanning:
		ret = HubState::Scanning;
		break;
	case State_Connecting:
		ret = HubState::Connecting;
		break;
	default:
		ret = HubState::Disconnected;
	}

	return ret;
}

tstring BLEHub::getDescString() const
{
	return tstring(_T("Welcome to the gForce world."));
}

// setup listener
GF_RET_CODE BLEHub::registerListener(const gfwPtr<HubListener>& listener)
{
	GF_LOGD(__FUNCTION__);
	if (nullptr == listener.lock())
		return GF_RET_CODE::GF_ERROR_BAD_PARAM;

	lock_guard<mutex> lock(mMutexListeners);
	auto ret = mListeners.insert(listener);
	cleanInvalidWeakP(mListeners);
	if (ret.second)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::unRegisterListener(const gfwPtr<HubListener>& listener)
{
	GF_LOGD(__FUNCTION__);
	if (nullptr == listener.lock())
		return GF_RET_CODE::GF_ERROR_BAD_PARAM;

	lock_guard<mutex> lock(mMutexListeners);
	auto before = mListeners.size();
	auto after = mListeners.erase(listener);
	cleanInvalidWeakP(mListeners);
	if (before != after)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::startScan(GF_UINT8 rssiThreshold)
{
	GF_LOGD(__FUNCTION__);
	if (nullptr == mAM)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT32 ret = executeCommand(make_shared<HubMsg>([this, rssiThreshold]()
	{
		GF_UINT32 ret = static_cast<GF_UINT32>(mAM->StartScan(rssiThreshold));
		if (GF_OK == ret)
		{
			// if scan started, remove all disconnected devices
			for (auto itor = mDisconnDevices.begin(); itor != mDisconnDevices.end();)
			{
				if ((*itor)->getConnectionStatus() == DeviceConnectionStatus::Disconnected)
				{
					// notify client that the device will be erased.
					mNotifHelper.onDeviceDiscard(*itor);
					GF_LOGD("Disconnected device removed.");
					mDisconnDevices.erase(itor++);
				}
				else
				{
					itor++;
				}
			}
		}
		return ret;
	}));
	if (GF_OK == ret)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::stopScan()
{
	GF_LOGD(__FUNCTION__);
	if (nullptr == mAM)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT32 ret = executeCommand(make_shared<HubMsg>([this]()
	{
		GF_UINT32 ret = static_cast<GF_UINT32>(mAM->StopScan());
		if (GF_OK == ret)
		{
			GF_LOGD("Scan stopped.");
#ifdef WIN32 // TODO: different behavior between win and android
			mNotifHelper.onScanFinished();
#endif
		}
		return ret;
	}));
	GF_LOGD("end of stopScan. %u", ret);
	if (GF_OK == ret)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

void BLEHub::enumDevices(std::function<bool(SPDEVICE)>& funEnum, bool bConnectedOnly)
{
	GF_LOGD(__FUNCTION__);
	if (nullptr == funEnum)
	{
		GF_LOGD("%s: Bad ptr.", __FUNCTION__);
		return;
	}

	lock_guard<mutex> lock(mTaskMutex);
	for (auto& itor : mConnectedDevices)
	{
		if (!funEnum(itor))
			return; // if client wants to stop, it will return false in funEnum
	}
	if (bConnectedOnly)
		return;

	for (auto& itor : mDisconnDevices)
	{
		if (!funEnum(itor))
			return; // if client wants to stop, it will return false in funEnum
	}
}

SPDEVICE BLEHub::findDevice(GF_UINT8 addrType, tstring address)
{
	GF_LOGD(__FUNCTION__);
	gfsPtr<Device> ret;

	lock_guard<mutex> lock(mTaskMutex);
	for (auto& itor : mConnectedDevices)
	{
		if (itor->isMyself(addrType, address))
		{
			ret = itor;
			return ret;
		}
	}
	for (auto& itor : mDisconnDevices)
	{
		if (itor->isMyself(addrType, address))
		{
			ret = itor;
			return ret;
		}
	}
	return ret;
}

GF_RET_CODE BLEHub::createVirtualDevice(vector<SPDEVICE> realDevices, SPDEVICE& newDevice)
{
	GF_LOGD(__FUNCTION__);
	return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; // not implemented
}

void BLEHub::onScanResult(GF_BLEDevice* device)
{
	GF_LOGD(__FUNCTION__);
	if (device == nullptr)
		return;

	// newly scanned should not be connected
	auto newItem = createDeviceBeforeConnect(*device);
	if (nullptr == newItem)
		return;

	{
		lock_guard<mutex> lock(mTaskMutex);

		pair<decltype(mDisconnDevices.begin()), bool> ret = mDisconnDevices.insert(newItem);
		auto exists = mDisconnDevices.find(newItem);
		// if an existing device has the same info, it will take place of the new item
		// then we will only update the existing item: ret.second == false
		newItem = *exists;
		if (ret.second == false)
		{
			newItem->updateData(*device);
		}
	}
	mNotifHelper.onDeviceFound(newItem);
}

void BLEHub::onScanFinished()
{
	GF_LOGD(__FUNCTION__);
	mNotifHelper.onScanFinished();
}

void BLEHub::onDeviceConnected(GF_STATUS status, GF_ConnectedDevice *device)
{
	GF_LOGD(__FUNCTION__);
	ASSERT_VALID_PTR(device);
	if (nullptr == device)
		return;

	// if status is not GF_OK, it means connecting to remote device canceled by the caller.
	if (GF_OK != status)
	{
		GF_LOGW("onDeviceConnected: status is %u, turn to onDeviceDisconnected.", status);
		onDeviceDisconnected(status, device, 0);
		return;
	}

	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mDisconnDevices)
		{
			if (itor->isMyself(device->address_type, device->address))
			{
				dev = itor;
				mDisconnDevices.erase(itor);
				break;
			}
		}
		if (nullptr == dev)
		{
			GF_LOGW("No device found in disconnect_list. %u:%s",
				(GF_UINT)device->address_type,
				utils::deviceAddressToString(device->address, sizeof(device->address)).c_str());
			for (auto& itor : mConnectedDevices)
			{
				if (itor->isMyself(device->address_type, device->address))
				{
					dev = itor;
					break;
				}
			}
		}
		if (nullptr == dev)
		{
			GF_LOGW("No device found in connect_list either. %u:%s",
				(GF_UINT)device->address_type,
				utils::deviceAddressToString(device->address, sizeof(device->address)).c_str());

			// TODO: add new device using GF_ConnectedDevice
			// need more info to do it
			return;
		}
		mConnectedDevices.insert(dev);
	}
	dev->onConnected(status, *device);
	mNotifHelper.onDeviceConnected(dev);
}

void BLEHub::onDeviceDisconnected(GF_STATUS status, GF_ConnectedDevice *device, GF_UINT8 reason)
{
	GF_LOGD(__FUNCTION__);
	ASSERT_VALID_PTR(device);
	if (nullptr == device)
		return;

	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (device->handle == itor->getHandle())
			{
				dev = itor;
				mConnectedDevices.erase(itor);
				break;
			}
		}
		if (nullptr == dev)
		{
			GF_LOGW("No device found in connect_list. handle is %u, %u:%s, reason is %u",
				(GF_UINT)device->handle, (GF_UINT)device->address_type,
				utils::deviceAddressToString(device->address, sizeof(device->address)).c_str(),
				(GF_UINT)reason);
			for (auto& itor : mDisconnDevices)
			{
				if (itor->isMyself(device->address_type, device->address))
				{
					dev = itor;
					break;
				}
			}
		}
		if (nullptr == dev)
		{
			GF_LOGW("No device found in disconnect_list. either handle is %u, %u:%s, reason is %u",
				(GF_UINT)device->handle, (GF_UINT)device->address_type,
				utils::deviceAddressToString(device->address, sizeof(device->address)).c_str(),
				(GF_UINT)reason);
			return;
		}
		mDisconnDevices.insert(dev);
	}
	dev->onDisconnected(status, reason);
	mNotifHelper.onDeviceDisconnected(dev, reason);
}


void BLEHub::onMTUSizeChanged(GF_STATUS status, GF_UINT16 handle, GF_UINT16 mtu_size)
{
	GF_LOGD(__FUNCTION__);

	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onMTUSizeChanged(status, mtu_size);
}

void BLEHub::onConnectionParmeterUpdated(GF_STATUS status, GF_UINT16 handle, GF_UINT16 conn_int, GF_UINT16 superTO, GF_UINT16 slavelatency)
{
	GF_LOGD(__FUNCTION__);

	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onConnectionParmeterUpdated(status, conn_int, superTO, slavelatency);
}

void BLEHub::onCharacteristicValueRead(GF_STATUS status, GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
{
	GF_LOGD(__FUNCTION__);

	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onCharacteristicValueRead(status, length, data);
}


/*Notification format: data length(1 byte N) + data(N Bytes)*/
void BLEHub::onNotificationReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
{
	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onCharNotify(ProfileCharType::PROF_DATA_NTF, length, data);
}

void BLEHub::onControlResponseReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
{
	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onCharNotify(ProfileCharType::PROF_DATA_CMD, length, data);
}

void BLEHub::onOADFailedReceived(GF_UINT16 handle)
{
	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onCharNotify(ProfileCharType::PROF_OAD_IDENTIFY, 0, nullptr);
}

void BLEHub::onOADBlockRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
{
	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onCharNotify(ProfileCharType::PROF_OAD_BLOCK, length, data);
}

void BLEHub::onOADFastRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
{
	gfsPtr<BLEDevice> dev;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor->getHandle() == handle)
			{
				dev = itor;
				break;
			}
		}
	}
	if (nullptr != dev)
		dev->onCharNotify(ProfileCharType::PROF_OAD_FAST, length, data);
}
void BLEHub::onComDestory()
{
	GF_LOGD(__FUNCTION__);
	// TODO: do something after com destoryed

	lock_guard<mutex> lock(mTaskMutex);
	mNotifHelper.onStateChanged(HubState::Disconnected);
}


GF_RET_CODE BLEHub::connect(BLEDevice& dev, bool directConn)
{
	if (nullptr == mAM)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT8 addr[BT_ADDRESS_SIZE];
	GF_RET_CODE ret = dev.getAddress(addr, sizeof(addr));
	if (GF_RET_CODE::GF_SUCCESS != ret)
		return ret;
	GF_UINT8 type = dev.getAddrType();
	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, &addr, type, directConn]()
	{
		GF_STATUS status = mAM->Connect(addr, type, directConn ? GF_TRUE : GF_FALSE);
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::cancelConnect(BLEDevice& dev)
{
	GF_RET_CODE ret = GF_RET_CODE::GF_SUCCESS;
	if (nullptr == mAM)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;
	GF_UINT8 addr[BT_ADDRESS_SIZE];
	ret = dev.getAddress(addr, sizeof(addr));
	if (GF_RET_CODE::GF_SUCCESS != ret)
		return ret;
	auto type = dev.getAddrType();
	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, &addr, type]()
	{
		GF_STATUS status = mAM->CancelConnect(addr, type);
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::disconnect(BLEDevice& dev)
{
	auto handle = dev.getHandle();
	if (nullptr == mAM || INVALID_HANDLE == handle)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, handle]()
	{
		GF_STATUS status = mAM->Disconnect(handle);
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::configMtuSize(BLEDevice& dev, GF_UINT16 mtuSize)
{
	auto handle = dev.getHandle();
	if (nullptr == mAM || INVALID_HANDLE == handle)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

#ifdef BLECOMMAND_INTERVAL_ENABLED
	auto now = chrono::system_clock::now();
	if (now - mLastExecTime < chrono::milliseconds(BLECOMMAND_INTERVAL))
	{
		auto delay = chrono::milliseconds(BLECOMMAND_INTERVAL) - (now - mLastExecTime);
		auto msdelay = chrono::duration_cast<chrono::milliseconds>(delay).count();
		GF_LOGD("HOLD: %lld", msdelay);
		utils::sleep((GF_UINT32)msdelay);
	}
	mLastExecTime = chrono::system_clock::now();
#endif
	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, handle, mtuSize]()
	{
		GF_STATUS status = mAM->ConfigMtuSize(handle, mtuSize);
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::connectionParameterUpdate(BLEDevice& dev, GF_UINT16 conn_interval_min,
	GF_UINT16 conn_interval_max, GF_UINT16 slave_latence, GF_UINT16 supervision_timeout)
{
	auto handle = dev.getHandle();
	if (nullptr == mAM || INVALID_HANDLE == handle)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, handle,
		conn_interval_min, conn_interval_max, slave_latence, supervision_timeout]()
	{
		GF_STATUS status = mAM->ConnectionParameterUpdate(handle, conn_interval_min,
			conn_interval_max, slave_latence, supervision_timeout);
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::writeCharacteristic(BLEDevice& dev,
	AttributeHandle attribute_handle, GF_UINT8 data_length, GF_PUINT8 data)
{
	auto handle = dev.getHandle();
	if (nullptr == mAM || INVALID_HANDLE == handle)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, handle,
		attribute_handle, data_length, data]()
	{
		GF_STATUS status = mAM->WriteCharacteristic(handle, static_cast<GF_UINT16>(attribute_handle),
			data_length, data);
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::readCharacteristic(BLEDevice& dev, AttributeHandle attribute_handle)
{
	auto handle = dev.getHandle();
	if (nullptr == mAM || INVALID_HANDLE == handle)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, handle, attribute_handle]()
	{
		GF_STATUS status = mAM->ReadCharacteristic(handle, static_cast<GF_UINT16>(attribute_handle));
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::getProtocol(BLEDevice& dev, DeviceProtocolType& type)
{
	auto handle = dev.getHandle();
	GF_DeviceProtocolType protoType = ProtocolType_Invalid;
	if (nullptr == mAM || INVALID_HANDLE == handle)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

	GF_UINT32 status = GF_OK;
	executeCommand(make_shared<HubMsg>([this, handle, &protoType]()
	{
		protoType = mAM->GetDeviceProtocolSupported(handle);
		return static_cast<GF_UINT32>(GF_OK);
	}));
	switch (protoType)
	{
	case ProtocolType_SimpleProfile:
		type = DeviceProtocolType::SimpleProfile;
		break;
	case ProtocolType_DataProtocol:
		type = DeviceProtocolType::DataProtocol;
		break;
	case ProtocolType_OADService:
		type = DeviceProtocolType::OADService;
		break;
	default:
		type = DeviceProtocolType::Invalid;
	}
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

GF_RET_CODE BLEHub::sendControlCommand(BLEDevice& dev, ProfileCharType type, GF_UINT8 data_length, GF_PUINT8 data)
{
	auto handle = dev.getHandle();
	if (nullptr == mAM || INVALID_HANDLE == handle)
		return GF_RET_CODE::GF_ERROR_BAD_STATE;

#ifdef BLECOMMAND_INTERVAL_ENABLED
	auto now = chrono::system_clock::now();
	if (now - mLastExecTime < chrono::milliseconds(BLECOMMAND_INTERVAL))
	{
		auto delay = chrono::milliseconds(BLECOMMAND_INTERVAL) - (now - mLastExecTime);
		auto msdelay = chrono::duration_cast<chrono::milliseconds>(delay).count();
		GF_LOGD("HOLD: %lld", msdelay);
		utils::sleep((GF_UINT32)msdelay);
	}
	mLastExecTime = chrono::system_clock::now();
#endif
	GF_UINT32 status = executeCommand(make_shared<HubMsg>([this, handle, type,
		data_length, data]()
	{
		GF_STATUS status = mAM->SendControlCommand(handle, type,
			data_length, data);
		return static_cast<GF_UINT32>(status);
	}));
	if (GF_OK == status)
		return GF_RET_CODE::GF_SUCCESS;
	else
		return GF_RET_CODE::GF_ERROR;
}

void BLEHub::notifyOrientationData(BLEDevice& dev, const Quaternion& rotation)
{
	SPDEVICE device;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor.get() == &dev)
			{
				device = itor;
				break;
			}
		}
	}
	if (nullptr != device)
		mNotifHelper.onOrientationData(device, rotation);
}

void BLEHub::notifyGestureData(BLEDevice& dev, Gesture gest)
{
	SPDEVICE device;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor.get() == &dev)
			{
				device = itor;
				break;
			}
		}
	}
	if (nullptr != device)
		mNotifHelper.onGestureData(device, gest);
}

void BLEHub::notifyDeviceStatusChanged(BLEDevice& dev, DeviceStatus status)
{
	SPDEVICE device;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor.get() == &dev)
			{
				device = itor;
				break;
			}
		}
	}
	if (nullptr != device)
		mNotifHelper.onDeviceStatusChanged(device, status);
}

void BLEHub::notifyExtendedData(BLEDevice& dev, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data)
{
	SPDEVICE device;
	{
		lock_guard<mutex> lock(mTaskMutex);
		for (auto& itor : mConnectedDevices)
		{
			if (itor.get() == &dev)
			{
				device = itor;
				break;
			}
		}
	}
	if (nullptr != device)
		mNotifHelper.onExtendedData(device, dataType, data);
}

const char* gForcePrefix = "gForce";

gfsPtr<BLEDevice> BLEHub::createDeviceBeforeConnect(const GF_BLEDevice& bleDev)
{
	gfsPtr<BLEDevice> pDevice;
	if (nullptr != strstr(bleDev.dev_name, gForcePrefix))
	{
		GF_LOGI("Device mathing. %s, rssi = %u", bleDev.dev_name, bleDev.rssi);
		pDevice = make_shared<BLEDevice>(*this, bleDev);
	}
	else
	{
#if ONLY_SCAN_GFORCE_DEVICE
		GF_LOGI("Device unmatch. %s, rssi = %u", bleDev.dev_name, bleDev.rssi);
#else
		pDevice = make_shared<BLEDevice>(*this, bleDev);
#endif
	}
	if (nullptr == pDevice)
		return nullptr;
	pDevice->mMyself = pDevice;
	return pDevice;
	}

void BLEHub::NotifyHelper::onScanFinished()
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor]() {
				auto sp = itor.lock();
				GF_LOGD("%s: callback fn here. %p", __FUNCTION__, sp.get());
				if (nullptr != sp)
					sp->onScanFinished();
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onScanFinished();
		}
	}
}

void BLEHub::NotifyHelper::onStateChanged(HubState state)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, state]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onStateChanged(state);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onStateChanged(state);
		}
	}
}

void BLEHub::NotifyHelper::onDeviceFound(SPDEVICE device)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onDeviceFound(device);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onDeviceFound(device);
		}
	}
}

void BLEHub::NotifyHelper::onDeviceDiscard(SPDEVICE device)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onDeviceDiscard(device);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onDeviceDiscard(device);
		}
	}
}

void BLEHub::NotifyHelper::onDeviceConnected(SPDEVICE device)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onDeviceConnected(device);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onDeviceConnected(device);
		}
	}
}

void BLEHub::NotifyHelper::onDeviceDisconnected(SPDEVICE device, GF_UINT8 reason)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device, reason]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onDeviceDisconnected(device, reason);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onDeviceDisconnected(device, reason);
		}
	}
}

void BLEHub::NotifyHelper::onOrientationData(SPDEVICE device, const Quaternion& rotation)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device, rotation]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onOrientationData(device, rotation);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onOrientationData(device, rotation);
		}
	}
}

void BLEHub::NotifyHelper::onGestureData(SPDEVICE device, Gesture gest)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device, gest]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onGestureData(device, gest);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onGestureData(device, gest);
		}
	}
}

void BLEHub::NotifyHelper::onDeviceStatusChanged(SPDEVICE device, DeviceStatus status)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device, status]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onDeviceStatusChanged(device, status);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onDeviceStatusChanged(device, status);
		}
	}
}

void BLEHub::NotifyHelper::onExtendedData(SPDEVICE device, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data)
{
	lock_guard<mutex> lock(mHub.mMutexListeners);
	if (WorkMode::Polling == mHub.mWorkMode)
	{
		for (auto& itor : mHub.mListeners)
		{
			mHub.mPollMsgQ.push(make_shared<PollingMsg>(
				[itor, device, dataType, data]() {
				auto sp = itor.lock();
				if (nullptr != sp)
					sp->onExtendedDeviceData(device, dataType, data);
			}
			));
		}
	}
	else
	{
		for (auto& itor : mHub.mListeners)
		{
			auto sp = itor.lock();
			if (nullptr != sp)
				sp->onExtendedDeviceData(device, dataType, data);
		}
	}
}

GF_RET_CODE BLEHub::run(GF_UINT32 ms, bool once)
{
	GF_RET_CODE ret = GF_RET_CODE::GF_ERROR_BAD_STATE;
	unique_lock<mutex> lock(mPollMutex, try_to_lock);
	if (!lock.owns_lock())
		return ret;
	if (mWorkMode != WorkMode::Polling)
		return ret;
	if (mPolling)
		return ret;

	mPolling = true;
	ret = GF_RET_CODE::GF_SUCCESS;

	///
	lock.unlock();
	bool loop = !once;
	auto until = chrono::system_clock::now() + chrono::duration<GF_UINT32, milli>(ms);
	GF_UINT32 time_last = ms;
	do {
		gfsPtr<PollingMsg> msg;
		if (!mPollMsgQ.pop_until(msg, until))
		{
			// timer out
			ret = GF_RET_CODE::GF_ERROR_TIMEOUT;
			break;
		}
		if (nullptr == msg)
		{
			GF_LOGE("Error: polling message is null.");
			ret = GF_RET_CODE::GF_ERROR_BAD_STATE;
			break;
		}
		msg->fun();
	} while (loop);
	///

	lock.lock();
	mPolling = false;
	return ret;
}

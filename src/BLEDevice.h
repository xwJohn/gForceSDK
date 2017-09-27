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
// define generic device
#pragma once

#include "Device.h"
#include "IHub.h"
#include "Utils.h"
#include "HubListener.h"
#include "ClientCallbackInterface.h"

#include <set>
#include <mutex>
using namespace std;

namespace gf
{

	const static GF_UINT16 INVALID_HANDLE = 0xFFFF;

	class DeviceProfile;

	class BLEDevice : public Device
	{
	public:
		BLEDevice(IHub& hub, const GF_BLEDevice& bleDev);
		virtual ~BLEDevice();

		virtual GF_UINT8 getAddrType() const override { return mAddrType; }
		virtual GF_RET_CODE getAddress(GF_UINT8* addr, GF_SIZE bufLen) const;
		virtual tstring getAddress() const override;
		virtual tstring getName() const override { return mName; }
		virtual GF_UINT getRssi() const override { return mRssi; }
		virtual GF_UINT16 getHandle() const { return mHandle; }
		virtual GF_UINT16 getConnInt() const { return mConnInt; }
		virtual GF_UINT16 getSuperTO() const { return mSuperTO; }
		virtual GF_UINT16 getSlavelatency() const { return mSlavelatency; }
		virtual GF_UINT16 getMTUsize() const { return mMTUsize; }

		virtual GF_RET_CODE identify(int msec) override;

		virtual DeviceConnectionStatus getConnectionStatus() const override { return mCnntStatus; }
		virtual GF_RET_CODE setAlias(tstring alias) override;
		virtual tstring getAlias() const override { return mAlias; }

		virtual GF_RET_CODE connect(bool directConn = true) override;
		virtual GF_RET_CODE disconnect() override;
		virtual GF_RET_CODE cancelConnect() override;
		virtual gfsPtr<DeviceSetting> getDeviceSetting() override;

		virtual GF_RET_CODE configMtuSize(GF_UINT16 mtuSize);
		virtual GF_RET_CODE connectionParameterUpdate(GF_UINT16 conn_interval_min, GF_UINT16 conn_interval_max,
			GF_UINT16 slave_latence, GF_UINT16 supervision_timeout);
		virtual GF_RET_CODE writeCharacteristic(AttributeHandle attribute_handle, GF_UINT8 dataLen, GF_PUINT8 data);
		virtual GF_RET_CODE readCharacteristic(AttributeHandle attribute_handle);
		virtual GF_RET_CODE sendControlCommand(ProfileCharType type, GF_UINT8 dataLen, GF_PUINT8 data);
		
	public:
		virtual void updateData(const GF_BLEDevice& bleDev);
		virtual void onConnected(GF_STATUS status, const GF_ConnectedDevice& connedDevice);
		virtual void onDisconnected(GF_STATUS status, GF_UINT8 reason);
		virtual void onMTUSizeChanged(GF_STATUS status, GF_UINT16 mtu_size);
		virtual void onConnectionParmeterUpdated(GF_STATUS status, GF_UINT16 conn_int, GF_UINT16 superTO, GF_UINT16 slavelatency);
		virtual void onCharacteristicValueRead(GF_STATUS status, GF_UINT8 length, GF_PUINT8 data);
		virtual void onCharNotify(ProfileCharType type, GF_UINT8 length, GF_PUINT8 data);
	public:
		virtual bool operator < (const BLEDevice& devRight) const;
		virtual bool isMyself(GF_UINT8 addrType, GF_UINT8 addr[]) const;
		virtual bool isMyself(GF_UINT8 addrType, tstring addr) const;
		virtual bool takeover(BLEDevice& from);

	protected:
		void connectStatusChanged(DeviceConnectionStatus status);

	public:
		// provide to profiles
		IHub& getHub() { return mHub; }
		mutex& getLockable() { return mMutex; }

		// WPDEVICE ptr for util objects creating
		// must be assigned right after a device created
		gfwPtr<BLEDevice> mMyself;

	protected:
		IHub& mHub;
		// TODO: thread-safe
		mutex mMutex;

	protected:
		DeviceConnectionStatus mCnntStatus = DeviceConnectionStatus::Disconnected;
		tstring		mAlias;
		GF_UINT8	mAddrType = 0;
		GF_UINT8	mAddress[BT_ADDRESS_SIZE];
		tstring		mName;
		GF_UINT8	mRssi = 0;

		GF_UINT16	mHandle = INVALID_HANDLE;
		GF_UINT16	mConnInt = 0;
		GF_UINT16	mSuperTO = 0;
		GF_UINT16	mSlavelatency = 0;
		GF_UINT16	mMTUsize = 0;

		gfsPtr<DeviceProfile> mProfile;
	};

} // namespace gf

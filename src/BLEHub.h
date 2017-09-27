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
#pragma once

#include <atomic>
#include <thread>
#include <set>

#include "Hub.h"
#include "IHub.h"
#include "BLEDevice.h"
#include "Utils.h"

#include "HubManager.h"
#include "ClientCallbackInterface.h"
#include "AdapterManagerInterface.h"


#define ONLY_SCAN_GFORCE_DEVICE (1)

/////////////////////////////////////
// disable the below define since AM has already support it.
//#define BLECOMMAND_INTERVAL_ENABLED
#ifdef BLECOMMAND_INTERVAL_ENABLED
#define BLECOMMAND_INTERVAL (100) // ms
#endif

namespace gf
{
	class BLEHub :
		public Hub, public GF_CClientCallback, public IHub
	{
	public:
		BLEHub(const tstring& sIdentifier);
		virtual ~BLEHub();

		//////////////////////////////////////////////////////////////
		// Derived from class Hub
		// module management
		virtual GF_RET_CODE init(GF_UINT8 comPort) override;
		virtual GF_RET_CODE deinit() override;
		virtual WorkMode getWorkMode() const override {
			return mWorkMode;
		}
		virtual void setWorkMode(WorkMode newMode) override {
			GF_LOGI("setWorkMode. %d", static_cast<int>(newMode));
			if (WorkMode::Polling == newMode)
				GF_LOGI("Please call method run(ms) to pull message.");
			mWorkMode = newMode;
		}
		// get status, version, etc.
		virtual HubState getState() override;
		virtual tstring getDescString() const override;

		// setup listener
		virtual GF_RET_CODE registerListener(const gfwPtr<HubListener>& listener) override;
		virtual GF_RET_CODE unRegisterListener(const gfwPtr<HubListener>& listener) override;

		virtual GF_RET_CODE startScan(GF_UINT8 rssiThreshold) override;
		virtual GF_RET_CODE stopScan() override;

		virtual GF_SIZE getNumOfDevices(bool bConnectedOnly = true) const override {
			lock_guard<mutex> lock(const_cast<mutex&>(mTaskMutex));
			if (bConnectedOnly)
				return mConnectedDevices.size();
			else
				return mDisconnDevices.size() + mConnectedDevices.size();
		}
		virtual void enumDevices(std::function<bool(SPDEVICE)>& funEnum, bool bConnectedOnly = true) override;
		virtual SPDEVICE findDevice(GF_UINT8 addrType, tstring address) override;
		// /// \brief create a virtual device with more than one physical gforce devices
		// ///
		// ///   By setting up virtual device, client can combine two or more gForce devices positioning in
		// ///   defferent location (such as right arm + left arm) into one virtual device. client still can receive gesture
		// ///   data of these devices seperately, and can get one combined gesture
		// ///   data (hug for example) if it happens
		// ///
		// /// \param realDevices A list of physical gForce devices to combine the virtual devices.
		// /// \param [out] newDevice New virtual device combined by given devices.
		// /// \return GF_RET_CODE::GF_SUCCESS if succeeded, others otherwise.
		// /// \remark Not implemented yet.
		virtual GF_RET_CODE createVirtualDevice(vector<SPDEVICE> realDevices, SPDEVICE& newDevice) /*override*/;

	protected:
		//////////////////////////////////////////////////////////////
		// Derived from class IClientCallback to receive BLE event
		virtual void onScanResult(GF_BLEDevice* device) override;
		virtual void onScanFinished() override;
		virtual void onDeviceConnected(GF_STATUS status, GF_ConnectedDevice *device) override;
		virtual void onDeviceDisconnected(GF_STATUS status, GF_ConnectedDevice *device, GF_UINT8 reason) override;

		virtual void onMTUSizeChanged(GF_STATUS status, GF_UINT16 handle, GF_UINT16 mtu_size) override;
		virtual void onConnectionParmeterUpdated(GF_STATUS status, GF_UINT16 handle,
			GF_UINT16 conn_int, GF_UINT16 superTO, GF_UINT16 slavelatency) override;
		virtual void onCharacteristicValueRead(GF_STATUS status, GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) override;

		/*Notification format: data length(1 byte N) + data(N Bytes)*/
		virtual void onNotificationReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) override;

		/*Control command response from device that support gForce data protocol.
		*Notification format: Response status(1 byte) + cmd type(1 byte) + Response parameter(depends on cmd type)*/
		virtual void onControlResponseReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) override;

		/*Characteristic notify from device that support OAD protocol*/
		virtual void onOADFailedReceived(GF_UINT16 handle) override;
		virtual void onOADBlockRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) override;
		virtual void onOADFastRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) override;
		
		virtual void onComDestory() override;

	protected:
		//////////////////////////////////////////////////////////////
		// IHub for device usage
		virtual GF_RET_CODE connect(BLEDevice& dev, bool directConn = true) override;
		virtual GF_RET_CODE cancelConnect(BLEDevice& dev) override;
		virtual GF_RET_CODE disconnect(BLEDevice& dev) override;
		virtual GF_RET_CODE configMtuSize(BLEDevice& dev, GF_UINT16 mtuSize) override;
		virtual GF_RET_CODE connectionParameterUpdate(BLEDevice& dev,
			GF_UINT16 conn_interval_min, GF_UINT16 conn_interval_max,
			GF_UINT16 slave_latence, GF_UINT16 supervision_timeout) override;
		virtual GF_RET_CODE writeCharacteristic(BLEDevice& dev,
			AttributeHandle attribute_handle, GF_UINT8 data_length, GF_PUINT8 data) override;
		virtual GF_RET_CODE readCharacteristic(BLEDevice& dev, AttributeHandle attribute_handle) override;
		virtual GF_RET_CODE getProtocol(BLEDevice& dev, DeviceProtocolType& type) override;
		virtual GF_RET_CODE sendControlCommand(BLEDevice& dev, ProfileCharType type, GF_UINT8 data_length, GF_PUINT8 data) override;
		virtual void notifyOrientationData(BLEDevice& dev, const Quaternion& rotation) override;
		virtual void notifyGestureData(BLEDevice& dev, Gesture gest) override;
		virtual void notifyDeviceStatusChanged(BLEDevice& dev, DeviceStatus status) override;
		virtual void notifyExtendedData(BLEDevice& dev, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data) override;

	protected:
		//////////////////////////////////////////////////////////////
		// device factory methods
		gfsPtr<BLEDevice> createDeviceBeforeConnect(const GF_BLEDevice& bleDev);
	protected:
		//////////////////////////////////////////////////////////////
		// generic members
		GF_CAdapterManagerInterface* mAM = nullptr;
		set<gfsPtr<BLEDevice>, DevComp<gfsPtr<BLEDevice>>> mDisconnDevices;
		set<gfsPtr<BLEDevice>, DevComp<gfsPtr<BLEDevice>>> mConnectedDevices;
		set<gfwPtr<HubListener>, owner_less<gfwPtr<HubListener>>> mListeners;
		mutex mMutexListeners;

	protected:
		//////////////////////////////////////////////////////////////
		// Inner working thread to process call to AdapterManager
		struct HubMsg
		{
			function<GF_UINT32()> fun;
			GF_UINT32 ret = 0;
			atomic<bool> executed; // don't initialize here, put into constructor for compatible in Android
			condition_variable syncCallCond;
			HubMsg(decltype(fun) foo)
			 : fun(foo)
			 , executed(false)
			 {
			 }
		};
		GF_UINT32 executeCommand(gfsPtr<HubMsg> msg);
		void commandTask();
		mutex mTaskMutex;
		thread mCommandThread;
		BQueue<gfsPtr<HubMsg>> mMsgQ;
#ifdef BLECOMMAND_INTERVAL_ENABLED
		chrono::system_clock::time_point mLastExecTime;
#endif

	public:
		//////////////////////////////////////////////////////////////
		// Inner working thread to process Polling mode
		virtual GF_RET_CODE run(GF_UINT32 ms, bool once = false) override;
	protected:
		class NotifyHelper
		{
		public:
			NotifyHelper(BLEHub& theHub) : mHub(theHub) {}
			virtual void onScanFinished();
			virtual void onStateChanged(HubState state);
			virtual void onDeviceFound(SPDEVICE device);
			virtual void onDeviceDiscard(SPDEVICE device);
			// regarding the device data/status updates
			virtual void onDeviceConnected(SPDEVICE device);
			virtual void onDeviceDisconnected(SPDEVICE device, GF_UINT8 reason);

			virtual void onOrientationData(SPDEVICE device, const Quaternion& rotation);
			virtual void onGestureData(SPDEVICE device, Gesture gest);
			virtual void onDeviceStatusChanged(SPDEVICE device, DeviceStatus status);
			virtual void onExtendedData(SPDEVICE device, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data);

			// remove warning
			NotifyHelper& operator = (const NotifyHelper& right) = delete;
		private:
			BLEHub& mHub;
		} mNotifHelper;
		struct PollingMsg
		{
			function<void()> fun;
			PollingMsg(decltype(fun) foo) : fun(foo) {}
		};
		BQueue<gfsPtr<PollingMsg>> mPollMsgQ;
		atomic<WorkMode> mWorkMode; // don't initialize here, put into constructor for compatible in Android
		mutex mPollMutex;
		bool mPolling = false;
		condition_variable mPollCond;
	};


} // namespace gf

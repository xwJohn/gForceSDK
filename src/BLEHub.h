#pragma once

#include <mutex>
#include <set>

#include "Hub.h"
#include "IHub.h"
#include "BLEDevice.h"
#include "Utils.h"

#include "HubManager.h"
#include "ClientCallbackInterface.h"
#include "AdapterManagerInterface.h"

namespace gf
{

	class BLEHub :
		public Hub, public GF_CClientCallback, public IHub
	{
	public:
		BLEHub(const tstring& sIdentifier);
		virtual ~BLEHub();


		// module management
		virtual GF_RET_CODE init(GF_UINT8 comPort);
		virtual GF_RET_CODE deinit();
		// get status, version, etc.
		virtual HubState getStatus() const;
		virtual tstring getDescString() const;

		// setup listener
		virtual GF_RET_CODE registerListener(const gfwPtr<HubListener>& listener);
		virtual GF_RET_CODE unRegisterListener(const gfwPtr<HubListener>& listener);

		virtual GF_RET_CODE startScan(GF_UINT8 rssiThreshold);
		virtual GF_RET_CODE stopScan();

		virtual GF_SIZE getNumOfDevices() const { return mDisconnDevices.size() + mConnectedDevices.size(); }
		virtual GF_SIZE getNumOfConnectedDevices() const { return mConnectedDevices.size(); }
		virtual void enumDevices(FunEnumDevice funEnum, bool bConnectedOnly = true);
		virtual WPDEVICE findDevice(GF_UINT8 addrType, tstring address);
		// set up virtual device, client can combine two or more gdevices positioning in
		//   defferent location into one virtual device. client still can receive gesture
		//   data of these devices seperately, and can get one combined gesture
		//   data if it happens
		virtual GF_RET_CODE createVirtualDevice(int numDevices, vector<WPDEVICE> realDevices, WPDEVICE& newDevice);

	protected:

		virtual void onScanResult(GF_BLEDevice* device);
		virtual void onScanFinished();
		virtual void onDeviceConnected(GF_STATUS status, GF_ConnectedDevice *device);
		virtual void onDeviceDisonnected(GF_STATUS status, GF_ConnectedDevice *device, GF_UINT8 reason);

		virtual void onMTUSizeChanged(GF_STATUS status, GF_UINT16 handle, GF_UINT16 mtu_size);
		virtual void onConnectionParmeterUpdated(GF_STATUS status, GF_UINT16 handle,
			GF_UINT16 conn_int, GF_UINT16 superTO, GF_UINT16 slavelatency);
		virtual void onCharacteristicValueRead(GF_STATUS status, GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data);

		/*Notification format: data length(1 byte N) + data(N Bytes)*/
		virtual void onNotificationReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data);

		virtual void onComDestory();

	protected:
		// IHub for device usage
		virtual GF_RET_CODE connect(BLEDevice& dev, bool directConn = true);
		virtual GF_RET_CODE cancelConnect(BLEDevice& dev);
		virtual GF_RET_CODE disconnect(BLEDevice& dev);
		virtual GF_RET_CODE configMtuSize(BLEDevice& dev, GF_UINT16 mtuSize);
		virtual GF_RET_CODE connectionParameterUpdate(BLEDevice& dev,
			GF_UINT16 conn_interval_min, GF_UINT16 conn_interval_max,
			GF_UINT16 slave_latence, GF_UINT16 supervision_timeout);
		virtual GF_RET_CODE writeCharacteristic(BLEDevice& dev,
			AttributeHandle attribute_handle, GF_UINT8 data_length, GF_PUINT8 data);
		virtual GF_RET_CODE readCharacteristic(BLEDevice& dev, AttributeHandle attribute_handle);

	protected:
		// device factory methods
		gfsPtr<BLEDevice> createDeviceBeforeConnect(const GF_BLEDevice& bleDev);
	protected:
		GF_CAdapterManagerInterface* mAM = nullptr;
		set<gfsPtr<BLEDevice>, DevComp<gfsPtr<BLEDevice>>> mDisconnDevices;
		set<gfsPtr<BLEDevice>, ConnectedDevComp<gfsPtr<BLEDevice>>> mConnectedDevices;
		set<gfwPtr<HubListener>, WeakPtrComp<HubListener>> mListeners;
	};


} // namespace gf
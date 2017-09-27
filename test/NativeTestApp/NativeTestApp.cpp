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
 // NativeTestApp.cpp : Defines the entry point for the console application.
 //
#include "stdafx.h"

#define TAG "NativeTestApp"

#include "HubManager.h"
#include "Device.h"
#include "HubListener.h"
#include "DeviceSetting.h"
#include "LogUtils.h"
#include "../common/Utils.h"

#include <atomic>
#include <list>
#include <algorithm>
#include <thread>

using namespace std;
using namespace gf;

#define DEFAULT_RSSI 0xA0

atomic<bool> bExiting = false;

BOOL ctrlhandler(DWORD fdwctrltype)
{
	switch (fdwctrltype)
	{
	case CTRL_C_EVENT:
		// handle the ctrl-c signal.
		GF_LOGI("ctrl-c event\n");
		bExiting = true;
		break;
	case CTRL_CLOSE_EVENT:
		// ctrl-close: confirm that the user wants to exit.
		GF_LOGI("ctrl-close event");
		bExiting = true;
		break;
	case CTRL_BREAK_EVENT:
		// pass other signals to the next handler.
		GF_LOGI("ctrl-break event");
		bExiting = true;
		break;
	case CTRL_LOGOFF_EVENT:
		GF_LOGI("ctrl-logoff event");
		bExiting = true;
		break;
	case CTRL_SHUTDOWN_EVENT:
		GF_LOGI("ctrl-shutdown event");
		bExiting = true;
		break;
	default:;
	}
	if (bExiting)
		return TRUE;
	else
		return FALSE;
}

void printHelp()
{
	GF_LOGI("\tPress Ctrl+C to exit.\n");
	GF_LOGI("h:\tPrint this help.");
	GF_LOGI("i:\tInit hub.");
	GF_LOGI("u:\tDeinit hub.");
	GF_LOGI("g:\tGet hub status.");
	GF_LOGI("s:\tStart to scan.");
	GF_LOGI("S:\tStop to scan.");
	GF_LOGI("e:\tEnum devices.");
	GF_LOGI("c:\tConnect to device.");
	GF_LOGI("C:\tCancel connecting to device.");
	GF_LOGI("d:\tDisconnect device.");
	GF_LOGI("f:\tFind device.");
	GF_LOGI("m:\tRetrieve device information.");
	GF_LOGI("n:\tTurn notification on.");
	GF_LOGI("o:\tOAD upgrade.");
	GF_LOGI("p:\tPackage ID control.");
	GF_LOGI("x:\tPoweroff.");
	GF_LOGI("y:\tSwtich to OAD.");
	GF_LOGI("z:\tDevice system reset.");
	//GF_LOGI("p:\tPolling mode.\n\n");
	GF_LOGI("q:\tExit.\n\n");
}

string gestureToString(Gesture gest)
{
	string gesture;
	switch (gest)
	{
	case Gesture::Relax:
		gesture = "Relax";
		break;
	case Gesture::Fist:
		gesture = "Fist";
		break;
	case Gesture::SpreadFingers:
		gesture = "SpreadFingers";
		break;
	case Gesture::WaveIn:
		gesture = "WaveIn";
		break;
	case Gesture::WaveOut:
		gesture = "WaveOut";
		break;
	case Gesture::Pinch:
		gesture = "Pinch";
		break;
	case Gesture::Shoot:
		gesture = "Shoot";
		break;
	case Gesture::Undefined:
	default:
	{
		gesture = "Unknown: ";
		string s;
		stringstream ss(s);
		ss << static_cast<int>(gest);
		gesture += ss.str();
	}
	}
	return gesture;
}

string ResponseResultToString(ResponseResult resp)
{
	string respText;
	switch (resp)
	{
	case ResponseResult::RREST_SUCCESS:
		respText = "RREST_SUCCESS->0";
		break;
	case ResponseResult::RREST_NOT_SUPPORT:
		respText = "RREST_NOT_SUPPORT->1---********* NOTICE ME ********";
		break;
	case ResponseResult::RREST_BAD_PARAM:
		respText = "RREST_BAD_PARAM->2---********* NOTICE ME ********";
		break;
	case ResponseResult::RREST_FAILED:
		respText = "RREST_FAILED->3---********* NOTICE ME ********";
		break;
	case ResponseResult::RREST_TIMEOUT:
		respText = "RREST_TIMEOUT->4---********* NOTICE ME ********";
		break;
	default:
	{
		respText = "Unknown->---********* NOTICE ME ********";
		string s;
		stringstream ss(s);
		ss << static_cast<int>(resp);
		respText += ss.str();
	}
	}
	return respText;
}

list<gfsPtr<Device>> listDev;
gfsPtr<HubListener> listener;

class HubListenerImp : public HubListener
{
	virtual void onScanFinished() override
	{

		GF_LOGD("ThreadId: %s: %s", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__);
	}
	virtual void onStateChanged(HubState state) override
	{
		GF_LOGI("ThreadId: %s: %s: HubState: %u", utils::threadIdToString(this_thread::get_id()).c_str(),
			__FUNCTION__, static_cast<GF_UINT>(state));
	}
	virtual void onDeviceFound(SPDEVICE device) override
	{
		GF_LOGI("ThreadId: %s: %s: Name: %s, RSSI: %u", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__,
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()),
			(nullptr == device ? 0 : device->getRssi()));
	}
	virtual void onDeviceDiscard(SPDEVICE device) override
	{
		GF_LOGD("ThreadId: %s: %s: device: %s", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__,
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()));
		//listDev.erase(remove_if(listDev.begin(), listDev.end(),
		//	[&device](decltype(*listDev.begin()) it){ return (device == it); }), listDev.end());

		for (auto itor = listDev.begin(); itor != listDev.end();)
		{
			if (device == (*itor))
				listDev.erase(itor++);
			else
				++itor;
		}
	}
	virtual void onDeviceConnected(SPDEVICE device) override
	{
		GF_LOGD("ThreadId: %s: %s: device: %s", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__,
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()));
	}
	virtual void onDeviceDisconnected(SPDEVICE device, GF_UINT8 reason) override
	{
		GF_LOGD("ThreadId: %s: %s: device: %s, reason: %u", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__,
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()), reason);
	}
	virtual void onOrientationData(SPDEVICE device, const Quaternion& rotation) override
	{
		auto now = chrono::system_clock::now();
		chrono::duration<GF_UINT32, milli> duration(1000);
		if (mLastPrinted + duration > now)
			return;

		mLastPrinted = now;
		GF_LOGD("ThreadId: %s: %s, Quaternion: %s, Eulerian angle: %s",
			utils::threadIdToString(this_thread::get_id()).c_str(),
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()),
			rotation.toString().c_str(), rotation.toEuler().toString().c_str());
	}
	virtual void onGestureData(SPDEVICE device, Gesture gest) override
	{
		string gesture = gestureToString(gest);
		GF_LOGD("ThreadId: %s: %s: Device: %s, Gesture data received: %s", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__,
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()), gesture.c_str());
	}
	virtual void onDeviceStatusChanged(SPDEVICE device, DeviceStatus status) override
	{
		GF_LOGD("ThreadId: %s: %s: device status changed. device = %s, status = %u", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__,
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()), static_cast<GF_UINT32>(status));
	}
	virtual void onExtendedDeviceData(SPDEVICE device, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data) override
	{
		GF_LOGD("ThreadId: %s: %s: device = %s, dataType = %u, length = %u, first byte: %2.2X, last byte: %2.2X", utils::threadIdToString(this_thread::get_id()).c_str(), __FUNCTION__,
			(nullptr == device ? "__empty__" : utils::tostring(device->getName()).c_str()),
			static_cast<GF_UINT32>(dataType), data->size(),
			data->size() > 0 ? data->at(0) : 0xFF,
			data->size() > 0 ? data->at(data->size() - 1) : 0xFF
		);
	}

private:
	chrono::system_clock::time_point mLastPrinted = chrono::system_clock::now();
};

bool enumDevice(SPDEVICE device)
{
	ASSERT_VALID_PTR(device);
	if (nullptr == device)
	{
		GF_LOGW("%s: empty device???", __FUNCTION__);
	}
	else
	{
		GF_LOGI("Dev: addrtype: %u, address: %s, name: %s, connstatus: %u, alias:%s",
			device->getAddrType(), utils::tostring(device->getAddress()).c_str(), utils::tostring(device->getName()).c_str(),
			static_cast<GF_UINT>(device->getConnectionStatus()), utils::tostring(device->getAlias()).c_str());
		listDev.push_back(device);
	}

	// don't want to break enumerate, so always return true
	return true;
}

struct test
{
	test()
	{
		a = 1;
	}
	~test()
	{
		a = 0;
	}
	int a = 0;
};
void handleCmd(gfsPtr<Hub>& pHub, string cmd)
{
	if (cmd.length() == 0)
		return;

	GF_LOGI("Command %s received.", cmd.c_str());
	switch (cmd[0])
	{
	case 'h':
		printHelp();
		break;
	case 'i':
		if (GF_RET_CODE::GF_SUCCESS != pHub->init())
		{
			GF_LOGE("failed to init hub.");
		}
		else
		{
			pHub->registerListener(listener);
			GF_LOGD("done init hub.");
		}
		break;
	case 'u':
		listDev.clear();
		if (GF_RET_CODE::GF_SUCCESS != pHub->deinit())
		{
			GF_LOGE("failed to deinit hub.");
		}
		else
		{
			GF_LOGD("done deinit hub.");
		}
		break;
	case 'g':
		GF_LOGI("hub status is: %u", static_cast<GF_UINT>(pHub->getState()));
		break;
	case 's':
		listDev.clear();
		pHub->startScan(DEFAULT_RSSI);
		break;
	case 'S':
		pHub->stopScan();
		break;
	case 'e':
	{
		GF_LOGI("Total %u devices found, %u of them are connected.",
			pHub->getNumOfDevices(false), pHub->getNumOfDevices(true));
		listDev.clear();
		function<bool(SPDEVICE)> enumFn(enumDevice);
		pHub->enumDevices(enumFn, false);
		break;
	}
	case 'c':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Disconnected == itor->getConnectionStatus())
			{
				itor->connect();
				break;
			}
		}
		break;
	}
	case 'C':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connecting == itor->getConnectionStatus())
			{
				itor->cancelConnect();
				break;
			}
		}
		break;
	}
	case 'd':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				itor->disconnect();
				break;
			}
		}
		break;
	}
	case 'f':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				itor->identify();
				break;
			}
		}
		break;
	}
	case '0':
	{
		gfsPtr<test> t = make_shared<test>();
		gfsPtr<void> v = static_pointer_cast<void>(t);
		t = nullptr;
		v = nullptr;
		break;
		//// entering polling mode
		//pHub->setWorkMode(WorkMode::Polling);
		//GF_LOGI("Hub work mode is %d now.", static_cast<GF_INT>(pHub->getWorkMode()));
		//GF_LOGI("Main thread id is %s.\n", utils::threadIdToString(this_thread::get_id()).c_str());
		//
		//int tryit = 5;
		//while (tryit--)
		//{
		//	pHub->run(2000);
		//}
		//pHub->setWorkMode(WorkMode::Freerun);
		//break;
	}
	case 'm':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected != itor->getConnectionStatus())
				continue;

			tstring str;
			GF_LOGD("Address type: %u", itor->getAddrType());
			GF_LOGD("Address: %s", utils::tostring(itor->getAddress()).c_str());
			GF_LOGD("Name: %s", utils::tostring(itor->getName()).c_str());
			GF_LOGD("Rssi: %u", itor->getRssi());
			auto ds = itor->getDeviceSetting();
			if (nullptr == ds)
				continue;

			GF_UINT32 featureMap = 0;
			ds->getProtocolVer([](ResponseResult res, tstring str) {
				GF_LOGD("getProtocolVer: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getFeatureMap([](ResponseResult res, GF_UINT32 featuremap) {
				GF_LOGD("getFeatureMap: retcode=%s, 0x%8.8X", ResponseResultToString(res).c_str(), featuremap); });
			ds->getDeviceName([](ResponseResult res, tstring str) {
				GF_LOGD("getDeviceName: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getModelNumber([](ResponseResult res, tstring str) {
				GF_LOGD("getModelNumber: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getSerialNumber([](ResponseResult res, tstring str) {
				GF_LOGD("getSerialNumber: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getHWRevision([](ResponseResult res, tstring str) {
				GF_LOGD("getHWRevision: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getFWRevision([](ResponseResult res, tstring str) {
				GF_LOGD("getFWRevision: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getManufacturerName([](ResponseResult res, tstring str) {
				GF_LOGD("getManufacturerName: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getBootloaderVer([](ResponseResult res, tstring str) {
				GF_LOGD("getBootloaderVer: retcode=%s, \'%s\'", ResponseResultToString(res).c_str(), utils::tostring(str).c_str()); });
			ds->getBatteryLevel([](ResponseResult res, GF_UINT32 batlevel) {
				GF_LOGD("getBatteryLevel: retcode=%s, %u", ResponseResultToString(res).c_str(), batlevel); });
			ds->getTemperature([](ResponseResult res, GF_UINT32 temperature) {
				GF_LOGD("getTemperature: retcode=%s, %u", ResponseResultToString(res).c_str(), temperature); });
			// get caps
			ds->getAccelerateCap([](ResponseResult res, GF_UINT16 maxSampleRateHz,
				GF_UINT8 maxScaleRange_g, GF_UINT8 maxPackageDataLength) {
				GF_LOGD("getAccelerateCap: retcode=%s, maxSampleRate=%u, maxScaleRange=%u, maxPkgLength=%u",
					ResponseResultToString(res).c_str(), maxSampleRateHz, maxScaleRange_g, maxPackageDataLength);
			});
			ds->getGyroscopeCap([](ResponseResult res, GF_UINT16 maxSampleRateHz,
				GF_UINT16 maxScaleRange_dps, GF_UINT8 maxPackageDataLength) {
				GF_LOGD("getGyroscopeCap: retcode=%s, maxSampleRate=%u, maxScaleRange=%u, maxPkgLength=%u",
					ResponseResultToString(res).c_str(), maxSampleRateHz, maxScaleRange_dps, maxPackageDataLength);
			});
			ds->getMagnetometerCap([](ResponseResult res, GF_UINT16 maxSampleRateHz,
				GF_UINT16 maxScaleRange_uT, GF_UINT8 maxPackageDataLength) {
				GF_LOGD("getMagnetometerCap: retcode=%s, maxSampleRate=%u, maxScaleRange=%u, maxPkgLength=%u",
					ResponseResultToString(res).c_str(), maxSampleRateHz, maxScaleRange_uT, maxPackageDataLength);
			});
			ds->getEulerangleCap([](ResponseResult res, GF_UINT16 maxSampleRateHz) {
				GF_LOGD("getEulerangleCap: retcode=%s, maxSampleRate=%u", ResponseResultToString(res).c_str(), maxSampleRateHz);
			});
			ds->getQuaternionCap([](ResponseResult res, GF_UINT16 maxSampleRateHz) {
				GF_LOGD("getQuaternionCap: retcode=%s, maxSampleRate=%u", ResponseResultToString(res).c_str(), maxSampleRateHz);
			});
			ds->getRotationMatrixCap([](ResponseResult res, GF_UINT16 maxSampleRateHz) {
				GF_LOGD("getRotationMatrixCap: retcode=%s, maxSampleRate=%u", ResponseResultToString(res).c_str(), maxSampleRateHz);
			});
			ds->getGestureCap([](ResponseResult res, GF_SIZE number, const Gesture supportedGestures[]) {
				GF_LOGD("getGestureCap: retcode=%s, supported gesture number=%u", ResponseResultToString(res).c_str(), number);
				for (GF_SIZE i = 0; i < number; i++)
					GF_LOGD("gesture[%u]: %s", i, gestureToString(supportedGestures[i]).c_str());
			});
			ds->getEMGRawDataCap([](ResponseResult res, GF_UINT16 maxSampleRateHz,
				DeviceSetting::EMGRowDataChannels supportedChannels, GF_UINT8 maxPackageDataLength) {
				GF_LOGD("getEMGRawDataCap: retcode=%s, maxSampleRate=%u, channelsCombine=0x%4.4X, maxPkgLength=%u",
					ResponseResultToString(res).c_str(), maxSampleRateHz, supportedChannels, maxPackageDataLength);
			});
			ds->getMouseDataCap([](ResponseResult res, GF_UINT16 maxSampleRateHz) {
				GF_LOGD("getMouseDataCap: retcode=%s, maxSampleRate=%u", ResponseResultToString(res).c_str(), maxSampleRateHz);
			});
			ds->getJoystickDataCap([](ResponseResult res, GF_UINT16 maxSampleRateHz) {
				GF_LOGD("getJoystickDataCap: retcode=%s, maxSampleRate=%u", ResponseResultToString(res).c_str(), maxSampleRateHz);
			});
			ds->getDeviceStatusCap([](ResponseResult res, DeviceSetting::DeviceStatusFlags flags) {
				GF_LOGD("getDeviceStatusCap: retcode=%s, DeviceStatusFlags=%u", ResponseResultToString(res).c_str(), flags);
			});
			GF_UINT8 data[100];
			ds->sendTrainingModelData(100, data, [](ResponseResult res, GF_UINT32 percentage) {
				GF_LOGD("sendTrainingModelData: retcode=%s, percentage=%u", ResponseResultToString(res).c_str(), percentage);
			});
		}
		break;
	}
	case 'b':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected != itor->getConnectionStatus())
				continue;
			auto ds = itor->getDeviceSetting();
			if (nullptr == ds)
				continue;

			// set caps
			ds->setAccelerateConfig(100, 14, 12 * 10, [](ResponseResult res) {
				GF_LOGD("setAccelerateConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setGyroscopeConfig(1000, 2000, 12 * 10, [](ResponseResult res) {
				GF_LOGD("setGyroscopeConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setMagnetometerConfig(1000, 4800, 12 * 10, [](ResponseResult res) {
				GF_LOGD("setMagnetometerConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setEulerangleConfig(10, [](ResponseResult res) {
				GF_LOGD("setEulerangleConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setQuaternionConfig(10, [](ResponseResult res) {
				GF_LOGD("setQuaternionConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setRotationMatrixConfig(10, [](ResponseResult res) {
				GF_LOGD("setRotationMatrixConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			Gesture gests[] = {
				Gesture::Relax,
				Gesture::Shoot
			};
			GF_UINT32 numGestures = sizeof(gests) / sizeof(Gesture);
			ds->setGestureConfig(numGestures, gests, [](ResponseResult res) {
				GF_LOGD("setGestureConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setEMGRawDataConfig(10, (DeviceSetting::EMGRowDataChannels)0xFF, 128, [](ResponseResult res) {
				GF_LOGD("setEMGRawDataConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setMouseDataConfig(10, [](ResponseResult res) {
				GF_LOGD("setMouseDataConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setJoystickDataConfig(10, [](ResponseResult res) {
				GF_LOGD("setJoystickDataConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
			ds->setDeviceStatusConfig((DeviceSetting::DeviceStatusFlags) (
				DeviceSetting::DeviceStatusFlags::DSF_NONE
				| DeviceSetting::DeviceStatusFlags::DSF_RECENTER
				//| DeviceSetting::DeviceStatusFlags::DSF_USBSTATUS
				//| DeviceSetting::DeviceStatusFlags::DSF_MOTIONLESS
				),
				[](ResponseResult res) {
				GF_LOGD("setDeviceStatusConfig: retcode=%s", ResponseResultToString(res).c_str());
			});
		}
		break;
	}
	case 'n':
	{
		static bool enabled = false;
		bool exec = false;
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				auto ds = itor->getDeviceSetting();
				if (nullptr != ds)
				{
					DeviceSetting::DataNotifFlags flags;
					if (enabled)
					{
						flags = DeviceSetting::DNF_OFF;
					}
					else
					{
						flags = (DeviceSetting::DataNotifFlags)
							(DeviceSetting::DNF_OFF
								| DeviceSetting::DNF_ACCELERATE
								| DeviceSetting::DNF_GYROSCOPE
								| DeviceSetting::DNF_MAGNETOMETER
								//| DeviceSetting::DNF_EULERANGLE
								//| DeviceSetting::DNF_QUATERNION
								//| DeviceSetting::DNF_ROTATIONMATRIX
								| DeviceSetting::DNF_EMG_GESTURE
								//| DeviceSetting::DNF_EMG_RAW
								//| DeviceSetting::DNF_HID_MOUSE
								//| DeviceSetting::DNF_HID_JOYSTICK
								| DeviceSetting::DNF_DEVICE_STATUS
								);
					}
					ds->setDataNotifSwitch(flags, [](ResponseResult result) {
						GF_LOGD("setDataNotifSwitch: %u", result);
					});
					exec = true;
				}
			}
		}
		if (exec)
			enabled = !enabled;
		break;
	}
	case 'o':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				auto ds = itor->getDeviceSetting();
				if (nullptr != ds)
				{
					char* oadfilename = "OAD_test.bin";
#ifdef WIN32
					FILE* pf = nullptr;
					fopen_s(&pf, oadfilename, "rb");
#else
					FILE* pf = fopen(oadfilename, "rb");
#endif
					if (nullptr == pf)
					{
						GF_LOGE("Failed to open OAD bin file: %s", oadfilename);
						break;
					}
					auto ret = ds->oadUpgrade(pf, [](ResponseResult res, GF_UINT32 percentage) {
						GF_LOGD("OAD Progress: resultcode=%s, %u%%", ResponseResultToString(res).c_str(), percentage);
					});
					GF_LOGD("OAD result: ret = %u", ret);
					//fclose(pf);
				}
			}
		}
		break;
	}
	case 'p':
	{
		static bool bPackageIdOn = true;
		bool exec = false;
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				auto ds = itor->getDeviceSetting();
				if (nullptr != ds)
				{
					ds->packageIdControl(bPackageIdOn ?
						DeviceSetting::PackageControlType::Enable : DeviceSetting::PackageControlType::Disable,
#if 0
						nullptr
#else
						[itor](ResponseResult res) {
						GF_LOGD("packageIdControl result: %s, %s", utils::tostring(itor->getName()).c_str(), ResponseResultToString(res).c_str());
						//GF_LOGD("packageIdControl result: %u", res);
					}
#endif
					);
					exec = true;
				}
			}
		}if (exec)
			bPackageIdOn = !bPackageIdOn;
		break;
	}
	case 'x':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				auto ds = itor->getDeviceSetting();
				if (nullptr != ds)
				{
					ds->powerOff();
				}
			}
		}
		break;
	}
	case 'y':
	{
		GF_LOGD("WARNING: After switching to OAD mode, only reflush fw image can return to normal mode.");
		GF_LOGD("Press ENTER key to terminate. Press y+ENTER key to continue.");
		char key;
		cin >> key;
		if (key != 'y')
			break;
		GF_UINT32 i = 0;
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
				++i;
		}
		if (i > 1)
		{
			GF_LOGD("Please only connect to one device.");
			break;
		}
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				auto ds = itor->getDeviceSetting();
				if (nullptr != ds)
				{
					ds->swithToOAD();
				}
			}
		}
		break;
	}
	case 'z':
	{
		for (auto& itor : listDev)
		{
			if (DeviceConnectionStatus::Connected == itor->getConnectionStatus())
			{
				auto ds = itor->getDeviceSetting();
				if (nullptr != ds)
				{
					ds->systemReset();
				}
			}
		}
		break;
	}
	case 'q':
	case 'Q':
		bExiting = true;
		break;
	default:;
		GF_LOGW("Invalid command %s.", cmd.c_str());
	}
}



int _tmain()
{
	printHelp();
	gfsPtr<Hub> pHub = HubManager::getHubInstance(_T("TestApp"));

	if (nullptr == pHub) {
		GF_LOGE("failed to get hub.");
		return 0;
	}

	pHub->setWorkMode(WorkMode::Freerun);
	GF_LOGI("Hub work mode is %d now.", static_cast<GF_INT>(pHub->getWorkMode()));
	GF_LOGI("Main thread id is %s.\n", utils::threadIdToString(this_thread::get_id()).c_str());

	listener = make_shared<HubListenerImp>();
	pHub->registerListener(listener);
	auto another = listener;
	pHub->registerListener(another);
	gfwPtr<HubListenerImp> wp = static_pointer_cast<HubListenerImp>(another);
	pHub->registerListener(wp);
	pHub->registerListener(make_shared<HubListenerImp>());
	//listener = nullptr;

	if (GF_RET_CODE::GF_SUCCESS != pHub->init())
	{
		GF_LOGE("failed to init hub.");
		return 0;
	}
	GF_LOGI("hub state is: %u", static_cast<GF_UINT>(pHub->getState()));
	GF_LOGI("hub string is: %s", utils::tostring(pHub->getDescString()).c_str());


	if (SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlhandler, TRUE))
	{
		do {
			string cmd;
			cin >> cmd;
			handleCmd(pHub, cmd);
		} while (!bExiting);
	}
	else
	{
		GF_LOGE("Error: could not set control handler.");
	}

	listDev.clear();
	pHub->unRegisterListener(listener);
	pHub->deinit();
	GF_LOGD("deinit done here.");

	SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlhandler, FALSE);
	return 0;
}


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
#include "BLEDataProfile4.h"
#include "DeviceSettingDataProfile4.h"

using namespace gf;

BLEDataProfile4::BLEDataProfile4(gfwPtr<BLEDevice> device)
	: DeviceProfile(device)
{
	for (auto i : mPackageIds)
		i = INVALID_PACKAGE_ID;
}

BLEDataProfile4::~BLEDataProfile4()
{
}

#define ATTRIB_HANDLE_GEVENT_V4 (0x22)

void BLEDataProfile4::onCharNotify(ProfileCharType type, GF_UINT8 length, GF_PUINT8 data){
	if (type == ProfileCharType::PROF_DATA_CMD)
	{
		this->onResponse(length, data);
	}
	else if (type == ProfileCharType::PROF_DATA_NTF)
	{
		this->onData(length, data);
	}
	else
	{
		GF_LOGE("Unknown profile char type: %d\n", type);
	}
}

void BLEDataProfile4::onData(GF_UINT8 length, GF_PUINT8 data)
{
	//GF_LOGD("%s", __FUNCTION__);
	gfsPtr<BLEDevice> device = mDevice.lock();
	if (nullptr == device)
		return;
	if (length <= 1)
		return;
	GF_UINT32 offset = 0;

	GF_UINT16 attrib_handle = data[offset] | (((GF_UINT16)data[offset + 1]) << 8);
	if (ATTRIB_HANDLE_GEVENT_V4 != attrib_handle)
		return;
	offset += 2;

	BLEDevice& ref = *device.get();
	GF_UINT8 dataHeader = data[offset++];
	GF_UINT8 dataType = dataHeader & EVENT_MASK;
	GF_UINT8 packageIdFlag = (dataHeader >> 7) & PCKID_FLAG_MASK;

	if (dataType >= DT_MAX)
	{
		GF_LOGE("Unknown data type: 0x%2.2X", dataType);
		return;
	}

	GF_UINT16 currPackageId = INVALID_PACKAGE_ID;
	if (packageIdFlag != 0)
	{
		currPackageId = data[offset++];

		lock_guard<mutex> lock(device->getLockable());
		auto& lastPackageId = mPackageIds[dataType];
		if (lastPackageId == INVALID_PACKAGE_ID)
		{
			lastPackageId = currPackageId;
		}
		else
		{
			++lastPackageId;
			if (lastPackageId > 0xFF)
				lastPackageId = 0;
			if (lastPackageId != currPackageId)
			{
				GF_LOGE("%s:%s: package id error. [%u] expect %u, but %u, gap %u", __FUNCTION__,
					utils::tostring(ref.getName()).c_str(), dataType,
					lastPackageId, currPackageId, (GF_UINT8)(currPackageId - lastPackageId));
				lastPackageId = currPackageId;
			}
		}
	}

	GF_PUINT8 payload = data + offset;
	GF_UINT32 lenPayload = length - offset;

	switch (dataType)
	{
	case DT_ACCELERATE:
		onAccelerateData(ref, lenPayload, payload); break;
	case DT_GYROSCOPE:
		onGyroscopeData(ref, lenPayload, payload); break;
	case DT_MAGNETOMETER:
		onMagnetometerData(ref, lenPayload, payload); break;
	case DT_EULERANGLE:
		onEulerAngleData(ref, lenPayload, payload); break;
	case DT_QUATERNION:
		onQuaternionData(ref, lenPayload, payload); break;
	case DT_ROTATIONMATRIX:
		onRotationMatrixData(ref, lenPayload, payload); break;
	case DT_GESTURE:
		onGestureData(ref, lenPayload, payload); break;
	case DT_EMGRAW:
		onEmgRawData(ref, lenPayload, payload); break;
	case DT_HIDMOUSE:
		onHIDMouseData(ref, lenPayload, payload); break;
	case DT_HIDJOYSTICK:
		onHIDJoystickData(ref, lenPayload, payload); break;
	case DT_DEVICESTATUS:
		onDeviceStatusData(ref, lenPayload, payload); break;
	default:
		;
	}
}

void BLEDataProfile4::onResponse(GF_UINT8 length, GF_PUINT8 data)
{
	if (nullptr != mDevSetting)
		mDevSetting->onResponse(length, data);
}

void BLEDataProfile4::onDeviceStatus(DeviceConnectionStatus oldStatus, DeviceConnectionStatus newStatus)
{
}

gfsPtr<DeviceSetting> BLEDataProfile4::getDeviceSetting()
{
	GF_LOGD("%s", __FUNCTION__);
	gfsPtr<BLEDevice> device = mDevice.lock();
	if (nullptr == device)
	{
		mDevSetting.reset();
		return gfsPtr<DeviceSetting>();
	}

	if (nullptr == mDevSetting)
	{
		mDevSetting = make_shared<DeviceSettingDataProfile4>(device);
	}

	return mDevSetting;
}

void BLEDataProfile4::sendExtendedData(BLEDevice& device, DeviceDataType dataType, GF_UINT8 length, GF_PUINT8 data)
{
	// TODO: frequently memory allocation causes memory fragmentation?
	gfsPtr<const vector<GF_UINT8>> buffer = make_shared<const vector<GF_UINT8>>(data, data+length);
	device.getHub().notifyExtendedData(device, dataType, buffer);
}

void BLEDataProfile4::onAccelerateData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_ACCELERATE, length, data);
}

void BLEDataProfile4::onGyroscopeData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_GYROSCOPE, length, data);
}

void BLEDataProfile4::onMagnetometerData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_MAGNETOMETER, length, data);
}

void BLEDataProfile4::onEulerAngleData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_EULERANGLE, length, data);
}

void BLEDataProfile4::onQuaternionData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	if (length < 16)
	{
		GF_LOGD("%s, length: %u, data insufficient.", __FUNCTION__, length);
		return;
	}
	GF_FLOAT f[4];
	// prevent align issue
	memcpy(f, data, sizeof(f));
	Quaternion q(f[0], f[1], f[2], f[3]);
	// GF_LOGD("Device: %s, Quaternion: %s", utils::tostring(getName()).c_str(), q.toString().c_str());

	device.getHub().notifyOrientationData(device, q);
}

void BLEDataProfile4::onRotationMatrixData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_ROTATIONMATRIX, length, data);
}

void BLEDataProfile4::onGestureData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	GF_LOGD("%s, length: %u", __FUNCTION__, length);
	if (length < 1)
		return;

	Gesture gesture;
	switch (data[0])
	{
	case static_cast<GF_UINT8>(Gesture::Relax) :
		gesture = Gesture::Relax;
		break;
	case static_cast<GF_UINT8>(Gesture::Fist) :
		gesture = Gesture::Fist;
		break;
	case static_cast<GF_UINT8>(Gesture::SpreadFingers) :
		gesture = Gesture::SpreadFingers;
		break;
	case static_cast<GF_UINT8>(Gesture::WaveIn) :
		gesture = Gesture::WaveIn;
		break;
	case static_cast<GF_UINT8>(Gesture::WaveOut) :
		gesture = Gesture::WaveOut;
		break;
	case static_cast<GF_UINT8>(Gesture::Pinch) :
		gesture = Gesture::Pinch;
		break;
	case static_cast<GF_UINT8>(Gesture::Shoot) :
		gesture = Gesture::Shoot;
		break;
	case static_cast<GF_UINT8>(Gesture::Undefined) :
	default:
		gesture = Gesture::Undefined;
	}
	device.getHub().notifyGestureData(device, gesture);
}

void BLEDataProfile4::onEmgRawData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_EMGRAW, length, data);
}

void BLEDataProfile4::onHIDMouseData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_HIDMOUSE, length, data);
}

void BLEDataProfile4::onHIDJoystickData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	// GF_LOGD("%s, length: %u", __FUNCTION__, length);
	sendExtendedData(device, DeviceDataType::DDT_HIDJOYSTICK, length, data);
}

void BLEDataProfile4::onDeviceStatusData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	GF_LOGD("%s, length: %u", __FUNCTION__, length);
	if (length < 1)
		return;

	GF_UINT8 status = data[0];
	switch (status)
	{
	case DSE_RECENTER:
		device.getHub().notifyDeviceStatusChanged(device, DeviceStatus::ReCenter);
		break;
	case DSC_USBPLUGGED:
		device.getHub().notifyDeviceStatusChanged(device, DeviceStatus::UsbPlugged);
		break;
	case DSC_USBPULLED:
		device.getHub().notifyDeviceStatusChanged(device, DeviceStatus::UsbPulled);
		break;
	case DSC_MOTIONLESS:
		device.getHub().notifyDeviceStatusChanged(device, DeviceStatus::Motionless);
		break;
	}
}

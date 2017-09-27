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
/*!
* \file DeviceProfile.h
* \brief The abstract of device profile
*
* \version 0.1
* \date 2017.8.4
*/
#pragma once

#include "DeviceProfile.h"

namespace gf
{
	class DeviceSettingDataProfile4;

	class BLEDataProfile4 : public DeviceProfile
	{
		enum DataType : GF_UINT32 {
			DT_INVALID = 0,
			DT_ACCELERATE,
			DT_GYROSCOPE,
			DT_MAGNETOMETER,
			DT_EULERANGLE,
			DT_QUATERNION,
			DT_ROTATIONMATRIX,
			DT_GESTURE,
			DT_EMGRAW,
			DT_HIDMOUSE,
			DT_HIDJOYSTICK,
			DT_DEVICESTATUS,
			DT_MAX
		};
		enum DeviceStatusEvent : GF_UINT8 {
			DSE_INVALID = 0x00,
			DSE_RECENTER = 0x01,
			DSC_USBPLUGGED = 0x02,
			DSC_USBPULLED = 0x04,
			DSC_MOTIONLESS = 0x08,
			DSE_MAX
		};
	public:
		BLEDataProfile4(gfwPtr<BLEDevice> device);
		~BLEDataProfile4();

	protected:
		virtual void onCharNotify(ProfileCharType type, GF_UINT8 length, GF_PUINT8 data) override;
		virtual void onDeviceStatus(DeviceConnectionStatus oldStatus, DeviceConnectionStatus newStatus) override;
		virtual gfsPtr<DeviceSetting> getDeviceSetting() override;

	private:
		void onData(GF_UINT8 length, GF_PUINT8 data);
		void onResponse(GF_UINT8 length, GF_PUINT8 data);
		void onAccelerateData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onGyroscopeData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onMagnetometerData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onEulerAngleData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onQuaternionData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onRotationMatrixData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onGestureData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onEmgRawData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onHIDMouseData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onHIDJoystickData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onDeviceStatusData(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);

		void sendExtendedData(BLEDevice& device, DeviceDataType dataType, GF_UINT8 length, GF_PUINT8 data);

	private:
		gfsPtr<DeviceSettingDataProfile4> mDevSetting;
		GF_UINT16 mPackageIds[DT_MAX];
	};

}


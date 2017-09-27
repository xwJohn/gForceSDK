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

#include "DeviceSetting.h"
#include "Utils.h"
#include <map>

namespace gf
{

	class BLEDevice;

	const GF_UINT32 MAX_COMMAND_TIMEOUT = 5000; // ms

	class DeviceSettingHandle : public DeviceSetting
	{
	public:
		DeviceSettingHandle(gfwPtr<BLEDevice> device);
		virtual ~DeviceSettingHandle();
		void onResponse(GF_UINT8 length, GF_PUINT8 data);

	public:
		virtual GF_RET_CODE getProtocolVer(function<void(ResponseResult res, tstring version)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getFeatureMap(function<void(ResponseResult res, GF_UINT32 featureMap)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getDeviceName(function<void(ResponseResult res, tstring name)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getModelNumber(function<void(ResponseResult res, tstring modelNumber)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getSerialNumber(function<void(ResponseResult res, tstring serialNumber)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getHWRevision(function<void(ResponseResult res, tstring version)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getFWRevision(function<void(ResponseResult res, tstring version)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getManufacturerName(function<void(ResponseResult res, tstring name)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getBootloaderVer(function<void(ResponseResult res, tstring version)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE powerOff() override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE swithToOAD() override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE systemReset() override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE switchService(DeviceService service) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE oadUpgrade(FILE* file, function<void(ResponseResult res, GF_UINT32 percentage)> progress) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE sendTrainingModelData(GF_UINT32 length, GF_UINT8 data[], function<void(ResponseResult res, GF_UINT32 percentage)> progress) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setDataNotifSwitch(DataNotifFlags flags, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getBatteryLevel(function<void(ResponseResult res, GF_UINT32 percentage)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getTemperature(function<void(ResponseResult res, GF_UINT32 temperature)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setLogLevel(SWOLogLevel level, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setLogModuleEnabled(SWOModule modules, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE printKernelMsg(KernelMsgType type, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE vibrateControl(VibrateControlType type, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE ledControlTest(LedControlTestType type, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE packageIdControl(PackageControlType type, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getAccelerateCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
			GF_UINT8 maxScaleRange_g, GF_UINT8 maxPackageDataLength)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setAccelerateConfig(GF_UINT16 sampleRateHz,
			GF_UINT8 fullScaleRange_g, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getGyroscopeCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
			GF_UINT16 maxScaleRange_dps, GF_UINT8 maxPackageDataLength)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setGyroscopeConfig(GF_UINT16 sampleRateHz,
			GF_UINT16 fullScaleRange_dps, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getMagnetometerCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
			GF_UINT16 maxScaleRange_uT, GF_UINT8 maxPackageDataLength)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setMagnetometerConfig(GF_UINT16 sampleRateHz,
			GF_UINT16 fullScaleRange_uT, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getEulerangleCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setEulerangleConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getQuaternionCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setQuaternionConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getRotationMatrixCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setRotationMatrixConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getGestureCap(function<void(ResponseResult res, GF_SIZE number, const Gesture supportedGestures[])> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setGestureConfig(GF_UINT32 number, Gesture interestingGesture[], function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getEMGRawDataCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
			EMGRowDataChannels supportedChannels, GF_UINT8 maxPackageDataLength)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setEMGRawDataConfig(GF_UINT16 sampleRateHz,
			EMGRowDataChannels interestingChannels, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getMouseDataCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setMouseDataConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getJoystickDataCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setJoystickDataConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE getDeviceStatusCap(function<void(ResponseResult res, DeviceStatusFlags flags)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }
		virtual GF_RET_CODE setDeviceStatusConfig(DeviceStatusFlags flags, function<void(ResponseResult res)> cb) override { return GF_RET_CODE::GF_ERROR_NOT_SUPPORT; }

	protected:
		gfwPtr<BLEDevice> mDevice;

	protected:
		//////////////////////////////////////////////////////////////
		// Message pool to process command and it's response
		mutex mMutex;
		SimpleTimer mTimer;
		struct CommandHandler
		{
			SimpleTimer::TimePoint t;
			///////////////
			// store any object of type function<class xxx> as void*,
			// covert it back to function<class xxx> pointer before call it.
			gfsPtr<void> cb;
		};
		map<GF_UINT8, CommandHandler> mExecutingList;
		GF_RET_CODE sendCommand(ProfileCharType type, GF_UINT8 dataLen, GF_PUINT8 commandData, bool hasResponse = false, gfsPtr<void> cb = nullptr);

		void updateTimer();
		void onTimer();
		virtual void dispatchResponse(GF_UINT8 command, GF_UINT8 retval, GF_UINT8 length,
			GF_PUINT8 data, gfsPtr<void> cb, bool timeout = false) = 0;
	};

}

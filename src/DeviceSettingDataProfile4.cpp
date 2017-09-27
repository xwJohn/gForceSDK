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
#include "Utils.h"
#include "BLEDevice.h"
#include "DeviceSettingDataProfile4.h"

using namespace gf;

#define MTU_DEFAULT (23)
#define MTU_HEAD_LENGTH (4+1) // +1 to consider package id takes 1 byte when enables
#define MTU_ROTATIONMATRIX (36)

////////////////////////////////////////////////////////
// API
#define DO_SEND_COMMAND_AND_CALLBACK_NO_RETURN(retval, length, buf)	\
	do {	\
		gfsPtr<decltype(cb)> pcb = make_shared<decltype(cb)>(cb);	\
		gfsPtr<void> pvoid = static_pointer_cast<void>(pcb);	\
		retval = sendCommand(ProfileCharType::PROF_DATA_CMD, (length), (buf), true, pvoid);	\
	} while (false)

#define DO_SEND_COMMAND_AND_CALLBACK(length, buf)	\
	do {	\
		GF_RET_CODE retval;	\
		DO_SEND_COMMAND_AND_CALLBACK_NO_RETURN(retval, length, buf);	\
		return retval;	\
	} while (false)

#define DO_SEND_GET_COMMAND(cmd)	\
	do {	\
		GF_UINT8 data = cmd;	\
		DO_SEND_COMMAND_AND_CALLBACK(1, &data);	\
	} while (false)

////////////////////////////////////////////////////////
// callback
#define ON_RESPONSE_RESPRESULT_PARAM(callback_type, ...) \
	do {	\
		GF_LOGD("%s, retval = %u", __FUNCTION__, retval);	\
		gfsPtr<function<callback_type>> callback = static_pointer_cast<function<callback_type>>(cb);	\
		if (callback && *callback.get())	\
			(*callback.get())(responseConvert(retval), ##__VA_ARGS__);	\
	} while (false)

#define ON_RESPONSE_RESPRESULT() ON_RESPONSE_RESPRESULT_PARAM(void(ResponseResult))

#define ON_RESPONSE_PARSING(do_parse, data_min_length, callback_type, ...) \
	do {	\
		if (retval == RC_SUCCESS)	\
		{	\
			if (length >= (data_min_length))	\
				(do_parse)(length, data);	\
			else	\
				retval = RC_FAILED;	\
		}	\
		gfsPtr<function<callback_type>> callback = static_pointer_cast<function<callback_type>>(cb);	\
		if (callback && *callback.get())	\
			(*callback.get())(responseConvert(retval), ##__VA_ARGS__);	\
	} while (false)

#define ON_RESPONSE_PARSING_2BYTES(represent_type) \
	do {	\
		GF_UINT16 v = 0;	\
		auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {	\
			v = data[0] | ((GF_UINT16)data[1] << 8);	\
		};	\
		ON_RESPONSE_PARSING(f, 2, void(ResponseResult, represent_type), static_cast<represent_type>(v));	\
	} while(false)

#define ON_RESPONSE_PARSING_4BYTES(represent_type) \
	do {	\
		GF_UINT32 v = 0;	\
		auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {	\
			v = (GF_UINT32)(data[0])	\
				| ((GF_UINT32)(data[1]) << 8)	\
				| ((GF_UINT32)(data[2]) << 16)	\
				| ((GF_UINT32)(data[3]) << 24);	\
		};	\
		ON_RESPONSE_PARSING(f, 4, void(ResponseResult, represent_type), static_cast<represent_type>(v));	\
	} while(false)

#define ON_RESPONSE_PARSING_4BYTES_WITH_211() \
	do {	\
		GF_UINT16 v1 = 0;	\
		GF_UINT8 v2 = 0;	\
		GF_UINT8 v3 = 0;	\
		auto f = [&v1, &v2, &v3](GF_UINT8 length, GF_PUINT8 data) {	\
		v1 = data[0] | ((GF_UINT16)data[1] << 8);	\
		v2 = data[2];	\
		v3 = data[3];	\
		};	\
		ON_RESPONSE_PARSING(f, 4, void(ResponseResult, GF_UINT16, GF_UINT8, GF_UINT8), v1, v2, v3);	\
	} while(false)

#define ON_RESPONSE_PARSING_5BYTES_WITH_221() \
	do {	\
		GF_UINT16 v1 = 0;	\
		GF_UINT16 v2 = 0;	\
		GF_UINT8 v3 = 0;	\
		auto f = [&v1, &v2, &v3](GF_UINT8 length, GF_PUINT8 data) {	\
		v1 = data[0] | ((GF_UINT16)data[1] << 8);	\
		v2 = data[2] | ((GF_UINT16)data[3] << 8);	\
		v3 = data[4];	\
		};	\
		ON_RESPONSE_PARSING(f, 5, void(ResponseResult, GF_UINT16, GF_UINT16, GF_UINT8), v1, v2, v3);	\
	} while(false)

#define ON_RESPONSE_PARSING_UINT16() ON_RESPONSE_PARSING_2BYTES(GF_UINT16)
#define ON_RESPONSE_PARSING_UINT32() ON_RESPONSE_PARSING_4BYTES(GF_UINT32)


enum CommandType {
	CMD_GET_PROTOCOL_VERSION = 0x00,
	CMD_GET_FEATURE_MAP = 0x01,
	CMD_GET_DEVICE_NAME = 0x02,
	CMD_GET_MODEL_NUMBER = 0x03,
	CMD_GET_SERIAL_NUMBER = 0x04,
	CMD_GET_HW_REVISION = 0x05,
	CMD_GET_FW_REVISION = 0x06,
	CMD_GET_MANUFACTURER_NAME = 0x07,
	CMD_GET_BOOTLOADER_VERSION = 0x0A,

	CMD_GET_BATTERY_LEVEL = 0x08,
	CMD_GET_TEMPERATURE = 0x09,

	CMD_POWEROFF = 0x1D,
	CMD_SWITCH_TO_OAD = 0x1E,
	CMD_SYSTEM_RESET = 0x1F,
	CMD_SWITCH_SERVICE = 0x20,

	CMD_SET_SWO_LOG_LEVEL = 0x21,
	CMD_SET_SWO_LOG_MODULE = 0x22,
	CMD_PRINT_KERNEL_MSG = 0x23,
	CMD_MOTOR_CONTROL = 0x24,
	CMD_LED_CONTROL_TEST = 0x25,
	CMD_PACKAGE_ID_CONTROL = 0x26,
	CMD_SEND_TRAINING_PACKAGE = 0x27,

	CMD_GET_ACCELERATE_CAP = 0x30,
	CMD_SET_ACCELERATE_CONFIG = 0x31,

	CMD_GET_GYROSCOPE_CAP = 0x32,
	CMD_SET_GYROSCOPE_CONFIG = 0x33,

	CMD_GET_MAGNETOMETER_CAP = 0x34,
	CMD_SET_MAGNETOMETER_CONFIG = 0x35,

	CMD_GET_EULER_ANGLE_CAP = 0x36,
	CMD_SET_EULER_ANGLE_CONFIG = 0x37,

	CMD_GET_QUATERNION_CAP = 0x38,
	CMD_SET_QUATERNION_CONFIG = 0x39,

	CMD_GET_ROTATION_MATRIX_CAP = 0x3A,
	CMD_SET_ROTATION_MATRIX_CONFIG = 0x3B,

	CMD_GET_GESTURE_CAP = 0x3C,
	CMD_SET_GESTURE_CONFIG = 0x3D,

	CMD_GET_EMG_RAWDATA_CAP = 0x3E,
	CMD_SET_EMG_RAWDATA_CONFIG = 0x3F,

	CMD_GET_MOUSE_DATA_CAP = 0x40,
	CMD_SET_MOUSE_DATA_CONFIG = 0x41,

	CMD_GET_JOYSTICK_DATA_CAP = 0x42,
	CMD_SET_JOYSTICK_DATA_CONFIG = 0x43,

	CMD_GET_DEVICE_STATUS_CAP = 0x44,
	CMD_SET_DEVICE_STATUS_CONFIG = 0x45,

	CMD_SET_DATA_NOTIF_SWITCH = 0x4F,
};

enum ResponseCode {
	RC_SUCCESS = 0x00,
	RC_NOT_SUPPORT = 0x01,
	RC_BAD_PARAM = 0x02,
	RC_FAILED = 0x03,
	RC_TIMEOUT = 0xFE,
	RC_MAX = 0xFF,
};

inline ResponseResult responseConvert(ResponseCode code)
{
	switch (code)
	{
	case RC_SUCCESS:
		return ResponseResult::RREST_SUCCESS; break;
	case RC_NOT_SUPPORT:
		return ResponseResult::RREST_NOT_SUPPORT; break;
	case RC_BAD_PARAM:
		return ResponseResult::RREST_BAD_PARAM; break;
	case RC_TIMEOUT:
		return ResponseResult::RREST_TIMEOUT; break;
	case RC_FAILED:
	default:
		return ResponseResult::RREST_FAILED; break;
	}
}
inline ResponseResult responseConvert(GF_UINT32 code)
{
	return responseConvert(static_cast<ResponseCode>(code));
}

struct Dispather {
	using OnRespFunc = void (DeviceSettingDataProfile4::*)(GF_UINT8, GF_UINT8, GF_PUINT8, gfsPtr<void>);
	GF_UINT32 cmd;
	OnRespFunc func;
};

DeviceSettingDataProfile4::DeviceSettingDataProfile4(gfwPtr<BLEDevice> device)
	: DeviceSettingHandle(device)
{
}

DeviceSettingDataProfile4::~DeviceSettingDataProfile4()
{
}

void DeviceSettingDataProfile4::dispatchResponse(GF_UINT8 command, GF_UINT8 retval, GF_UINT8 length,
	GF_PUINT8 data, gfsPtr<void> cb, bool timeout)
{
	const Dispather DispatchTable[] =
	{
		{ CMD_GET_PROTOCOL_VERSION, &DeviceSettingDataProfile4::onProtocolVer },
		{ CMD_GET_FEATURE_MAP, &DeviceSettingDataProfile4::onFeatureMap },
		{ CMD_GET_DEVICE_NAME, &DeviceSettingDataProfile4::onDeviceName },
		{ CMD_GET_MODEL_NUMBER, &DeviceSettingDataProfile4::onModelNumber },
		{ CMD_GET_SERIAL_NUMBER, &DeviceSettingDataProfile4::onSerialNumber },
		{ CMD_GET_HW_REVISION, &DeviceSettingDataProfile4::onHWRevision },
		{ CMD_GET_FW_REVISION, &DeviceSettingDataProfile4::onFWRevision },
		{ CMD_GET_MANUFACTURER_NAME, &DeviceSettingDataProfile4::onManufacturerName },
		{ CMD_GET_BOOTLOADER_VERSION, &DeviceSettingDataProfile4::onBootloaderVer },
		{ CMD_SEND_TRAINING_PACKAGE, &DeviceSettingDataProfile4::onSendTrainingModelData },
		{ CMD_SET_DATA_NOTIF_SWITCH, &DeviceSettingDataProfile4::onSetDataNotifSwitch },
		{ CMD_GET_BATTERY_LEVEL, &DeviceSettingDataProfile4::onGetBatteryLevel },
		{ CMD_GET_TEMPERATURE, &DeviceSettingDataProfile4::onGetTemperature },
		{ CMD_SET_SWO_LOG_LEVEL, &DeviceSettingDataProfile4::onSetLogLevel },
		{ CMD_SET_SWO_LOG_MODULE, &DeviceSettingDataProfile4::onSetLogModuleEnabled },
		{ CMD_PRINT_KERNEL_MSG, &DeviceSettingDataProfile4::onPrintKernelMsg },
		{ CMD_MOTOR_CONTROL, &DeviceSettingDataProfile4::onVibrateControl },
		{ CMD_LED_CONTROL_TEST, &DeviceSettingDataProfile4::onLedControlTest },
		{ CMD_PACKAGE_ID_CONTROL, &DeviceSettingDataProfile4::onPackageIdControl },
		{ CMD_GET_ACCELERATE_CAP, &DeviceSettingDataProfile4::onGetAccelerateCap },
		{ CMD_SET_ACCELERATE_CONFIG, &DeviceSettingDataProfile4::onSetAccelerateConfig },
		{ CMD_GET_GYROSCOPE_CAP, &DeviceSettingDataProfile4::onGetGyroscopeCap },
		{ CMD_SET_GYROSCOPE_CONFIG, &DeviceSettingDataProfile4::onSetGyroscopeConfig },
		{ CMD_GET_MAGNETOMETER_CAP, &DeviceSettingDataProfile4::onGetMagnetometerCap },
		{ CMD_SET_MAGNETOMETER_CONFIG, &DeviceSettingDataProfile4::onSetMagnetometerConfig },
		{ CMD_GET_EULER_ANGLE_CAP, &DeviceSettingDataProfile4::onGetEulerangleCap },
		{ CMD_SET_EULER_ANGLE_CONFIG, &DeviceSettingDataProfile4::onSetEulerangleConfig },
		{ CMD_GET_QUATERNION_CAP, &DeviceSettingDataProfile4::onGetQuaternionCap },
		{ CMD_SET_QUATERNION_CONFIG, &DeviceSettingDataProfile4::onSetQuaternionConfig },
		{ CMD_GET_ROTATION_MATRIX_CAP, &DeviceSettingDataProfile4::onGetRotationMatrixCap },
		{ CMD_SET_ROTATION_MATRIX_CONFIG, &DeviceSettingDataProfile4::onSetRotationMatrixConfig },
		{ CMD_GET_GESTURE_CAP, &DeviceSettingDataProfile4::onGetGestureCap },
		{ CMD_SET_GESTURE_CONFIG, &DeviceSettingDataProfile4::onSetGestureConfig },
		{ CMD_GET_EMG_RAWDATA_CAP, &DeviceSettingDataProfile4::onGetEMGRawDataCap },
		{ CMD_SET_EMG_RAWDATA_CONFIG, &DeviceSettingDataProfile4::onSetEMGRawDataConfig },
		{ CMD_GET_MOUSE_DATA_CAP, &DeviceSettingDataProfile4::onGetMouseDataCap },
		{ CMD_SET_MOUSE_DATA_CONFIG, &DeviceSettingDataProfile4::onSetMouseDataConfig },
		{ CMD_GET_JOYSTICK_DATA_CAP, &DeviceSettingDataProfile4::onGetJoystickDataCap },
		{ CMD_SET_JOYSTICK_DATA_CONFIG, &DeviceSettingDataProfile4::onSetJoystickDataConfig },
		{ CMD_GET_DEVICE_STATUS_CAP, &DeviceSettingDataProfile4::onGetDeviceStatusCap },
		{ CMD_SET_DEVICE_STATUS_CONFIG, &DeviceSettingDataProfile4::onSetDeviceStatusConfig },
	};

	if (timeout)
	{
		retval = RC_TIMEOUT;
	}
	for (auto& i : DispatchTable)
	{
		if (i.cmd == command)
		{
			(this->*(i.func))(retval, length, data, cb);
			break;
		}
	}
}

GF_RET_CODE DeviceSettingDataProfile4::getProtocolVer(function<void(ResponseResult res, tstring version)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mProtocolVer.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mProtocolVer);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_PROTOCOL_VERSION);
}
GF_RET_CODE DeviceSettingDataProfile4::getFeatureMap(function<void(ResponseResult res, GF_UINT32 featureMap)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mFeatureMap != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mFeatureMap);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_FEATURE_MAP);
}
GF_RET_CODE DeviceSettingDataProfile4::getDeviceName(function<void(ResponseResult res, tstring name)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mDeviceName.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mDeviceName);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_DEVICE_NAME);
}
GF_RET_CODE DeviceSettingDataProfile4::getModelNumber(function<void(ResponseResult res, tstring modelNumber)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mModelNumber.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mModelNumber);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_MODEL_NUMBER);
}
GF_RET_CODE DeviceSettingDataProfile4::getSerialNumber(function<void(ResponseResult res, tstring serialNumber)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mSerialNumber.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mSerialNumber);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_SERIAL_NUMBER);
}
GF_RET_CODE DeviceSettingDataProfile4::getHWRevision(function<void(ResponseResult res, tstring version)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mHWRev.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mHWRev);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_HW_REVISION);
}
GF_RET_CODE DeviceSettingDataProfile4::getFWRevision(function<void(ResponseResult res, tstring version)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mFWRev.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mFWRev);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_FW_REVISION);
}
GF_RET_CODE DeviceSettingDataProfile4::getManufacturerName(function<void(ResponseResult res, tstring name)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mManuName.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mManuName);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_MANUFACTURER_NAME);
}
GF_RET_CODE DeviceSettingDataProfile4::getBootloaderVer(function<void(ResponseResult res, tstring version)> cb)
{
	GF_LOGD(__FUNCTION__);
	if (mBootloaderVer.size() != 0)
	{
		if (nullptr != cb)
			cb(responseConvert(RC_SUCCESS), mBootloaderVer);
		return GF_RET_CODE::GF_SUCCESS;
	}
	DO_SEND_GET_COMMAND(CMD_GET_BOOTLOADER_VERSION);
}

GF_RET_CODE DeviceSettingDataProfile4::powerOff()
{
	GF_LOGD(__FUNCTION__);
	GF_UINT8 data = CMD_POWEROFF;
	return sendCommand(ProfileCharType::PROF_DATA_CMD, 1, &data);
}
GF_RET_CODE DeviceSettingDataProfile4::swithToOAD()
{
	GF_LOGD(__FUNCTION__);
	GF_UINT8 data = CMD_SWITCH_TO_OAD;
	return sendCommand(ProfileCharType::PROF_DATA_CMD, 1, &data);
}
GF_RET_CODE DeviceSettingDataProfile4::systemReset()
{
	GF_LOGD(__FUNCTION__);
	GF_UINT8 data = CMD_SYSTEM_RESET;
	return sendCommand(ProfileCharType::PROF_DATA_CMD, 1, &data);
}

GF_RET_CODE DeviceSettingDataProfile4::switchService(DeviceService service)
{
	GF_LOGD("%s: service = %u", __FUNCTION__, service);
	GF_UINT8 data[2];
	data[0] = CMD_SWITCH_SERVICE;
	data[1] = static_cast<GF_UINT8>(service);
	return sendCommand(ProfileCharType::PROF_DATA_CMD, 2, data);
}

GF_RET_CODE DeviceSettingDataProfile4::setDataNotifSwitch(DataNotifFlags flags, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: flags = 0x%8.8X", __FUNCTION__, flags);
	if (flags | DeviceSetting::DNF_ROTATIONMATRIX)
		configMtuSize(MTU_ROTATIONMATRIX);

	const GF_UINT8 length = 5;
	GF_UINT8 data[length];
	data[0] = CMD_SET_DATA_NOTIF_SWITCH;
	data[1] = (GF_UINT8)(flags);
	data[2] = (GF_UINT8)(flags >> 8);
	data[3] = (GF_UINT8)(flags >> 16);
	data[4] = (GF_UINT8)(flags >> 24);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}

///////////////////////////////////////////////
#define TRANING_PACKAGE_DATA_LENGTH (16)
#define INVALID_NEXT_PACKAGE_NUMBER (0xFFFF)
void DeviceSettingDataProfile4::trainingModelOnResponse(ResponseResult respval, GF_UINT16 nextPkgNo)
{
	GF_LOGD("sending model data: respval = %u, nextPkgNo = %u",
		respval, nextPkgNo);
	if (respval != ResponseResult::RREST_SUCCESS ||
		nextPkgNo == 0 || // package number started from 1
		nextPkgNo == INVALID_NEXT_PACKAGE_NUMBER)
	{
		// Failed and stop sending data
		GF_LOGD("sending data step: error. stop.");
		mTrainingModelCallback(ResponseResult::RREST_FAILED, 0);
		resetTrainingModelData();
		return;
	}
	auto ret = trainingModelSendNextPackage(nextPkgNo);
	if (GF_RET_CODE::GF_SUCCESS != ret)
	{
		if (GF_RET_CODE::GF_ERROR_BAD_PARAM == ret)
		{
			// data sending completed, tell client that the resut is good
			mTrainingModelCallback(ResponseResult::RREST_SUCCESS, 100);
		}
		else
		{
			GF_LOGD("sending model data: error in sending: ret = %u, total size = %u,",
				ret, mTraingModelBuffer.size());
			mTrainingModelCallback(ResponseResult::RREST_FAILED, 0);
		}
		resetTrainingModelData();
	}
	else
	{
		// calculate progress percentage
		GF_UINT32 currentStartPos = TRANING_PACKAGE_DATA_LENGTH * (nextPkgNo - 1);
		GF_UINT32 percentage = (GF_UINT32)(currentStartPos * 100 / mTraingModelBuffer.size());
		if (100 <= percentage)
			percentage = 99; // 100 means completion, it should be sent above
		if ((GF_UINT32)-1 == mLastSentPercentage || (mLastSentPercentage + 5) <= percentage)
		{
			mTrainingModelCallback(ResponseResult::RREST_SUCCESS, percentage);
			mLastSentPercentage = percentage;
		}

	}
}
GF_RET_CODE DeviceSettingDataProfile4::trainingModelSendNextPackage(GF_UINT16 nextPkgId)
{
	if (1 > nextPkgId)
		return GF_RET_CODE::GF_ERROR_BAD_PARAM;
	GF_UINT16 pkgNo = nextPkgId - 1;
	if ((GF_UINT32)(TRANING_PACKAGE_DATA_LENGTH * pkgNo) >= mTraingModelBuffer.size())
		return GF_RET_CODE::GF_ERROR_BAD_PARAM;

	GF_UINT32 remainDataLen = (GF_UINT32)(mTraingModelBuffer.size() - (TRANING_PACKAGE_DATA_LENGTH * pkgNo));
	const GF_UINT8 cmdlen = TRANING_PACKAGE_DATA_LENGTH + 3;
	GF_UINT8 cmddata[cmdlen];
	memset(cmddata, 0, sizeof(cmddata));
	cmddata[0] = CMD_SEND_TRAINING_PACKAGE;
	cmddata[1] = (GF_UINT8)nextPkgId;
	cmddata[2] = (GF_UINT8)(nextPkgId >> 8);
	// NOTE that package number is started from 1.
	memcpy(cmddata + 3, mTraingModelBuffer.data() + (TRANING_PACKAGE_DATA_LENGTH * pkgNo),
		min<GF_UINT32>(TRANING_PACKAGE_DATA_LENGTH, remainDataLen));
	auto onResponse = bind(&DeviceSettingDataProfile4::trainingModelOnResponse, this, placeholders::_1, placeholders::_2);
	gfsPtr<function<void(ResponseResult, GF_UINT16)>> pcb = make_shared<function<void(ResponseResult, GF_UINT16)>>(onResponse);
	gfsPtr<void> pvoid = static_pointer_cast<void>(pcb);
	return sendCommand(ProfileCharType::PROF_DATA_CMD, cmdlen, cmddata, true, pvoid);
}
void DeviceSettingDataProfile4::resetTrainingModelData()
{
	mTraingModelBuffer = vector<GF_UINT8>();
	mTrainingModelCallback = nullptr;
	mLastSentPercentage = (GF_UINT32)-1;
}
GF_RET_CODE DeviceSettingDataProfile4::sendTrainingModelData(GF_UINT32 length,
	GF_UINT8 data[], function<void(ResponseResult res, GF_UINT32 percentage)> progress)
{
	if (0 == length || nullptr == data)
	{
		return GF_RET_CODE::GF_ERROR_BAD_PARAM;
	}
	if (!mTraingModelBuffer.empty())
	{
		return GF_RET_CODE::GF_ERROR_DEVICE_BUSY;
	}

	// first backup data buffer
	mTraingModelBuffer = vector<GF_UINT8>(data, data + length);
	mTrainingModelCallback = progress;

	// send first package
	auto ret = trainingModelSendNextPackage(1);
	if (GF_RET_CODE::GF_SUCCESS != ret)
	{
		GF_LOGD("sending data step: error in sending: ret = %u, given total size = %u", ret, length);
		resetTrainingModelData();
	}
	return ret;
}
GF_RET_CODE DeviceSettingDataProfile4::getBatteryLevel(function<void(ResponseResult res, GF_UINT32 percentage)> cb)
{
	GF_LOGD(__FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_BATTERY_LEVEL);
}
// C.1
GF_RET_CODE DeviceSettingDataProfile4::getTemperature(function<void(ResponseResult res, GF_UINT32 temperature)> cb)
{
	GF_LOGD(__FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_TEMPERATURE);
}
// C.3
GF_RET_CODE DeviceSettingDataProfile4::setLogLevel(SWOLogLevel level, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: level = %u", __FUNCTION__, level);
	const GF_UINT8 length = 2;
	GF_UINT8 data[length];
	data[0] = CMD_SET_SWO_LOG_LEVEL;
	data[1] = static_cast<GF_UINT8>(level);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.3
GF_RET_CODE DeviceSettingDataProfile4::setLogModuleEnabled(SWOModule modules, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: modules = %u", __FUNCTION__, modules);
	const GF_UINT8 length = 5;
	GF_UINT8 data[length];
	data[0] = CMD_SET_SWO_LOG_MODULE;
	data[1] = (GF_UINT8)(modules);
	data[2] = (GF_UINT8)(modules >> 8);
	data[3] = (GF_UINT8)(modules >> 16);
	data[4] = (GF_UINT8)(modules >> 24);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.3
GF_RET_CODE DeviceSettingDataProfile4::printKernelMsg(KernelMsgType type, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: type = %u", __FUNCTION__, (GF_UINT32)type);
	const GF_UINT8 length = 2;
	GF_UINT8 data[length];
	data[0] = CMD_PRINT_KERNEL_MSG;
	data[1] = static_cast<GF_UINT8>(type);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.4
GF_RET_CODE DeviceSettingDataProfile4::vibrateControl(VibrateControlType type, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: type = %u", __FUNCTION__, (GF_UINT32)type);
	const GF_UINT8 length = 2;
	GF_UINT8 data[length];
	data[0] = CMD_MOTOR_CONTROL;
	data[1] = static_cast<GF_UINT8>(type);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.5
GF_RET_CODE DeviceSettingDataProfile4::ledControlTest(LedControlTestType type, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: type = %u", __FUNCTION__, (GF_UINT32)type);
	const GF_UINT8 length = 2;
	GF_UINT8 data[length];
	data[0] = CMD_LED_CONTROL_TEST;
	data[1] = static_cast<GF_UINT8>(type);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
GF_RET_CODE DeviceSettingDataProfile4::packageIdControl(PackageControlType type, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: %s", __FUNCTION__, type == PackageControlType::Enable ? "Enabled" : "Disabled");
	const GF_UINT8 length = 2;
	GF_UINT8 data[length];
	data[0] = CMD_PACKAGE_ID_CONTROL;
	data[1] = (type == PackageControlType::Enable ? 0x01 : 0x00);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}

// C.7
GF_RET_CODE DeviceSettingDataProfile4::getAccelerateCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
	GF_UINT8 maxScaleRange_g, GF_UINT8 maxPackageDataLength)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_ACCELERATE_CAP);
}
// C.7
GF_RET_CODE DeviceSettingDataProfile4::setAccelerateConfig(GF_UINT16 sampleRateHz,
	GF_UINT8 fullScaleRange_g, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz), full scale range = %u(g), package data length = %u(bytes)",
		__FUNCTION__, sampleRateHz, fullScaleRange_g, packageDataLength);

	// set mtu size first
	configMtuSize(packageDataLength);

	const GF_UINT8 length = 5;
	GF_UINT8 data[length];
	data[0] = CMD_SET_ACCELERATE_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	data[3] = fullScaleRange_g;
	data[4] = packageDataLength;
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.8
GF_RET_CODE DeviceSettingDataProfile4::getGyroscopeCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
	GF_UINT16 maxScaleRange_dps, GF_UINT8 maxPackageDataLength)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_GYROSCOPE_CAP);
}
// C.8
GF_RET_CODE DeviceSettingDataProfile4::setGyroscopeConfig(GF_UINT16 sampleRateHz,
	GF_UINT16 fullScaleRange_dps, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz), full scale range = %u(g), package data length = %u(bytes)",
		__FUNCTION__, sampleRateHz, fullScaleRange_dps, packageDataLength);

	// set mtu size first
	configMtuSize(packageDataLength);

	const GF_UINT8 length = 6;
	GF_UINT8 data[length];
	data[0] = CMD_SET_GYROSCOPE_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	data[3] = (GF_UINT8)(fullScaleRange_dps);
	data[4] = (GF_UINT8)(fullScaleRange_dps >> 8);
	data[5] = packageDataLength;
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.9
GF_RET_CODE DeviceSettingDataProfile4::getMagnetometerCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
	GF_UINT16 maxScaleRange_uT, GF_UINT8 maxPackageDataLength)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_MAGNETOMETER_CAP);
}
// C.9
GF_RET_CODE DeviceSettingDataProfile4::setMagnetometerConfig(GF_UINT16 sampleRateHz,
	GF_UINT16 fullScaleRange_uT, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz), full scale range = %u(g), package data length = %u(bytes)",
		__FUNCTION__, sampleRateHz, fullScaleRange_uT, packageDataLength);

	// set mtu size first
	configMtuSize(packageDataLength);

	const GF_UINT8 length = 6;
	GF_UINT8 data[length];
	data[0] = CMD_SET_MAGNETOMETER_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	data[3] = (GF_UINT8)(fullScaleRange_uT);
	data[4] = (GF_UINT8)(fullScaleRange_uT >> 8);
	data[5] = packageDataLength;
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.10
GF_RET_CODE DeviceSettingDataProfile4::getEulerangleCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_EULER_ANGLE_CAP);
}
// C.10
GF_RET_CODE DeviceSettingDataProfile4::setEulerangleConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz)", __FUNCTION__, sampleRateHz);
	const GF_UINT8 length = 3;
	GF_UINT8 data[length];
	data[0] = CMD_SET_EULER_ANGLE_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.11
GF_RET_CODE DeviceSettingDataProfile4::getQuaternionCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_QUATERNION_CAP);
}
// C.11
GF_RET_CODE DeviceSettingDataProfile4::setQuaternionConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz)", __FUNCTION__, sampleRateHz);
	const GF_UINT8 length = 3;
	GF_UINT8 data[length];
	data[0] = CMD_SET_QUATERNION_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.12
GF_RET_CODE DeviceSettingDataProfile4::getRotationMatrixCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_ROTATION_MATRIX_CAP);
}
// C.12
GF_RET_CODE DeviceSettingDataProfile4::setRotationMatrixConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz)", __FUNCTION__, sampleRateHz);
	const GF_UINT8 length = 3;
	GF_UINT8 data[length];
	data[0] = CMD_SET_ROTATION_MATRIX_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.13
GF_RET_CODE DeviceSettingDataProfile4::getGestureCap(function<void(ResponseResult res, GF_SIZE number, const Gesture supportedGestures[])> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_GESTURE_CAP);
}
// C.13
enum GestureMask : GF_UINT32
{
	GM_NONE = 0x00000000,
	GM_RELAX = 0x00000001,
	GM_FIST = 0x00000002,
	GM_SPREAD = 0x00000004,
	GM_WAVEIN = 0x00000008,
	GM_WAVEOUT = 0x00000010,
	GM_PINCH = 0x00000020,
	GM_SHOOT = 0x00000040,
};
GF_RET_CODE DeviceSettingDataProfile4::setGestureConfig(GF_UINT32 number, Gesture interestingGesture[], function<void(ResponseResult res)> cb)
{
	GF_UINT32 mask = GM_NONE;
	for (GF_UINT32 i = 0; i < number; i++)
	{
		switch (interestingGesture[i])
		{
		case Gesture::Relax:
			mask |= GM_RELAX; break;
		case Gesture::Fist:
			mask |= GM_FIST; break;
		case Gesture::SpreadFingers:
			mask |= GM_SPREAD; break;
		case Gesture::WaveIn:
			mask |= GM_WAVEIN; break;
		case Gesture::WaveOut:
			mask |= GM_WAVEOUT; break;
		case Gesture::Pinch:
			mask |= GM_PINCH; break;
		case Gesture::Shoot:
			mask |= GM_SHOOT; break;
		default:
			GF_LOGD("%s: unknown gesture type: %u", __FUNCTION__, static_cast<GF_UINT32>(interestingGesture[i]));
		}
	}
	GF_LOGD("%s: mask = %8.8X", __FUNCTION__, mask);
	const GF_UINT8 length = 5;
	GF_UINT8 data[length];
	data[0] = CMD_SET_GESTURE_CONFIG;
	data[1] = (GF_UINT8)(mask);
	data[2] = (GF_UINT8)(mask >> 8);
	data[3] = (GF_UINT8)(mask >> 16);
	data[4] = (GF_UINT8)(mask >> 24);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.14
GF_RET_CODE DeviceSettingDataProfile4::getEMGRawDataCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz,
	EMGRowDataChannels supportedChannels, GF_UINT8 maxPackageDataLength)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_EMG_RAWDATA_CAP);
}
// C.14
GF_RET_CODE DeviceSettingDataProfile4::setEMGRawDataConfig(GF_UINT16 sampleRateHz,
	EMGRowDataChannels interestingChannels, GF_UINT8 packageDataLength, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz), channels = %u, package data length = %u(bytes)",
		__FUNCTION__, sampleRateHz, interestingChannels, packageDataLength);

	// set mtu size first
	configMtuSize(packageDataLength);

	const GF_UINT8 length = 6;
	GF_UINT8 data[length];
	data[0] = CMD_SET_EMG_RAWDATA_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	data[3] = (GF_UINT8)(interestingChannels);
	data[4] = (GF_UINT8)(interestingChannels >> 8);
	data[5] = packageDataLength;
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.15
GF_RET_CODE DeviceSettingDataProfile4::getMouseDataCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_MOUSE_DATA_CAP);
}
// C.15
GF_RET_CODE DeviceSettingDataProfile4::setMouseDataConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz)", __FUNCTION__, sampleRateHz);
	const GF_UINT8 length = 3;
	GF_UINT8 data[length];
	data[0] = CMD_SET_MOUSE_DATA_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.16
GF_RET_CODE DeviceSettingDataProfile4::getJoystickDataCap(function<void(ResponseResult res, GF_UINT16 maxSampleRateHz)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_JOYSTICK_DATA_CAP);
}
// C.16
GF_RET_CODE DeviceSettingDataProfile4::setJoystickDataConfig(GF_UINT16 sampleRateHz, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: sample rate = %u(Hz)", __FUNCTION__, sampleRateHz);
	const GF_UINT8 length = 3;
	GF_UINT8 data[length];
	data[0] = CMD_SET_JOYSTICK_DATA_CONFIG;
	data[1] = (GF_UINT8)(sampleRateHz);
	data[2] = (GF_UINT8)(sampleRateHz >> 8);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}
// C.17
GF_RET_CODE DeviceSettingDataProfile4::getDeviceStatusCap(function<void(ResponseResult res, DeviceStatusFlags flags)> cb)
{
	GF_LOGD("%s", __FUNCTION__);
	DO_SEND_GET_COMMAND(CMD_GET_DEVICE_STATUS_CAP);
}
// C.17
GF_RET_CODE DeviceSettingDataProfile4::setDeviceStatusConfig(DeviceStatusFlags flags, function<void(ResponseResult res)> cb)
{
	GF_LOGD("%s: flags = 0x%4.4X", __FUNCTION__, flags);
	const GF_UINT8 length = 3;
	GF_UINT8 data[length];
	data[0] = CMD_SET_DEVICE_STATUS_CONFIG;
	data[1] = (GF_UINT8)(flags);
	data[2] = (GF_UINT8)(flags >> 8);
	DO_SEND_COMMAND_AND_CALLBACK(length, data);
}



///////////////////////////////////////////////////////////////////////////////////////////////////

tstring DeviceSettingDataProfile4::getStringCommon(GF_UINT8 length, GF_PUINT8 data)
{
	vector<char> buf((GF_SIZE)(length + 1));
	char* str = buf.data();
	memcpy(str, data, length);
	str[length] = '\0';
	return utils::totstring(str);
}

void DeviceSettingDataProfile4::onProtocolVer(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mProtocolVer;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		std::ostringstream stm;
		stm << 'V' << (GF_UINT32)data[0] << '.' << (GF_UINT32)data[1];
		v = utils::totstring(stm.str());
	};
	ON_RESPONSE_PARSING(f, 2, void(ResponseResult, tstring), mProtocolVer);
}
void DeviceSettingDataProfile4::onFeatureMap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	GF_UINT32& v = mFeatureMap;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		v = (GF_UINT32)(data[0])
			| ((GF_UINT32)(data[1]) << 8)
			| ((GF_UINT32)(data[2]) << 16)
			| ((GF_UINT32)(data[3]) << 24);
	};
	ON_RESPONSE_PARSING(f, 4, void(ResponseResult, GF_UINT32), mFeatureMap);
}
void DeviceSettingDataProfile4::onDeviceName(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mDeviceName;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		v = getStringCommon(length, data);
	};
	ON_RESPONSE_PARSING(f, 0, void(ResponseResult, tstring), mDeviceName);
}
void DeviceSettingDataProfile4::onModelNumber(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mModelNumber;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		v = getStringCommon(length, data);
	};
	ON_RESPONSE_PARSING(f, 0, void(ResponseResult, tstring), mModelNumber);
}
void DeviceSettingDataProfile4::onSerialNumber(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mSerialNumber;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		v = getStringCommon(length, data);
	};
	ON_RESPONSE_PARSING(f, 0, void(ResponseResult, tstring), mSerialNumber);
}
void DeviceSettingDataProfile4::onHWRevision(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mHWRev;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		std::ostringstream stm;
		stm << 'R' << (GF_UINT32)data[0];
		v = utils::totstring(stm.str());
	};
	ON_RESPONSE_PARSING(f, 1, void(ResponseResult, tstring), mHWRev);
}
void DeviceSettingDataProfile4::onFWRevision(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mFWRev;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		std::ostringstream stm;
		stm << 'V' << (GF_UINT32)data[0] << '.' << (GF_UINT32)data[1] << '-' << (GF_UINT32)data[2];
		v = utils::totstring(stm.str());
	};
	ON_RESPONSE_PARSING(f, 3, void(ResponseResult, tstring), mFWRev);
}
void DeviceSettingDataProfile4::onManufacturerName(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mManuName;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		v = getStringCommon(length, data);
	};
	ON_RESPONSE_PARSING(f, 0, void(ResponseResult, tstring), mManuName);
}
void DeviceSettingDataProfile4::onBootloaderVer(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	auto& v = mBootloaderVer;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		std::ostringstream stm;
		stm << 'V' << (GF_UINT32)data[0] << '.' << (GF_UINT32)data[1] << '-' << (GF_UINT32)data[2];
		v = utils::totstring(stm.str());
	};
	ON_RESPONSE_PARSING(f, 3, void(ResponseResult, tstring), mBootloaderVer);
}
void DeviceSettingDataProfile4::onSendTrainingModelData(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_UINT16();
}

void DeviceSettingDataProfile4::onSetDataNotifSwitch(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetBatteryLevel(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	GF_UINT8 v = 0;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		v = data[0];
	};
	ON_RESPONSE_PARSING(f, 1, void(ResponseResult, GF_UINT32), v);
}
void DeviceSettingDataProfile4::onGetTemperature(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	GF_UINT8 v = 0;
	auto f = [&v](GF_UINT8 length, GF_PUINT8 data) {
		// according to the spec, the value should subtract 40
		v = data[0] - 40;
	};
	ON_RESPONSE_PARSING(f, 1, void(ResponseResult, GF_UINT32), v);
}

void DeviceSettingDataProfile4::onSetLogLevel(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onSetLogModuleEnabled(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onPrintKernelMsg(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onVibrateControl(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onLedControlTest(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onPackageIdControl(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetAccelerateCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_4BYTES_WITH_211();
}
void DeviceSettingDataProfile4::onSetAccelerateConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetGyroscopeCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_5BYTES_WITH_221();
}
void DeviceSettingDataProfile4::onSetGyroscopeConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetMagnetometerCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_5BYTES_WITH_221();
}
void DeviceSettingDataProfile4::onSetMagnetometerConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetEulerangleCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_UINT16();
}
void DeviceSettingDataProfile4::onSetEulerangleConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetQuaternionCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_UINT16();
}
void DeviceSettingDataProfile4::onSetQuaternionConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetRotationMatrixCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_UINT16();
}
void DeviceSettingDataProfile4::onSetRotationMatrixConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetGestureCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	GF_UINT32 gestureMap = 0;
	vector<Gesture> supportedGestures;
	if (RC_SUCCESS == retval)
	{
		if (4 <= length)
		{
			gestureMap = (GF_UINT32)(data[0])
				| ((GF_UINT32)(data[1]) << 8)
				| ((GF_UINT32)(data[2]) << 16)
				| ((GF_UINT32)(data[3]) << 24);
			if (gestureMap | GM_RELAX)
				supportedGestures.push_back(Gesture::Relax);
			if (gestureMap | GM_FIST)
				supportedGestures.push_back(Gesture::Fist);
			if (gestureMap | GM_SPREAD)
				supportedGestures.push_back(Gesture::SpreadFingers);
			if (gestureMap | GM_WAVEIN)
				supportedGestures.push_back(Gesture::WaveIn);
			if (gestureMap | GM_WAVEOUT)
				supportedGestures.push_back(Gesture::WaveOut);
			if (gestureMap | GM_PINCH)
				supportedGestures.push_back(Gesture::Pinch);
			if (gestureMap | GM_SHOOT)
				supportedGestures.push_back(Gesture::Shoot);
		}
		else
		{
			retval = RC_FAILED;
		}
	}
	ON_RESPONSE_RESPRESULT_PARAM(void(ResponseResult res, GF_SIZE, const Gesture[]),
		supportedGestures.size(), supportedGestures.data());
}
void DeviceSettingDataProfile4::onSetGestureConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetEMGRawDataCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_5BYTES_WITH_221();
}
void DeviceSettingDataProfile4::onSetEMGRawDataConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}

void DeviceSettingDataProfile4::onGetMouseDataCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_UINT16();
}
void DeviceSettingDataProfile4::onSetMouseDataConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetJoystickDataCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_UINT16();
}
void DeviceSettingDataProfile4::onSetJoystickDataConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}
void DeviceSettingDataProfile4::onGetDeviceStatusCap(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_PARSING_2BYTES(DeviceStatusFlags);
}
void DeviceSettingDataProfile4::onSetDeviceStatusConfig(GF_UINT8 retval, GF_UINT8 length, GF_PUINT8 data, gfsPtr<void> cb)
{
	ON_RESPONSE_RESPRESULT();
}

void DeviceSettingDataProfile4::configMtuSize(GF_UINT16 newPayloadLength)
{
	auto dev = mDevice.lock();
	if (nullptr == dev)
		return;
	GF_UINT16 mtuSize = dev->getMTUsize();
	GF_UINT16 newSize = newPayloadLength + MTU_HEAD_LENGTH;
	if (newSize > mtuSize)
	{
		dev->configMtuSize(newSize);
	}
}

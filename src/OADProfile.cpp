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
#include "OADProfile.h"

using namespace gf;

GF_RET_CODE OADSetting::oadUpgrade(FILE* file, function<void(ResponseResult res, GF_UINT32 percentage)> progress)
{
	GF_LOGD("%s coming", __FUNCTION__);
	return GF_RET_CODE::GF_ERROR_NOT_SUPPORT;
}

void OADSetting::dispatchResponse(GF_UINT8 command, GF_UINT8 retval, GF_UINT8 length,
	GF_PUINT8 data, gfsPtr<void> cb, bool timeout)
{

}

void OADProfile::onData(GF_UINT8 length, GF_PUINT8 data)
{
	GF_LOGD("%s", __FUNCTION__);
}

void OADProfile::onResponse(GF_UINT8 length, GF_PUINT8 data)
{
	if (nullptr != mDevSetting)
		mDevSetting->onResponse(length, data);
}

void OADProfile::onDeviceStatus(DeviceConnectionStatus oldStatus, DeviceConnectionStatus newStatus)
{
}

gfsPtr<DeviceSetting> OADProfile::getDeviceSetting()
{
	gfsPtr<BLEDevice> device = mDevice.lock();
	if (nullptr == device)
	{
		mDevSetting.reset();
		return gfsPtr<DeviceSetting>();
	}

	if (nullptr == mDevSetting)
	{
		mDevSetting = make_shared<OADSetting>(device);
	}

	return mDevSetting;
}
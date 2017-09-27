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
#include "DeviceProfile.h"
#include "DeviceSettingHandle.h"

namespace gf
{
	enum class OADState : GF_UINT8 {
		OAD_STATE_IDLE,
		OAD_STATE_IDENTIFY,
		OAD_STATE_SEND_BLOCK,
		OAD_STATE_SEND_FAST,
		OAD_STATE_FAILED,
		OAD_STATE_SUCCESS
	};
	class OADSetting :
		public DeviceSettingHandle
	{
	public:
		OADSetting(gfwPtr<BLEDevice> device)
			: DeviceSettingHandle(device) {}
	public:
		virtual GF_RET_CODE oadUpgrade(FILE* file, function<void(ResponseResult res, GF_UINT32 percentage)> progress) override;
		virtual void dispatchResponse(GF_UINT8 command, GF_UINT8 retval, GF_UINT8 length,
			GF_PUINT8 data, gfsPtr<void> cb, bool timeout = false) override;
	public:
		GF_UINT8 mOADPercent;
		OADState mOADState;
		FILE* mOADFile;
		GF_UINT16 mReqNumber;
		function<void(ResponseResult res, GF_UINT32 percentage)> cb;
		std::mutex mMutex;
		std::condition_variable mCond;

	private:
		static void oadRun();
		static thread oadThread;
	};

	class OADProfile :
		public DeviceProfile
	{
	public:
		OADProfile(gfwPtr<BLEDevice> device)
			: DeviceProfile(device) {}

	public:
		virtual void onCharNotify(ProfileCharType type, GF_UINT8 length, GF_PUINT8 data) override;
		virtual void onDeviceStatus(DeviceConnectionStatus oldStatus, DeviceConnectionStatus newStatus) override;
		virtual gfsPtr<DeviceSetting> getDeviceSetting() override;

	private:
		gfsPtr<OADSetting> mDevSetting;
	};

}

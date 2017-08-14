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
#include "DeviceSettingHandle.h"
#include "BLEDevice.h"

using namespace gf;

DeviceSettingHandle::DeviceSettingHandle(gfwPtr<BLEDevice> device)
	: mDevice(device)
{
}


DeviceSettingHandle::~DeviceSettingHandle()
{
}

GF_RET_CODE DeviceSettingHandle::sendCommand(GF_UINT8 dataLen, GF_PUINT8 commandData, bool hasResponse)
{
	if (nullptr == commandData || 0 == dataLen)
		return GF_RET_CODE::GF_ERROR_BAD_PARAM;

	GF_UINT8 command = commandData[0];
	auto dev = mDevice.lock();
	if (nullptr == dev)
	{
		GF_LOGD("%s: error: command (0x%2.2X) failed, device is .", __FUNCTION__, command);
		return GF_RET_CODE::GF_ERROR_BAD_STATE;
	}

	lock_guard<mutex> lock(mMutex);
	// first check if there is similar command in the queue,
	// if yes, reject this request
	if (hasResponse && mExecutingList.find(command) != mExecutingList.end())
	{
		GF_LOGD("%s: error: same command (0x%2.2X) in executing.", __FUNCTION__, command);
		return GF_RET_CODE::GF_ERROR_DEVICE_BUSY;
	}

	// send the command to device
	GF_RET_CODE ret = dev->sendControlCommand(dataLen, commandData);
	if (GF_RET_CODE::GF_SUCCESS != ret)
	{
		GF_LOGD("%s: error: command (0x%2.2X) failed.", __FUNCTION__, command);
		return ret;
	}

	if (!hasResponse)
		return ret;

	// if the command has response, store the expiration of this command
	mExecutingList[command] = chrono::steady_clock::now() + MilisDuration(MAX_COMMAND_TIMEOUT);

	updateTimer();

	return ret;
}

void DeviceSettingHandle::onResponse(GF_UINT8 length, GF_PUINT8 data)
{
	GF_UINT8 ret = 0xFF;
	GF_UINT8 cmd = 0xFF;
	if (length >= 2)
	{
		ret = data[0];
		cmd = data[1];
	}

	GF_LOGD("%s. cmd = 0x%2.2X, ret = 0x%2.2X", __FUNCTION__, cmd, ret);
	{
		lock_guard<mutex> lock(mMutex);
		// check responders
		if (mExecutingList.find(cmd) == mExecutingList.end())
		{
			GF_LOGD("cmd 0x%2.2X is not handled", cmd);
			// not in list, do nothing
			return;
		}
		mExecutingList.erase(cmd);
		updateTimer();
	}

	dispatchResponse(cmd, ret, length - 2, length == 2 ? nullptr : data + 2);
}

void DeviceSettingHandle::updateTimer()
{
	auto now = chrono::steady_clock::now();
	// check and reset timer
	mTimer.stop();

	// find the earliest command and set timer
	auto invalidpoint = now + MilisDuration(MAX_COMMAND_TIMEOUT * 2);
	auto earliest = invalidpoint;
	for (auto& itor : mExecutingList)
	{
		if (itor.second < earliest)
			earliest = itor.second;
	}
	// if cannot find a reasonable time point, then stop timer
	if (earliest == invalidpoint)
	{
		//GF_LOGD("%s: executing list is empty.", __FUNCTION__);
		return;
	}

	if (earliest < now)
	{
		// if there is an item expired, process it next time
		earliest = now + MilisDuration(1);
	}

	mTimer.start((earliest - now), [this](){ this->onTimer(); });

}

void DeviceSettingHandle::onTimer()
{
	lock_guard<mutex> lock(mMutex);
	auto now = chrono::steady_clock::now();
	for (auto& itor : mExecutingList)
	{
		if (itor.second < now)
		{
			// find one expired
			//GF_LOGD("%s: command 0x%2.2X time out.", __FUNCTION__, itor.first);
			dispatchResponse(itor.first, 0xFF, 0, nullptr, true);
			// simply remove it and update timer
			// other possible expired items will be handled next time
			mExecutingList.erase(itor.first);
			break;
		}
	}
	updateTimer();
}

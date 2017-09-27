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
#include "SimpleProfile.h"
#include "Utils.h"
#include "LogUtils.h"
#include "Quaternion.h"

using namespace gf;

#if defined(DEBUG_ANALYSE_PACKAGE_LOST)
#ifndef WIN32
#include <unistd.h>
#endif
atomic<GF_UINT32> SimpleProfile::dataCnt(0);
atomic<GF_UINT32> SimpleProfile::lastUpdated(0);
thread SimpleProfile::dbgThread;
void SimpleProfile::timefun()
{
	bool loop = true;
	while (loop)
	{
		utils::sleep(1000);
		if (lastUpdated < dataCnt)
		{
			GF_LOGD("gForce data package rate: %d", dataCnt - lastUpdated);
			lastUpdated = dataCnt.load();
		}
	}
}
#endif

#define ATTRIB_HANDLE_GEVENT (0x27)
#define EVENT_RECENTER_MASK		0x01

void SimpleProfile::onCharNotify(ProfileCharType type, GF_UINT8 length, GF_PUINT8 data)
{
	gfsPtr<BLEDevice> device = mDevice.lock();
	if (nullptr == device)
		return;
	// parse data header
	if (length <= 4)
		return;
	GF_UINT32 offset = 1;
	GF_UINT16 attrib_handle = data[offset] | ((GF_UINT16)data[offset + 1] << 8);
	if (ATTRIB_HANDLE_GEVENT != attrib_handle)
	{
		GF_LOGD("%s: not supported event type. attrib_handle is 0x%4.4X", __FUNCTION__, attrib_handle);
		return;
	}
	offset += 2;

#if defined(DEBUG_ANALYSE_PACKAGE_LOST)
	if (!dbgThread.joinable())
		dbgThread = thread(SimpleProfile::timefun);
	dataCnt++;
#endif
	GF_UINT8 evtType = data[offset] & EVENT_MASK;
	GF_UINT8 packageIdFlag = (data[offset] >> 7) & PCKID_FLAG_MASK;
	offset++;
	GF_UINT8 payloadLength = data[offset++];
	GF_UINT16 currPackageId = INVALID_PACKAGE_ID;
	GF_PUINT8 payload = nullptr;

	BLEDevice& ref = *device.get();
	if (packageIdFlag != 0)
	{
		currPackageId = data[offset++];
		--payloadLength;

		lock_guard<mutex> lock(device->getLockable());
		if (mPackageId == INVALID_PACKAGE_ID)
		{
			mPackageId = currPackageId;
		}
		else
		{
			++mPackageId;
			if (mPackageId > 0xFF)
				mPackageId = 0;
			if (mPackageId != currPackageId)
			{
//#if defined(DEBUG_ANALYSE_PACKAGE_LOST)
				GF_LOGE("%s:%s: package id error. expect %u, but %u, gap %u", __FUNCTION__,
					utils::tostring(ref.getName()).c_str(), mPackageId,
					currPackageId, (GF_UINT8)(currPackageId - mPackageId));
//#endif
				mPackageId = currPackageId;
			}
		}
	}

	payload = &data[offset];
	switch (evtType)
	{
	case EVENT_QUATERNION:
		onQuaternion(ref, payloadLength, payload);
		break;
	case EVENT_GESTURE:
		onGesture(ref, payloadLength, payload);
		break;
	case EVENT_STATUS:
		onStatus(ref, payloadLength, payload);
		break;
	default:
		GF_LOGE("%s: unknown event ID: %2.2X", __FUNCTION__, evtType);
		;
	}
}

void SimpleProfile::onDeviceStatus(DeviceConnectionStatus oldStatus, DeviceConnectionStatus newStatus)
{
}

gfsPtr<DeviceSetting> SimpleProfile::getDeviceSetting()
{
	GF_LOGD("%s: No implementation.", __FUNCTION__);
	return gfsPtr<DeviceSetting>();
}

void SimpleProfile::onQuaternion(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
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

void SimpleProfile::onGesture(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
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

void SimpleProfile::onStatus(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data)
{
	GF_LOGD("%s, length: %u", __FUNCTION__, length);
	if (length < 1)
		return;

	GF_UINT8 status = data[0] & EVENT_RECENTER_MASK;
	if (1 == status)
	{
		device.getHub().notifyDeviceStatusChanged(device, DeviceStatus::ReCenter);
	}
}

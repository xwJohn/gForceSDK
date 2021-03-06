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
 * \file SimpleProfile.h
 * \brief Implement the BLE simple profile protocol
 *
 * \version 0.1
 * \date 2017.4.0
 */

#pragma once

#include "DeviceProfile.h"

//#define DEBUG_ANALYSE_PACKAGE_LOST

#if defined(DEBUG_ANALYSE_PACKAGE_LOST)
#include <chrono>
#include <atomic>
#include <thread>
#endif

namespace gf
{
	class BLEHub;

	enum {
		EVENT_QUATERNION = 0x02,
		EVENT_GESTURE = 0x0F,
		EVENT_STATUS = 0x14
	};

	class SimpleProfile : public DeviceProfile
	{
	public:
		SimpleProfile(gfwPtr<BLEDevice> device) : DeviceProfile(device) {}
		virtual ~SimpleProfile() {}

	protected:
		virtual void onData(GF_UINT8 length, GF_PUINT8 data) override;
		virtual void onResponse(GF_UINT8 length, GF_PUINT8 data) override;
		virtual void onDeviceStatus(DeviceConnectionStatus oldStatus, DeviceConnectionStatus newStatus) override;
		virtual gfsPtr<DeviceSetting> getDeviceSetting() override;

	private:
		void onQuaternion(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onGesture(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		void onStatus(BLEDevice& device, GF_UINT8 length, GF_PUINT8 data);
		GF_UINT16 mPackageId = INVALID_PACKAGE_ID;

#if defined(DEBUG_ANALYSE_PACKAGE_LOST)
	private:
		static atomic<GF_UINT32> dataCnt;
		static atomic<GF_UINT32> lastUpdated;
		static void timefun();
		static thread dbgThread;
#endif
	};

} // namespace gf

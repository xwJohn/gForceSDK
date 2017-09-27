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

#include "gfTypes.h"
#include "BLEDevice.h"

#include <mutex>

namespace gf
{

#define EVENT_MASK				0x7F
#define PCKID_FLAG_MASK			0x01
#define INVALID_PACKAGE_ID		0xFFFF

	class DeviceSetting;

	class DeviceProfile
	{
	public:
		DeviceProfile(gfwPtr<BLEDevice> device)
			: mDevice(device)
		{}
		virtual void onCharNotify(ProfileCharType type, GF_UINT8 length, GF_PUINT8 data) = 0;
		virtual void onDeviceStatus(DeviceConnectionStatus oldStatus, DeviceConnectionStatus newStatus) = 0;
		virtual gfsPtr<DeviceSetting> getDeviceSetting() = 0;

	protected:
		/// \brief Virtual deconstructor
		///
		virtual ~DeviceProfile() {}

	protected:
		gfwPtr<BLEDevice> mDevice;
	};

}

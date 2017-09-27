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

#include "GFBLETypes.h"
#include "gfTypes.h"
#include "Quaternion.h"
#include <functional>
#include <vector>
using namespace std;

namespace gf
{
	class BLEDevice;
	class HubListener;

	enum class AttributeHandle : GF_UINT16 {
		Max,
	};
	enum class DeviceProtocolType : GF_UINT32 {
		SimpleProfile = ProtocolType_SimpleProfile,
		DataProtocol = ProtocolType_DataProtocol,
		OADService = ProtocolType_OADService,
		Invalid = ProtocolType_Invalid,
	};
	class IHub
	{
	public:
		virtual GF_RET_CODE connect(BLEDevice& dev, bool directConn = true) = 0;
		virtual GF_RET_CODE cancelConnect(BLEDevice& dev) = 0;
		virtual GF_RET_CODE disconnect(BLEDevice& dev) = 0;
		virtual GF_RET_CODE configMtuSize(BLEDevice& dev, GF_UINT16 mtuSize) = 0;
		virtual GF_RET_CODE connectionParameterUpdate(BLEDevice& dev,
			GF_UINT16 conn_interval_min, GF_UINT16 conn_interval_max,
			GF_UINT16 slave_latence, GF_UINT16 supervision_timeout) = 0;
		virtual GF_RET_CODE writeCharacteristic(BLEDevice& dev,
			AttributeHandle attribute_handle, GF_UINT8 data_length, GF_PUINT8 data) = 0;
		virtual GF_RET_CODE readCharacteristic(BLEDevice& dev, AttributeHandle attribute_handle) = 0;
		virtual GF_RET_CODE getProtocol(BLEDevice& dev, DeviceProtocolType& type) = 0;
		virtual GF_RET_CODE sendControlCommand(BLEDevice& dev, ProfileCharType type, GF_UINT8 data_length, GF_PUINT8 data) = 0;

		virtual void notifyOrientationData(BLEDevice& dev, const Quaternion& rotation) = 0;
		virtual void notifyGestureData(BLEDevice& dev, Gesture gest) = 0;
		virtual void notifyDeviceStatusChanged(BLEDevice& dev, DeviceStatus status) = 0;
		virtual void notifyExtendedData(BLEDevice& dev, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data) = 0;
	};
} // namespace gf


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
#ifndef ICLIENTCALLBADK
#define ICLIENTCALLBADK
#include "GFBLETypes.h"

class GF_CClientCallback
{
public:
	virtual void onScanResult(GF_BLEDevice* device) = 0;
	virtual void onScanFinished() = 0;
	virtual void onDeviceConnected(GF_STATUS status, GF_ConnectedDevice *device) = 0;
	virtual void onDeviceDisconnected(GF_STATUS status, GF_ConnectedDevice *device, GF_UINT8 reason) = 0;

	virtual void onMTUSizeChanged(GF_STATUS status, GF_UINT16 handle, GF_UINT16 mtu_size) = 0;
	virtual void onConnectionParmeterUpdated(GF_STATUS status, GF_UINT16 handle, GF_UINT16 conn_int, GF_UINT16 superTO, GF_UINT16 slavelatency) = 0;
	virtual void onCharacteristicValueRead(GF_STATUS status, GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) = 0;

	/*Notification format: data length(1 byte N) + data(N Bytes)*/
	virtual void onNotificationReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) = 0;

	/*Control command response from device that support gForce data protocol.
	 *Notification format: Response status(1 byte) + cmd type(1 byte) + Response parameter(depends on cmd type)*/
	virtual void onControlResponseReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) = 0;

	/*Characteristic notify from device that support OAD protocol*/
	virtual void onOADFailedReceived(GF_UINT16 handle) = 0;
	virtual void onOADBlockRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) = 0;
	virtual void onOADFastRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data) = 0;
	
	virtual void onComDestory() = 0;
};
#endif
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

#define OAD_HEADER_LEN (16)
#define OAD_RESPONSE_LEN (12)
#define OAD_BLOCK_LEN (16)
////////////////////////////////////////////////////////
// API
#define DO_SEND_COMMAND_AND_CALLBACK_NO_RETURN(type, retval, length, buf)	\
do {	\
	retval = oadDevHdl->sendCommand(type, (length), (buf), false, nullptr);	\
} while (false)

#define DO_SEND_COMMAND_AND_CALLBACK(type, length, buf)	\
do {	\
	GF_RET_CODE retval;	\
	DO_SEND_COMMAND_AND_CALLBACK_NO_RETURN(type, retval, length, buf);	\
	return retval;	\
} while (false)


static OADSetting* oadDevHdl;
thread OADSetting::oadThread;
void OADSetting::oadRun()
{
	GF_LOGD("OAD Thread Start\n");
	bool loop = true;
	GF_RET_CODE retval;
	GF_UINT32 size;
	GF_UINT32 total_pack;
	GF_UINT32 result;
	GF_UINT8 buffer[2+OAD_BLOCK_LEN];

	std::unique_lock <std::mutex> lock(oadDevHdl->mMutex);

	while (loop)
	{
		switch(oadDevHdl->mOADState)
		{
			case OADState::OAD_STATE_IDLE:
				oadDevHdl->mCond.wait(lock);
				break;
			case OADState::OAD_STATE_IDENTIFY:
				{
				fseek (oadDevHdl->mOADFile, 0, SEEK_END);   //get file size
				size=ftell (oadDevHdl->mOADFile);
				rewind(oadDevHdl->mOADFile);

				total_pack = (size-OAD_HEADER_LEN)/OAD_BLOCK_LEN;
				GF_LOGD("OAD file size = %d\n",size);
				result = fread (buffer,1,OAD_HEADER_LEN,oadDevHdl->mOADFile);
				if(result != OAD_HEADER_LEN)
				{
					GF_LOGD("Reading error(%d)\n",result);
					oadDevHdl->mOADState = OADState::OAD_STATE_FAILED;
				}
				else
				{
					//send 16bytes header info
					DO_SEND_COMMAND_AND_CALLBACK_NO_RETURN(ProfileCharType::PROF_OAD_IDENTIFY, \
						                                  retval, OAD_HEADER_LEN, buffer);
				}
				}
				if(retval == GF_RET_CODE::GF_SUCCESS)
				    oadDevHdl->mOADState = OADState::OAD_STATE_IDLE;
				break;
			case OADState::OAD_STATE_SEND_BLOCK:
				buffer[0] = static_cast<GF_UINT8>(oadDevHdl->mReqNumber & 0x00FF);
				buffer[1] = static_cast<GF_UINT8>(oadDevHdl->mReqNumber >> 8);
				fseek(oadDevHdl->mOADFile, OAD_HEADER_LEN+(oadDevHdl->mReqNumber-1)*OAD_BLOCK_LEN, SEEK_SET);
				result = fread (&buffer[2], 1, OAD_BLOCK_LEN, oadDevHdl->mOADFile);

				if(result != OAD_HEADER_LEN)
				{
					GF_LOGD("Reading error(%d)\n",result);
					oadDevHdl->mOADState = OADState::OAD_STATE_FAILED;
				}
				else
				{
					//GF_LOGD("%s",utils::tostring(utils::deviceAddressToStringT(buffer, sizeof(buffer))).c_str());
					//send 16bytes header info
					DO_SEND_COMMAND_AND_CALLBACK_NO_RETURN(ProfileCharType::PROF_OAD_BLOCK, \
														retval, 2+OAD_BLOCK_LEN, buffer);
					if (retval == GF_RET_CODE::GF_SUCCESS)
					{
						oadDevHdl->mOADPercent = static_cast<GF_UINT8>(static_cast<GF_FLOAT>(oadDevHdl->mReqNumber) / total_pack * 100);
						oadDevHdl->cb(ResponseResult::RREST_SUCCESS,oadDevHdl->mOADPercent);
						if(oadDevHdl->mOADPercent == 100)
						{
							oadDevHdl->mOADState = OADState::OAD_STATE_SUCCESS;
						}
						else
						{
							oadDevHdl->mOADState = OADState::OAD_STATE_IDLE;
						}
					}
					else
					{
						GF_LOGD("SendCommandFailed(%d)",retval);
					}				
				}
				break;
			case OADState::OAD_STATE_SEND_FAST:
				buffer[0] = static_cast<GF_UINT8>(oadDevHdl->mReqNumber & 0x00FF);
				buffer[1] = static_cast<GF_UINT8>(oadDevHdl->mReqNumber >> 8);
				fseek(oadDevHdl->mOADFile, OAD_HEADER_LEN+(oadDevHdl->mReqNumber-1)*OAD_BLOCK_LEN, SEEK_SET);
				result = fread (&buffer[2], 1, OAD_BLOCK_LEN, oadDevHdl->mOADFile);

				if(result != OAD_HEADER_LEN)
				{
					GF_LOGD("Reading error(%d)\n",result);
					oadDevHdl->mOADState = OADState::OAD_STATE_FAILED;
				}
				else
				{
					//send 16bytes header info
					DO_SEND_COMMAND_AND_CALLBACK_NO_RETURN(ProfileCharType::PROF_OAD_FAST, \
														retval, 2+OAD_BLOCK_LEN, buffer);
					if (retval == GF_RET_CODE::GF_SUCCESS)
					{
						oadDevHdl->mOADPercent = static_cast<GF_UINT8>(static_cast<GF_FLOAT>(oadDevHdl->mReqNumber) / total_pack * 100);
						oadDevHdl->cb(ResponseResult::RREST_SUCCESS,oadDevHdl->mOADPercent);
						if(oadDevHdl->mOADPercent == 100)
						{
							oadDevHdl->mOADState = OADState::OAD_STATE_SUCCESS;
						}
						else
						{
							oadDevHdl->mReqNumber++;
							utils::sleep(80);
						}
					}
					else
					{
						GF_LOGD("SendCommandFailed(%d)",retval);
					}				
				}
				break;
			case OADState::OAD_STATE_FAILED:
				GF_LOGD("OAD State update failed.");
				fclose(oadDevHdl->mOADFile);
				oadDevHdl->cb(ResponseResult::RREST_FAILED,oadDevHdl->mOADPercent);
				loop = false;
				break;
			case OADState::OAD_STATE_SUCCESS:
				GF_LOGD("OAD State update success.");
				fclose(oadDevHdl->mOADFile);
				oadDevHdl->cb(ResponseResult::RREST_SUCCESS,oadDevHdl->mOADPercent);
				loop = false;
				break;
			default:
				GF_LOGD("OAD State Error: %d", oadDevHdl->mOADState);
				loop = false;
				break;
		}
	}
	GF_LOGD("OAD Thread Exit With State: %d\n", oadDevHdl->mOADState);
}

GF_RET_CODE OADSetting::oadUpgrade(FILE* file, function<void(ResponseResult res, GF_UINT32 percentage)> progress)
{
	if(file && progress)
	{
		if (!oadThread.joinable())
		    oadThread = thread(OADSetting::oadRun);

		oadDevHdl = this;
		oadDevHdl->mOADFile = file;
		oadDevHdl->mOADState = OADState::OAD_STATE_IDENTIFY;
		oadDevHdl->mOADPercent = 0;
		oadDevHdl->cb = progress;

		oadThread.join();
		return GF_RET_CODE::GF_SUCCESS;
	}
	return GF_RET_CODE::GF_ERROR;
}

void OADSetting::dispatchResponse(GF_UINT8 command, GF_UINT8 retval, GF_UINT8 length,
	GF_PUINT8 data, gfsPtr<void> cb, bool timeout)
{

}

void OADProfile::onCharNotify(ProfileCharType type, GF_UINT8 length, GF_PUINT8 data)
{
	std::unique_lock <std::mutex> lock(oadDevHdl->mMutex);
	if(ProfileCharType::PROF_OAD_IDENTIFY == type)
	{
		GF_LOGD("OAD Identify failed,packID=%d\n", oadDevHdl->mReqNumber);
	    oadDevHdl->mOADState = OADState::OAD_STATE_FAILED;
	}
	else if(ProfileCharType::PROF_OAD_BLOCK == type)
	{
		//normal oad mode supported
		oadDevHdl->mReqNumber= data[0] + (data[1]<<8);
		GF_LOGD("OAD request block: %d", oadDevHdl->mReqNumber);
		if(oadDevHdl->mReqNumber == 0xFFFF){
			GF_LOGD("OAD request block failed\n");
			oadDevHdl->mOADState = OADState::OAD_STATE_FAILED;
		}else{
			oadDevHdl->mOADState = OADState::OAD_STATE_SEND_BLOCK;
		}

	}
	else if(ProfileCharType::PROF_OAD_FAST == type)
	{
		//fast oad mode supported
		oadDevHdl->mReqNumber= data[0] + (data[1]<<8);
		GF_LOGD("OAD request fast: %d", oadDevHdl->mReqNumber);
		if(oadDevHdl->mReqNumber == 0xFFFF){
			GF_LOGD("OAD request block failed\n");
			oadDevHdl->mOADState = OADState::OAD_STATE_FAILED;
		}else{
			oadDevHdl->mOADState = OADState::OAD_STATE_SEND_FAST;
		}
	}
	else
	{
		GF_LOGD("=======Type Error: %d=======\n", type);
	}
	oadDevHdl->mCond.notify_all();
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
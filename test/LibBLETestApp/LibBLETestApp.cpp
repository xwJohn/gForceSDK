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
// App.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "windows.h"
#include "app.h"
#include "ClientCallbackInterface.h"
#include "AdapterManagerInterface.h"
#include "list"
using namespace std;

bool ctrlhandler(DWORD fdwctrltype)
{
	switch (fdwctrltype)
	{
		// handle the ctrl-c signal.
	case CTRL_C_EVENT:
		printf("ctrl-c event\n\n");
		return(true);
		// ctrl-close: confirm that the user wants to exit.
	case CTRL_CLOSE_EVENT:
		printf("ctrl-close event\n\n");
		return(false);
		// pass other signals to the next handler.
	case CTRL_BREAK_EVENT:
		printf("ctrl-break event\n\n");
		return false;
	case CTRL_LOGOFF_EVENT:
		printf("ctrl-logoff event\n\n");
		return false;
	case CTRL_SHUTDOWN_EVENT:
		printf("ctrl-shutdown event\n\n");
		return false;
	default:
		return false;
	}
}

class GattClient :public GF_CClientCallback
{
public:
	GattClient(GF_CAdapterManagerInterface* am)
	{
		mAMInterface = am;
	}

	void onScanResult(GF_BLEDevice* GF_BLEDevice)
	{
		printf("\n onScanResult RSSI=%d\n", GF_BLEDevice->rssi);
		GF_CDevice* device = new GF_CDevice;
		device->address_type = GF_BLEDevice->addr_type;
		memcpy(device->address, GF_BLEDevice->addr, 6);
		memcpy(device->name, GF_BLEDevice->dev_name, 50);
		device->rssi = GF_BLEDevice->rssi;
		device->state = 0;

        //insert item based on rssi
        list<GF_CDevice*>::iterator iter;
		if(mAvailabeDevice.size() == 0)
		{
			mAvailabeDevice.push_front(device);
		}
		else
		{
			for(iter = mAvailabeDevice.begin(); iter != mAvailabeDevice.end(); iter++)
			{
				if(((GF_CDevice*)(*iter))->rssi < device->rssi)
				{
					mAvailabeDevice.insert(iter, device);
					break;
				}
			}
			if (iter == mAvailabeDevice.end())
			{
				mAvailabeDevice.push_back(device);
			}
		}

	}

	void onScanFinished()
	{
		GF_CDevice* device;
		printf("\n onScanFinished ITEM=%d\n", mAvailabeDevice.size());
        list<GF_CDevice*>::iterator iter;

		if (mAvailabeDevice.size() != 0)
		{
			device = mAvailabeDevice.front();
			mAMInterface->Connect(device->address, device->address_type, GF_TRUE);
		}
	}

	GF_UINT8 ControlCommandParameter[10][1];

	void onDeviceConnected(GF_STATUS status, GF_ConnectedDevice *device)
	{
		printf("\n onDeviceConnected withs status = %d\n", status);
		if (GF_OK == status)
		{
			printf("\n protocol of device is = %d\n", mAMInterface->GetDeviceProtocolSupported(device->handle));
			Sleep(1000);
			if (ProtocolType_DataProtocol == mAMInterface->GetDeviceProtocolSupported(device->handle))
			{
				for (GF_UINT8 i = 0; i < 10; i++)
				{
					ControlCommandParameter[i][0] = i;
					printf("\n SendControlCommand cmd type: %d!\n", ControlCommandParameter[i][0]);
					if (GF_OK == mAMInterface->SendControlCommand(device->handle, ProfileCharType::PROF_DATA_CMD, 0x01, ControlCommandParameter[i]))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
				}
			}
		}
		
	}

	void onDeviceDisconnected(GF_STATUS status, GF_ConnectedDevice *device, GF_UINT8 reason)
	{
		printf("\n onDeviceDisconnected with status = %d\n", status);
		Sleep(5000);
		mAMInterface->Connect(device->address, device->address_type, GF_TRUE);
		//Sleep(5000);
		//mAMInterface->CancelConnect(device->address, device->address_type);
	}

	void onMTUSizeChanged(GF_STATUS status, GF_UINT16 handle, GF_UINT16 mtu_size)
	{
		printf("\n onMTUSizeChanged \n");
	}

	void onConnectionParmeterUpdated(GF_STATUS status, GF_UINT16 handle, GF_UINT16 conn_int, GF_UINT16 superTO, GF_UINT16 slavelatency)
	{
		printf("\n onConnectionParmeterUpdated \n");
	}

	void onCharacteristicValueRead(GF_STATUS status, GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
	{
		printf("\n onChracteristicValueRead \n");
	}

	void onComDestory()
	{
		printf("\n onComDestory \n");
	}

	void onNotificationReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
	{
		GF_UINT8 i = 0;
		printf("\n onNotificationReceived with length = %d: ", length);
		for (i = 0; i < length; i++)
		{
			printf("0x%x :", data[i]);
		}
	}

	void onControlResponseReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
	{
		GF_UINT8 i = 0;
		printf("\n onControlRespenseReceived with length = %d\n", length);
		for (i = 0; i < length; i++)
		{
			printf("data[%d] = %d\n", i, data[i]);
		}

		GF_UINT8 status = data[0];
		GF_UINT8 cmd_type = data[1];

		switch (cmd_type)
		{
			case GET_PROTOCOL_VERSION:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					printf("Protocol version: %d.%d\n", data[2], data[3]);
					//Sleep(1000);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_FEATURE_MAP);
					GF_UINT8 parameter[1] = { GET_FEATURE_MAP };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Protocol version with error, error code = %d\n", status);
				}
				break;
			}
			case GET_FEATURE_MAP:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					printf("Feature Map: %x:%x:%x:%x\n", data[2], data[3], data[4], data[5]);
					//Sleep(1000);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_DEVICE_NAME);
					GF_UINT8 parameter[1] = { GET_DEVICE_NAME };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Feature Map with error, error code = %d\n", status);
				}
				break;
			}
			case GET_DEVICE_NAME:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					GF_UINT8 dev_name[50];
					memcpy(dev_name, data + 2, length - 2);
					dev_name[length - 2] = '\0';
					printf("Device Name: %s\n", dev_name);
					//Sleep(1000);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_MODEL_NUMBER);
					GF_UINT8 parameter[1] = { GET_MODEL_NUMBER };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Device Name with error, error code = %d\n", status);
				}
				break;
			}
			case GET_MODEL_NUMBER:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					GF_UINT8 model_num[50];
					memcpy(model_num, data + 2, length - 2);
					model_num[length - 2] = '\0';
					printf("Model Number: %s\n", model_num);
					//Sleep(1000);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_SERIAL_NUMBER);
					GF_UINT8 parameter[1] = { GET_SERIAL_NUMBER };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Model Number with error, error code = %d\n", status);
				}
				break;
			}
			case GET_SERIAL_NUMBER:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					GF_UINT8 serial_num[50];
					memcpy(serial_num, data + 2, length - 2);
					serial_num[length - 2] = '\0';
					printf("Serial Number: %s\n", serial_num);
					//Sleep(1000);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_HARDWARE_REVISION);
					GF_UINT8 parameter[1] = { GET_HARDWARE_REVISION };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Serial Number with error, error code = %d\n", status);
				}
				break;
			}

			case GET_HARDWARE_REVISION:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					printf("Hardware Revision R%d\n", data[2]);
					//Sleep(1000);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_FIRMWARE_REVISION);
					GF_UINT8 parameter[1] = { GET_FIRMWARE_REVISION };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Hardware Revision with error, error code = %d\n", status);
				}
				break;
			}

			case GET_FIRMWARE_REVISION:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					printf("Firmware Revision R%d.%d-%d\n", data[2], data[3], data[4]);
					//Sleep(1000);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_MANUFACTURER_NAME);
					GF_UINT8 parameter[1] = { GET_MANUFACTURER_NAME };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Firmwar Revision with error, error code = %d\n", status);
				}
				break;
			}
			case GET_MANUFACTURER_NAME:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					GF_UINT8 manu_name[50];
					memcpy(manu_name, data + 2, length - 2);
					manu_name[length - 2] = '\0';
					printf("Manufacturer Name: %s\n", manu_name);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", GET_BATTERY_LEVEL);
					GF_UINT8 parameter[1] = { GET_BATTERY_LEVEL };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x01, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Manufacturer Name with error, error code = %d\n", status);
				}
				break;
			}
			case GET_BATTERY_LEVEL:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					printf("Battery Level: %d%\n", data[2]);
					/*
					printf("\n SendControlCommand cmd type: %d!\n", 0x4F);
					GF_UINT8 parameter[5] = { 0x4F, (GF_UINT8)0xFF, (GF_UINT8)0xFF, (GF_UINT8)0xFF, (GF_UINT8)0xFF };
					if (GF_OK == mAMInterface->SendControlCommand(handle, 0x05, parameter))
					{
						printf("\n SendControlCommand sucessful!\n");
					}
					*/
				}
				else
				{
					printf("Get Battery Level with error, error code = %d\n", status);
				}
				break;
			}
			case GET_TEMPERATURE_LEVEL:
			{
				if (status == CONTROL_COMMAND_RESPONSE_OK)
				{
					printf("Temperature Level: %d degree\n", (data[2] - 40));
					Sleep(1000);

					//mAMInterface->Disconnect(handle);
				}
				else
				{
					printf("Get Temperature Level with error, error code = %d\n", status);
				}
				break;
			}
			default:
				printf("unknown command type = %d\n", cmd_type);
		}
	}
	void onOADFailedReceived(GF_UINT16 handle)
	{
	}
	void onOADBlockRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
	{
	}
	void onOADFastRequestReceived(GF_UINT16 handle, GF_UINT8 length, GF_PUINT8 data)
	{
	}
private:
	GF_CAdapterManagerInterface* mAMInterface;
	list<GF_CDevice*> mAvailabeDevice;
};

int _tmain(int charc, char* argv[]) {
	GF_STATUS status;
	GF_UINT8 loop = 0;

	GF_CAdapterManagerInterface* amInterface = GF_CAdapterManagerInterface::GetInstance();
	GattClient gattclient(amInterface);
	if (amInterface != NULL && amInterface->Init(0, LOGTYPE_FILE) == GF_OK)
	{
		printf("\n Start to scan... \n");
		amInterface->RegisterClientCallback(&gattclient);
		status = amInterface->StartScan(100);
	}
	else
	{
		printf("\n Com Open Failed!! \n");
		Sleep(5000);
		exit(0);
	}

	if (SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlhandler, true))
	{
		while (1)
		{
			Sleep(5000);
			loop++;
			printf("loop = %d \n", loop);
		}
	}
	else
	{
		printf("\nerror: could not set control handler");
	}

	return 0;
}

#ifndef __ADAPTERMANAGER_H__
#define __ADAPTERMANAGER_H__
#include "oym_types.h"
#include "OYM_NIF.h"
#include <OYM_Log.h>
#include <OYM_Callback.h>
#include "DiscoveryService.h"
#include "RemoteDevice.h"

#define MODUAL_TAG_AM "AdaperManager"
//#define ADAPTERMANAGER_EVENT (EVENT_MASK_LINK_PARA_UPDATE_MSG | EVENT_MASK_AUTH_COMPLETE_MSG | EVENT_MASK_SLAVE_REQUESTED_SECURITY_MSG | EVENT_MASK_INTERNAL_DEVICE_FOUND | EVENT_MASK_INTERNAL_SCAN_FINISHED | EVENT_MASK_GAP_LINK_ESTABLISHED_MSG)

#define ADAPTER_MANAGER_CALLBACK_INDEX 1
#define ADAPTER_MANAGER_ATT_EVENT  (EVENT_MASK_DEVICE_CONNECTED | EVENT_MASK_INTERNAL_SCAN_FINISHED | EVENT_MASK_ATT_WRITE_MSG | EVENT_MASK_ATT_EXCHANGE_MTU_MSG | EVENT_MASK_ATT_READ_BLOB_RESP_MSG | EVENT_MASK_ATT_NOTI_MSG | EVENT_MASK_ATT_READ_RESP_MSG | EVENT_MASK_ATT_ERROR_MSG | EVENT_MASK_ATT_READ_BY_GRP_TYPE_MSG | EVENT_MASK_ATT_READ_BY_TYPE_MSG | EVENT_MASK_ATT_READ_BY_INFO_MSG)
#define ADAPTER_MANAGER_EVENT (0x0100FD | ADAPTER_MANAGER_ATT_EVENT)

class OYM_AdapterManager: public OYM_CallBack
{
public:
	OYM_AdapterManager();
	~OYM_AdapterManager();

	OYM_STATUS Init();
	OYM_STATUS Deinit();
	OYM_STATUS StartScan(OYM_UINT8 RSSI_Threshold);
	OYM_STATUS StopScan();
	OYM_STATUS Connect(OYM_PUINT8 addr, UINT8 addr_type, OYM_BOOL is_direct_conn);
	OYM_STATUS CancelConnect(OYM_PUINT8 addr, OYM_UINT8 addr_type);
	OYM_STATUS Disconnect(OYM_UINT16 handle);
	OYM_STATUS RegisterClientCallback(IClientCallback* callback)
	{
		mClientCallback = callback;
		return OYM_SUCCESS;
	}

	OYM_STATUS UnregisterClientCallback()
	{
		mClientCallback = NULL;
		return OYM_SUCCESS;
	}

	OYM_STATUS ConfigMtuSize(OYM_UINT16 conn_handle, OYM_UINT16 MTU_Size);
	OYM_STATUS ConnectionParameterUpdate(OYM_UINT16 conn_handle, OYM_UINT16 conn_interval_min, OYM_UINT16 conn_interval_max, OYM_UINT16 slave_latence, OYM_UINT16 supervision_timeout);
	OYM_STATUS WriteCharacteristic(OYM_UINT16 conn_handle, OYM_UINT16 attribute_handle, OYM_UINT8 data_length, OYM_PUINT8 data);
	OYM_STATUS ReadCharacteristic(OYM_UINT16 conn_handle, OYM_UINT16 attribute_handle);

	OYM_STATUS OnEvent(OYM_UINT32 event, OYM_PUINT8 data, OYM_UINT16 length);
	OYM_STATUS OnDeviceFound(BLE_DEVICE new_device);
private:
	OYM_STATUS ConnectComplete(OYM_UINT16 handle);
	OYM_STATUS ScanFinished();
	OYM_STATUS OnConnectEvent(OYM_PUINT8 data, OYM_UINT16 length);
	OYM_RemoteDevice* GetDeviceByHandle(OYM_UINT16 handle);
	OYM_STATUS mIsConnecting;
	OYM_STATUS mIsScanning;
	OYM_NPI_Interface* mInterface;
	OYM_Log* mLog;
	OYM_Discovery_Service* mDS;

	list<OYM_RemoteDevice*> mAvailabeDevice;
	list<OYM_RemoteDevice*> mConnectingDevice;
	list<OYM_RemoteDevice*> mConnectedDevice;

	IClientCallback* mClientCallback;
};
#endif
/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2026.
 * Non-connectable SLE advertiser for the RSSI scan sample.
 */
#include <string.h>
#include "common_def.h"
#include "securec.h"
#include "soc_osal.h"
#include "sle_common.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#include "sle_connection_manager.h"
#include "sle_ssap_server.h"
#include "app_init.h"
#include "return_service.h"

#define SLE_RSSI_SCAN_ADV_LOG          "[sle rssi scan adv]"
#define SLE_RSSI_SCAN_ADV_HANDLE       1
#define SLE_RSSI_SCAN_ADV_INTERVAL     0x0320U /* 100 ms: interval unit is 125 us */
#define SLE_RSSI_SCAN_ADV_DATA_MAX_LEN 32U
#define SLE_RSSI_SCAN_DEVICE_NAME      "R550_BC01"
#define SLE_RSSI_SCAN_ADV_HEADER_LEN   2U
#define RETURN_ADV_HANDLE 2U

static uint16_t return_property;
static uint8_t return_server;

static void receive_return(uint8_t server, uint16_t connection, ssaps_req_write_cb_t *write,
    errcode_t status)
{
    if (write == NULL || server != return_server) {
        return;
    }
    return_sample_t sample;
    bool accepted = status == ERRCODE_SLE_SUCCESS && write->handle == return_property &&
        return_decode(&sample, write->value, write->length);
    if (accepted) {
        osal_printk("+BCRX:%u,%u,%llu,%d\r\n", (unsigned)sample.session,
            (unsigned)sample.sequence, (unsigned long long)sample.received_us, (int)sample.rssi);
    }
    if (write->need_rsp) {
        ssaps_send_rsp_t response = {
            .request_id = write->request_id,
            .status = accepted ? 0 : (uint8_t)ERRCODE_SSAP_INVALID_PDU,
        };
        (void)ssaps_send_response(server, connection, &response);
    }
}

static errcode_t start_return_service(void)
{
    ssaps_callbacks_t callbacks = {.write_request_cb = receive_return};
    sle_uuid_t uuid = return_uuid(1);
    uint16_t handle = 0;
    static uint8_t initial_value[RETURN_WIRE_SIZE];
    ssaps_property_info_t property = {
        .uuid = return_uuid(2),
        .permissions = SSAP_PERMISSION_WRITE,
        .operate_indication = SSAP_OPERATE_INDICATION_BIT_WRITE,
        .value_len = sizeof(initial_value),
        .value = initial_value,
    };
    errcode_t rc = ssaps_register_callbacks(&callbacks);
    if (rc == ERRCODE_SLE_SUCCESS) {
        rc = ssaps_register_server(&uuid, &return_server);
    }
    if (rc == ERRCODE_SLE_SUCCESS) {
        ssap_exchange_info_t info = {.mtu_size = RETURN_MTU, .version = 1};
        rc = ssaps_set_info(return_server, &info);
    }
    if (rc == ERRCODE_SLE_SUCCESS) {
        rc = ssaps_add_service_sync(return_server, &uuid, true, &handle);
    }
    if (rc == ERRCODE_SLE_SUCCESS) {
        rc = ssaps_add_property_sync(return_server, handle, &property, &return_property);
    }
    if (rc == ERRCODE_SLE_SUCCESS) {
        rc = ssaps_start_service(return_server, handle);
    }
    return rc;
}

enum {
    SLE_RSSI_SCAN_AD_DISCOVERY_LEVEL = 0x01,
    SLE_RSSI_SCAN_AD_ACCESS_MODE = 0x02,
    SLE_RSSI_SCAN_AD_COMPLETE_LOCAL_NAME = 0x0B,
};

static uint16_t sle_rssi_scan_build_adv_data(uint8_t *data, uint16_t capacity, const char *device_name)
{
    const uint8_t name_len = (uint8_t)strlen(device_name);
    uint16_t index = 0;

    if ((data == NULL) || (capacity < (uint16_t)(8U + name_len))) {
        return 0;
    }

    /* Keep the two standard SLE fields used by the SDK samples. */
    data[index++] = 2;
    data[index++] = SLE_RSSI_SCAN_AD_DISCOVERY_LEVEL;
    data[index++] = SLE_ANNOUNCE_LEVEL_NORMAL;
    data[index++] = 2;
    data[index++] = SLE_RSSI_SCAN_AD_ACCESS_MODE;
    data[index++] = 0;

    /* A short name in the primary announcement lets passive scanners identify us. */
    data[index++] = name_len + 1U;
    data[index++] = SLE_RSSI_SCAN_AD_COMPLETE_LOCAL_NAME;
    if (memcpy_s(&data[index], capacity - index, device_name, name_len) != EOK) {
        return 0;
    }
    return (uint16_t)(index + name_len);
}

static errcode_t configure_advertising(bool uplink)
{
    sle_announce_param_t param = {0};
    sle_announce_data_t announce_data = {0};
    uint8_t data[SLE_RSSI_SCAN_ADV_DATA_MAX_LEN] = {0};
    uint8_t empty_seek_rsp_data = 0;
    const uint8_t local_addr[SLE_ADDR_LEN] = {0x52, 0x35, 0x35, 0x30, 0x00, 0x01};
    errcode_t ret;

    const char *name = uplink ? RETURN_ADV_NAME : SLE_RSSI_SCAN_DEVICE_NAME;
    param.announce_handle = uplink ? RETURN_ADV_HANDLE : SLE_RSSI_SCAN_ADV_HANDLE;
    param.announce_mode = uplink ? SLE_ANNOUNCE_MODE_CONNECTABLE_SCANABLE : SLE_ANNOUNCE_MODE_NONCONN_NONSCAN;
    param.announce_gt_role = uplink ? SLE_ANNOUNCE_ROLE_T_CAN_NEGO : SLE_ANNOUNCE_ROLE_T_NO_NEGO;
    param.announce_level = SLE_ANNOUNCE_LEVEL_NORMAL;
    param.announce_interval_min = SLE_RSSI_SCAN_ADV_INTERVAL;
    param.announce_interval_max = SLE_RSSI_SCAN_ADV_INTERVAL;
    param.announce_channel_map = 0x07;
    param.announce_tx_power = 0; /* Fixed 0 dBm setting; actual EIRP is hardware dependent. */
    param.own_addr.type = 0;
    if (memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, local_addr, SLE_ADDR_LEN) != EOK) {
        return ERRCODE_SLE_FAIL;
    }

    ret = sle_set_announce_param(param.announce_handle, &param);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s set announce parameters failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
        return ret;
    }

    announce_data.announce_data_len = sle_rssi_scan_build_adv_data(data, sizeof(data), name);
    if (announce_data.announce_data_len == 0) {
        osal_printk("%s build announce data failed\r\n", SLE_RSSI_SCAN_ADV_LOG);
        return ERRCODE_SLE_FAIL;
    }
    announce_data.announce_data = data;
    announce_data.seek_rsp_data_len = 0;
    announce_data.seek_rsp_data = &empty_seek_rsp_data;
    ret = sle_set_announce_data(param.announce_handle, &announce_data);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s set announce data failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
        return ret;
    }

    ret = sle_start_announce(param.announce_handle);
    osal_printk("%s announcement start, name=%s, status=0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, name, ret);
    return ret;
}

static void return_disconnected(uint16_t connection, const sle_addr_t *address,
    sle_acb_state_t state, sle_pair_state_t pair_state, sle_disc_reason_t reason)
{
    (void)connection;
    (void)address;
    (void)pair_state;
    (void)reason;
    if (state == SLE_ACB_STATE_DISCONNECTED && return_property != 0) {
        (void)sle_start_announce(RETURN_ADV_HANDLE);
    }
}

static void sle_rssi_scan_announce_enable_cb(uint32_t announce_id, errcode_t status)
{
    osal_printk("%s announcement enabled, id=%u, status=0x%x\r\n",
                SLE_RSSI_SCAN_ADV_LOG, (unsigned int)announce_id, status);
}

static void sle_rssi_scan_sle_enable_cb(errcode_t status)
{
    osal_printk("%s SLE enabled, status=0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, status);
    if (status == ERRCODE_SLE_SUCCESS) {
        (void)configure_advertising(false);
        sle_connection_callbacks_t callbacks = {.connect_state_changed_cb = return_disconnected};
        errcode_t rc = sle_connection_register_callbacks(&callbacks);
        if (rc == ERRCODE_SLE_SUCCESS) {
            rc = start_return_service();
        }
        if (rc == ERRCODE_SLE_SUCCESS) {
            rc = configure_advertising(true);
        }
        osal_printk("+BCUPBASE:%x\r\n", rc);
    }
}

errcode_t sle_rssi_scan_advertiser_init(void)
{
    sle_announce_seek_callbacks_t callbacks = {0};
    errcode_t ret;

    callbacks.sle_enable_cb = sle_rssi_scan_sle_enable_cb;
    callbacks.announce_enable_cb = sle_rssi_scan_announce_enable_cb;
    ret = sle_announce_seek_register_callbacks(&callbacks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s register callbacks failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
        return ret;
    }

    ret = enable_sle();
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s enable SLE failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
    }
    return ret;
}

static int fixed_beacon_task(void *arg)
{
    (void)arg;
    osal_msleep(1200);
    return sle_rssi_scan_advertiser_init() == ERRCODE_SLE_SUCCESS ? 0 : -1;
}
static void fixed_beacon_start(void) {
 osal_kthread_lock();osal_task *t=osal_kthread_create(fixed_beacon_task,NULL,"BeaconTx",0x1400);
 if(t!=NULL) { osal_kthread_set_priority(t,26); }
 osal_kthread_unlock();
}
app_run(fixed_beacon_start);

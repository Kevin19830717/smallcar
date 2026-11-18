/* SLE return channel. The existing UART survey consumes a separate queue. */
#include <string.h>
#include "soc_osal.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#include "sle_ssap_client.h"
#include "return_client.h"
#include "return_service.h"

enum { SEARCH, STOPPING_SCAN, CONNECT, CONNECTING, EXCHANGE, EXCHANGING,
       DISCOVER, DISCOVERING, READY, CLOSE, CLOSING };

#define QUEUE_CAPACITY 128
#define STEP_TIMEOUT_US 5000000ULL
#define RETRY_US 500000ULL

static struct {
    return_sample_t queue[QUEUE_CAPACITY];
    uint32_t head, tail, drops, confirmed;
    uint16_t connection, property;
    uint8_t client, state;
    bool found, scanning, pending;
    sle_addr_t peer;
    uint64_t deadline, retry, next_scan, write_deadline;
    uint8_t wire[RETURN_WIRE_SIZE];
} link;

void return_client_enqueue(const return_sample_t *sample)
{
    /* Called with the survey queue lock held. */
    if (link.head - link.tail == QUEUE_CAPACITY) {
        ++link.drops;
    } else {
        link.queue[link.head++ % QUEUE_CAPACITY] = *sample;
    }
}

void return_client_found(const sle_addr_t *address)
{
    osal_kthread_lock();
    if (link.state == SEARCH) {
        link.peer = *address;
        link.found = true;
    }
    osal_kthread_unlock();
}

void return_client_scan_started(errcode_t status)
{
    osal_kthread_lock();
    link.scanning = status == ERRCODE_SLE_SUCCESS;
    osal_kthread_unlock();
    if (status != ERRCODE_SLE_SUCCESS) {
        osal_printk("+BCUPSCAN:%x\r\n", status);
    }
}

void return_client_scan_stopped(errcode_t status)
{
    osal_kthread_lock();
    link.scanning = status != ERRCODE_SLE_SUCCESS;
    if (link.state == STOPPING_SCAN) {
        link.state = link.scanning ? SEARCH : CONNECT;
    }
    osal_kthread_unlock();
}

static void state_changed(uint16_t connection, const sle_addr_t *address, sle_acb_state_t state,
    sle_pair_state_t pair_state, sle_disc_reason_t reason)
{
    (void)pair_state;
    (void)reason;
    osal_kthread_lock();
    if (address != NULL && memcmp(address->addr, link.peer.addr, SLE_ADDR_LEN) == 0) {
        if (state == SLE_ACB_STATE_CONNECTED && link.state == CONNECTING) {
            link.connection = connection;
            link.state = EXCHANGE;
        } else if (state == SLE_ACB_STATE_DISCONNECTED) {
            link.state = SEARCH;
            link.property = 0;
            link.found = false;
            link.pending = false;
            link.scanning = false;
        }
    }
    osal_kthread_unlock();
}

static void exchanged(uint8_t client, uint16_t connection, ssap_exchange_info_t *info, errcode_t status)
{
    osal_kthread_lock();
    if (client == link.client && connection == link.connection && link.state == EXCHANGING) {
        link.state = status == ERRCODE_SLE_SUCCESS && info != NULL &&
            info->mtu_size >= RETURN_WIRE_SIZE + 3 ? DISCOVER : CLOSE;
    }
    osal_kthread_unlock();
}

static void found_property(uint8_t client, uint16_t connection, ssapc_find_property_result_t *property,
    errcode_t status)
{
    sle_uuid_t expected = return_uuid(2);
    osal_kthread_lock();
    if (client == link.client && connection == link.connection && link.state == DISCOVERING &&
        status == ERRCODE_SLE_SUCCESS && property != NULL && property->uuid.len == expected.len &&
        memcmp(property->uuid.uuid, expected.uuid, expected.len) == 0) {
        link.property = property->handle;
    }
    osal_kthread_unlock();
}

static void discovery_done(uint8_t client, uint16_t connection, ssapc_find_structure_result_t *result,
    errcode_t status)
{
    (void)result;
    osal_kthread_lock();
    if (client == link.client && connection == link.connection && link.state == DISCOVERING) {
        link.state = status == ERRCODE_SLE_SUCCESS && link.property != 0 ? READY : CLOSE;
    }
    osal_kthread_unlock();
}

static void write_done(uint8_t client, uint16_t connection, ssapc_write_result_t *result, errcode_t status)
{
    osal_kthread_lock();
    if (client == link.client && connection == link.connection && link.pending && link.state == READY) {
        if (status == ERRCODE_SLE_SUCCESS && (result == NULL || result->handle == link.property)) {
            ++link.tail;
            ++link.confirmed;
        } else {
            link.state = CLOSE;
        }
        link.pending = false;
    }
    osal_kthread_unlock();
}

errcode_t return_client_init(void)
{
    static sle_connection_callbacks_t connection = {.connect_state_changed_cb = state_changed};
    static ssapc_callbacks_t callbacks = {
        .exchange_info_cb = exchanged,
        .ssapc_find_property_cbk = found_property,
        .find_structure_cmp_cb = discovery_done,
        .write_cfm_cb = write_done,
    };
    errcode_t rc = sle_connection_register_callbacks(&connection);
    if (rc == ERRCODE_SLE_SUCCESS) {
        rc = ssapc_register_callbacks(&callbacks);
    }
    return rc;
}

errcode_t return_client_register(void)
{
    sle_uuid_t uuid = return_uuid(3);
    return ssapc_register_client(&uuid, &link.client);
}

void return_client_tick(uint64_t now)
{
    enum { NOTHING, START_SCAN, STOP_SCAN, OPEN_LINK, EXCHANGE_INFO, FIND_PROPERTY, SEND, DISCONNECT };
    int action = NOTHING;
    osal_kthread_lock();
    if (now >= link.deadline && (link.state == CONNECTING || link.state == EXCHANGING ||
        link.state == DISCOVERING || link.state == STOPPING_SCAN)) {
        link.state = CLOSE;
    } else if (link.state == CLOSING && now >= link.deadline) {
        link.state = SEARCH;
        link.found = false;
        link.scanning = false;
    }
    if (link.state == SEARCH && link.found && link.scanning && now >= link.retry) {
        link.state = STOPPING_SCAN;
        link.found = false;
        action = STOP_SCAN;
    } else if (link.state == CONNECT) {
        link.state = CONNECTING;
        action = OPEN_LINK;
    } else if (link.state == EXCHANGE) {
        link.state = EXCHANGING;
        action = EXCHANGE_INFO;
    } else if (link.state == DISCOVER) {
        link.state = DISCOVERING;
        action = FIND_PROPERTY;
    } else if (link.state == CLOSE || (link.pending && now >= link.write_deadline)) {
        link.pending = false;
        link.state = CLOSING;
        link.retry = now + RETRY_US;
        action = DISCONNECT;
    } else if ((link.state == SEARCH || link.state == EXCHANGING || link.state == DISCOVERING ||
        link.state == READY) && !link.scanning && now >= link.next_scan) {
        link.next_scan = now + RETRY_US;
        action = START_SCAN;
    } else if (link.state == READY && !link.pending && link.tail != link.head && now >= link.retry) {
        return_encode(link.wire, &link.queue[link.tail % QUEUE_CAPACITY]);
        link.pending = true;
        link.write_deadline = now + STEP_TIMEOUT_US;
        action = SEND;
    }
    if (action != NOTHING && action != START_SCAN && action != SEND) {
        link.deadline = now + STEP_TIMEOUT_US;
    }
    sle_addr_t peer = link.peer;
    uint16_t connection = link.connection;
    uint16_t property = link.property;
    osal_kthread_unlock();

    errcode_t rc = ERRCODE_SLE_SUCCESS;
    if (action == START_SCAN) {
        rc = sle_start_seek();
    } else if (action == STOP_SCAN) {
        rc = sle_stop_seek();
    } else if (action == OPEN_LINK) {
        rc = sle_connect_remote_device(&peer);
    } else if (action == EXCHANGE_INFO) {
        ssap_exchange_info_t info = {.mtu_size = RETURN_MTU, .version = 1};
        rc = ssapc_exchange_info_req(link.client, connection, &info);
    } else if (action == FIND_PROPERTY) {
        ssapc_find_structure_param_t find = {
            .type = SSAP_FIND_TYPE_PROPERTY, .start_hdl = 1, .end_hdl = 0xffff,
        };
        rc = ssapc_find_structure(link.client, connection, &find);
    } else if (action == SEND) {
        ssapc_write_param_t write = {
            .handle = property, .type = SSAP_PROPERTY_TYPE_VALUE,
            .data_len = RETURN_WIRE_SIZE, .data = link.wire,
        };
        rc = ssapc_write_req(link.client, connection, &write);
    } else if (action == DISCONNECT) {
        rc = sle_disconnect_remote_device(&peer);
    }
    if (rc != ERRCODE_SLE_SUCCESS && action != START_SCAN) {
        osal_kthread_lock();
        if (action == STOP_SCAN || action == OPEN_LINK || action == DISCONNECT) {
            link.state = SEARCH;
            link.found = false;
            link.retry = now + RETRY_US;
        } else if (action == SEND) {
            link.pending = false;
            link.retry = now + RETRY_US;
        } else {
            link.state = CLOSE;
        }
        osal_kthread_unlock();
        osal_printk("+BCUPERR:%d,%x\r\n", action, rc);
    }
}

void return_client_status(void)
{
    osal_kthread_lock();
    unsigned state = link.state;
    unsigned pending = link.head - link.tail;
    unsigned drops = link.drops;
    unsigned confirmed = link.confirmed;
    bool scanning = link.scanning;
    osal_kthread_unlock();
    osal_printk("+BCUP:%u,%u,%u,%u,%u\r\n", state, (unsigned)scanning, pending, drops, confirmed);
}

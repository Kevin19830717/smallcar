/* Preserve the passive survey output while also queuing samples for SLE return. */
#include <stdint.h>
#include <string.h>
#include "soc_osal.h"
#include "app_init.h"
#include "at.h"
#include "tcxo.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#include "security_unified/trng.h"
#include "return_client.h"
typedef struct { uint64_t us; uint32_t seq; int8_t rssi; } sample_t;
static sample_t samples[128];
static uint32_t head,tail,sequence,dropped,invalid;
static uint32_t return_session;
static bool return_ready;
static const uint8_t beacon_addr[6]={0x52,0x35,0x35,0x30,0x00,0x01};
static const char name[]="R550_BC01";
static int matches(const uint8_t *p,uint8_t n,const char *target)
{
    if (p==NULL) return 0;
    size_t target_len=strlen(target);
    for (unsigned i=0;i<n;) {
        unsigned len=p[i],next=i+len+1;
        if (!len || next>n) return 0;
        if (len==target_len+1 && p[i+1]==0x0b && !memcmp(p+i+2,target,target_len)) return 1;
        i=next;
    }
    return 0;
}
static void scan_result(sle_seek_result_info_t *r)
{
    if (!r || memcmp(r->addr.addr,beacon_addr,6)) return;
    if (return_ready && matches(r->data,r->data_length,RETURN_ADV_NAME)) {
        return_client_found(&r->addr);
        return;
    }
    if (!matches(r->data,r->data_length,name)) return;
    osal_kthread_lock();
    if (r->rssi==0x7f) {invalid++;osal_kthread_unlock();return;}
    sample_t s={uapi_tcxo_get_us(),++sequence,(int8_t)r->rssi};
    if (head-tail>=128) dropped++;
    else samples[head++%128]=s;
    if (return_ready) {
        return_sample_t uplink = {
            .session = return_session, .sequence = s.seq, .received_us = s.us, .rssi = s.rssi,
        };
        return_client_enqueue(&uplink);
    }
    osal_kthread_unlock();
}
static void enabled(errcode_t status)
{
    if (status!=ERRCODE_SLE_SUCCESS) return;
    sle_seek_param_t p={0};
    p.filter_duplicates=0;p.seek_filter_policy=SLE_SEEK_FILTER_ALLOW_ALL;
    p.seek_phys=SLE_SEEK_PHY_1M;p.seek_type[0]=SLE_SEEK_PASSIVE;
    p.seek_window[0]=100;p.seek_interval[0]=100;
    errcode_t scan_status = sle_set_seek_param(&p);
    if (scan_status != ERRCODE_SLE_SUCCESS) {
        osal_printk("+BCUPSCAN:%x\r\n", scan_status);
        return;
    }
    errcode_t rc = return_session != 0 ? return_client_register() : ERRCODE_SLE_FAIL;
    return_ready = rc == ERRCODE_SLE_SUCCESS;
    osal_printk("+BCUPCLIENT:%x,%u\r\n", rc, (unsigned)return_session);
    if (!return_ready) (void)sle_start_seek();
}
static at_ret_t hello(void)
{
    uapi_at_print("+BCHELLO:1,RX,1,0,100\r\n");return AT_RET_OK;
}
static const at_cmd_entry_t commands[]={{.name="BCHELLO",.cmd_id=0x7100,.cmd=hello}};
static int beacon_receiver_task(void *arg)
{
    (void)arg;osal_msleep(1200);
    if (uapi_at_cmd_table_register(commands,1,sizeof(uint32_t))!=0) return -1;
    if (uapi_drv_cipher_trng_get_random(&return_session) != ERRCODE_SLE_SUCCESS ||
        return_client_init() != ERRCODE_SLE_SUCCESS) {
        return_session = 0;
    }
    sle_announce_seek_callbacks_t c={0};c.sle_enable_cb=enabled;c.seek_result_cb=scan_result;
    c.seek_enable_cb=return_client_scan_started;c.seek_disable_cb=return_client_scan_stopped;
    if (sle_announce_seek_register_callbacks(&c)!=ERRCODE_SLE_SUCCESS) return -1;
    (void)enable_sle();uint64_t last=0;
    for (;;) {
        sample_t s;int have=0;
        osal_kthread_lock();
        if (tail!=head) {s=samples[tail++%128];have=1;}
        uint32_t lost=dropped,bad=invalid,count=sequence;
        osal_kthread_unlock();
        if (have) uapi_at_print("+BCRSSI:%llu,%u,1,%d,%u\r\n",s.us,s.seq,(int)s.rssi,lost);
        uint64_t now=uapi_tcxo_get_us();
        if (return_ready) return_client_tick(now);
        if (now-last>=1000000ULL) {
            uapi_at_print("+BCSTATUS:%llu,%u,%u,%u\r\n",now,count,lost,bad);
            if (return_ready) return_client_status();
            last=now;
        }
        osal_msleep(have?1:10);
    }
    return 0;
}
static void beacon_receiver_start(void)
{
    osal_kthread_lock();
    osal_task *t=osal_kthread_create(beacon_receiver_task,NULL,"BeaconRx",0x1800);
    if(t!=NULL) osal_kthread_set_priority(t,26);
    osal_kthread_unlock();
}
app_run(beacon_receiver_start);

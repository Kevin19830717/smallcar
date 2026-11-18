#ifndef R550_RETURN_CLIENT_H
#define R550_RETURN_CLIENT_H

#include "sle_device_discovery.h"
#include "return_wire.h"

errcode_t return_client_init(void);
errcode_t return_client_register(void);
void return_client_found(const sle_addr_t *address);
void return_client_scan_started(errcode_t status);
void return_client_scan_stopped(errcode_t status);
void return_client_enqueue(const return_sample_t *sample);
void return_client_tick(uint64_t now);
void return_client_status(void);
#endif

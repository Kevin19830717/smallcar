#ifndef R550_RETURN_SERVICE_H
#define R550_RETURN_SERVICE_H

#include "sle_common.h"
#include "return_wire.h"

static inline sle_uuid_t return_uuid(uint8_t kind)
{
    sle_uuid_t uuid = {.len = 16,
        .uuid = {kind, 0x52, 0x35, 0x35, 0x30, 0x6b, 0xd5, 0x48,
                 0x9c, 0xa4, 0x33, 0x0b, 0x78, 0xe4, 0x12, 0x67}};
    return uuid;
}
#endif

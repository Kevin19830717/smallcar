#ifndef R550_RETURN_WIRE_H
#define R550_RETURN_WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RETURN_ADV_NAME "R550_UP01"
#define RETURN_WIRE_SIZE 18
#define RETURN_VERSION 1
#define RETURN_MTU 64

typedef struct {
    uint32_t session;
    uint32_t sequence;
    uint64_t received_us;
    int8_t rssi;
} return_sample_t;

static inline void return_put_le(uint8_t *out, uint64_t value, unsigned size)
{
    for (unsigned i = 0; i < size; ++i) {
        out[i] = (uint8_t)(value >> (8 * i));
    }
}

static inline uint64_t return_get_le(const uint8_t *data, unsigned size)
{
    uint64_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        value |= (uint64_t)data[i] << (8 * i);
    }
    return value;
}

static inline void return_encode(uint8_t out[RETURN_WIRE_SIZE], const return_sample_t *sample)
{
    out[0] = RETURN_VERSION;
    return_put_le(out + 1, sample->session, 4);
    return_put_le(out + 5, sample->sequence, 4);
    return_put_le(out + 9, sample->received_us, 8);
    out[17] = (uint8_t)sample->rssi;
}

static inline bool return_decode(return_sample_t *sample, const uint8_t *data, size_t size)
{
    if (data == NULL || size != RETURN_WIRE_SIZE || data[0] != RETURN_VERSION) {
        return false;
    }
    const int rssi = data[17] < 128 ? data[17] : (int)data[17] - 256;
    sample->session = (uint32_t)return_get_le(data + 1, 4);
    sample->sequence = (uint32_t)return_get_le(data + 5, 4);
    sample->received_us = return_get_le(data + 9, 8);
    sample->rssi = (int8_t)rssi;
    return sample->session != 0 && sample->sequence != 0 && rssi >= -127 && rssi <= 20;
}
#endif

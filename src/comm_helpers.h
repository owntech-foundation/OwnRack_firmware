#include <arm_math.h>

#define GET_UPPER_12BITS(buf_3bytes) ((int16_t)((*(buf_3bytes + 2) << 4) + (*(buf_3bytes + 1) >> 4)))

#define GET_LOWER_12BITS(buf_3bytes) ((int16_t)(((*(buf_3bytes + 1) & 0xf) << 8) + (*(buf_3bytes) & 0xff)))

#define PUT_UPPER_12BITS(buf_3bytes, data)                                    \
    {                                                                         \
        *(buf_3bytes + 1) = (0xf0 & (data << 4)) + (0xf & *(buf_3bytes + 1)); \
        *(buf_3bytes + 2) = (data & 0xff0) >> 4;                              \
    }

#define PUT_LOWER_12BITS(buf_3bytes, data)                                      \
    {                                                                           \
        *buf_3bytes = data & 0xff;                                              \
        *(buf_3bytes + 1) = (0xf0 & *(buf_3bytes + 1)) + ((data & 0xf00) >> 8); \
    }

__inline__ int16_t to_12bits(float32_t data, float32_t scale, float32_t offset = 0)
{
    return (int16_t)((data + offset) * 4096.0 / (scale));
}

__inline__ float32_t from_12bits(int16_t data, float32_t scale, float32_t offset = 0)
{
    return (scale * (float32_t)data) / 4096.0 - offset;
}


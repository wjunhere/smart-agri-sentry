#include "cj702.h"
#include <string.h>

uint8_t cj702_checksum(const uint8_t *frame)
{
    uint16_t sum = 0;
    for (int i = 0; i < CJ702_FRAME_LEN - 1; i++) {
        sum += frame[i];
    }
    return (uint8_t)(sum & 0xFF);
}

bool cj702_parse(const uint8_t *frame, cj702_data_t *out)
{
    if (frame == NULL || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    if (frame[0] != CJ702_HEADER_0 || frame[1] != CJ702_HEADER_1) {
        return false;
    }
    if (cj702_checksum(frame) != frame[CJ702_FRAME_LEN - 1]) {
        return false;
    }

    out->co2      = ((uint16_t)frame[2] << 8) | frame[3];
    out->hcho     = ((uint16_t)frame[4] << 8) | frame[5];
    out->tvoc     = ((uint16_t)frame[6] << 8) | frame[7];
    out->pm25     = ((uint16_t)frame[8] << 8) | frame[9];
    out->pm10     = ((uint16_t)frame[10] << 8) | frame[11];

    uint8_t temp_int = frame[12];
    int sign = (temp_int & 0x80) ? -1 : 1;
    int mag  = temp_int & 0x7F;
    out->temp = (int16_t)(sign * (mag * 100 + frame[13]));

    out->humidity = ((uint16_t)frame[14] * 100) + frame[15];

    return true;
}

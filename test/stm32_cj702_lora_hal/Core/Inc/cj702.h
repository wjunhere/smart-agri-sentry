#ifndef __CJ702_H
#define __CJ702_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define CJ702_FRAME_LEN 17
#define CJ702_HEADER_0  0x3C
#define CJ702_HEADER_1  0x02

typedef struct {
    uint16_t co2;       // ppm
    uint16_t hcho;      // raw
    uint16_t tvoc;      // ppb
    uint16_t pm25;      // ug/m3
    uint16_t pm10;      // ug/m3
    int16_t  temp;      // 0.01 C
    uint16_t humidity;  // 0.01 %RH
} cj702_data_t;

bool cj702_parse(const uint8_t *frame, cj702_data_t *out);
uint8_t cj702_checksum(const uint8_t *frame);

#endif

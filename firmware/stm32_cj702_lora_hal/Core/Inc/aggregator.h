#ifndef __AGGREGATOR_H
#define __AGGREGATOR_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "cj702.h"

#define AGGREGATOR_MIN_SAMPLES 10

typedef struct {
    uint32_t co2_sum;
    uint32_t hcho_sum;
    uint32_t tvoc_sum;
    uint32_t pm25_sum;
    uint32_t pm10_sum;
    int32_t  temp_sum;
    uint32_t humidity_sum;
    int32_t  soil_temp_sum;
    uint32_t soil_humidity_sum;
    uint32_t ec_sum;
    uint32_t leaf_wetness_sum;
    int32_t  leaf_temp_sum;
    uint16_t count;
} aggregator_t;

void aggregator_init(aggregator_t *agg);
void aggregator_add(aggregator_t *agg, const cj702_data_t *sample);
bool aggregator_get_average(const aggregator_t *agg, cj702_data_t *out);

#endif

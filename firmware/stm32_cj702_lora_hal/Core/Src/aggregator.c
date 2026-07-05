#include "aggregator.h"
#include <string.h>

void aggregator_init(aggregator_t *agg)
{
    memset(agg, 0, sizeof(*agg));
}

void aggregator_add(aggregator_t *agg, const cj702_data_t *sample)
{
    agg->co2_sum            += sample->co2;
    agg->hcho_sum           += sample->hcho;
    agg->tvoc_sum           += sample->tvoc;
    agg->pm25_sum           += sample->pm25;
    agg->pm10_sum           += sample->pm10;
    agg->temp_sum           += sample->temp;
    agg->humidity_sum       += sample->humidity;
    agg->soil_temp_sum      += sample->soil_temp;
    agg->soil_humidity_sum  += sample->soil_humidity;
    agg->ec_sum             += sample->ec;
    agg->leaf_wetness_sum   += sample->leaf_wetness;
    agg->leaf_temp_sum      += sample->leaf_temp;
    agg->count++;
}

bool aggregator_get_average(const aggregator_t *agg, cj702_data_t *out)
{
    if (agg == NULL || out == NULL || agg->count < AGGREGATOR_MIN_SAMPLES) {
        return false;
    }
    out->co2            = (uint16_t)(agg->co2_sum / agg->count);
    out->hcho           = (uint16_t)(agg->hcho_sum / agg->count);
    out->tvoc           = (uint16_t)(agg->tvoc_sum / agg->count);
    out->pm25           = (uint16_t)(agg->pm25_sum / agg->count);
    out->pm10           = (uint16_t)(agg->pm10_sum / agg->count);
    out->temp           = (int16_t)(agg->temp_sum / agg->count);
    out->humidity       = (uint16_t)(agg->humidity_sum / agg->count);
    out->soil_temp      = (int16_t)(agg->soil_temp_sum / agg->count);
    out->soil_humidity  = (uint16_t)(agg->soil_humidity_sum / agg->count);
    out->ec             = (uint16_t)(agg->ec_sum / agg->count);
    out->leaf_wetness   = (uint16_t)(agg->leaf_wetness_sum / agg->count);
    out->leaf_temp      = (int16_t)(agg->leaf_temp_sum / agg->count);
    return true;
}

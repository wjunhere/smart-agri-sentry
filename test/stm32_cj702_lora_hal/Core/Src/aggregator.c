#include "aggregator.h"

void aggregator_init(aggregator_t *agg)
{
    agg->co2_sum = 0;
    agg->hcho_sum = 0;
    agg->tvoc_sum = 0;
    agg->pm25_sum = 0;
    agg->pm10_sum = 0;
    agg->temp_sum = 0;
    agg->humidity_sum = 0;
    agg->count = 0;
}

void aggregator_add(aggregator_t *agg, const cj702_data_t *sample)
{
    agg->co2_sum      += sample->co2;
    agg->hcho_sum     += sample->hcho;
    agg->tvoc_sum     += sample->tvoc;
    agg->pm25_sum     += sample->pm25;
    agg->pm10_sum     += sample->pm10;
    agg->temp_sum     += sample->temp;
    agg->humidity_sum += sample->humidity;
    agg->count++;
}

bool aggregator_get_average(const aggregator_t *agg, cj702_data_t *out)
{
    if (agg == NULL || out == NULL || agg->count < AGGREGATOR_MIN_SAMPLES) {
        return false;
    }
    out->co2      = (uint16_t)(agg->co2_sum / agg->count);
    out->hcho     = (uint16_t)(agg->hcho_sum / agg->count);
    out->tvoc     = (uint16_t)(agg->tvoc_sum / agg->count);
    out->pm25     = (uint16_t)(agg->pm25_sum / agg->count);
    out->pm10     = (uint16_t)(agg->pm10_sum / agg->count);
    out->temp     = (int16_t)(agg->temp_sum / agg->count);
    out->humidity = (uint16_t)(agg->humidity_sum / agg->count);
    return true;
}

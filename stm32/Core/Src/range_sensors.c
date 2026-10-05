#include "range_sensors.h"

#include <stddef.h>

void RangeSensors_FilterInit(AmrRangeMedianFilter *filter)
{
    size_t index;
    if (filter == NULL) {
        return;
    }
    for (index = 0U; index < AMR_RANGE_FILTER_SIZE; index++) {
        filter->samples[index] = AMR_RANGE_INVALID_MM;
    }
    filter->count = 0U;
    filter->next = 0U;
}

uint16_t RangeSensors_FilterUpdate(
    AmrRangeMedianFilter *filter,
    uint16_t distance_mm
)
{
    uint16_t sorted[AMR_RANGE_FILTER_SIZE];
    uint8_t index;
    uint8_t inner;
    if ((filter == NULL) || (distance_mm == AMR_RANGE_INVALID_MM)) {
        return AMR_RANGE_INVALID_MM;
    }
    filter->samples[filter->next] = distance_mm;
    filter->next = (uint8_t)((filter->next + 1U) % AMR_RANGE_FILTER_SIZE);
    if (filter->count < AMR_RANGE_FILTER_SIZE) {
        filter->count++;
    }
    for (index = 0U; index < filter->count; index++) {
        sorted[index] = filter->samples[index];
    }
    for (index = 1U; index < filter->count; index++) {
        uint16_t value = sorted[index];
        inner = index;
        while ((inner > 0U) && (sorted[inner - 1U] > value)) {
            sorted[inner] = sorted[inner - 1U];
            inner--;
        }
        sorted[inner] = value;
    }
    return sorted[filter->count / 2U];
}

uint16_t RangeSensors_UltrasonicPulseToMm(
    uint32_t echo_pulse_us,
    int16_t temperature_c_x10,
    uint16_t minimum_mm,
    uint16_t maximum_mm
)
{
    int32_t speed_mm_s;
    uint64_t distance_mm;
    if ((echo_pulse_us == 0U) || (minimum_mm > maximum_mm)) {
        return AMR_RANGE_INVALID_MM;
    }
    /* speed[m/s] = 331.3 + 0.606*T[C] */
    speed_mm_s = 331300 + ((606 * (int32_t)temperature_c_x10) / 10);
    if (speed_mm_s <= 0) {
        return AMR_RANGE_INVALID_MM;
    }
    distance_mm = ((uint64_t)echo_pulse_us * (uint32_t)speed_mm_s) / 2000000ULL;
    if ((distance_mm < minimum_mm) || (distance_mm > maximum_mm)) {
        return AMR_RANGE_INVALID_MM;
    }
    return (uint16_t)distance_mm;
}

uint16_t RangeSensors_SharpAdcToMm(
    uint16_t adc_count,
    const AmrSharpCalibrationPoint *table,
    size_t table_length
)
{
    size_t index;
    if ((table == NULL) || (table_length < 2U)) {
        return AMR_RANGE_INVALID_MM;
    }
    if ((adc_count < table[0].adc_count)
        || (adc_count > table[table_length - 1U].adc_count)) {
        return AMR_RANGE_INVALID_MM;
    }
    for (index = 1U; index < table_length; index++) {
        uint16_t x0 = table[index - 1U].adc_count;
        uint16_t x1 = table[index].adc_count;
        int32_t y0 = (int32_t)table[index - 1U].distance_mm;
        int32_t y1 = (int32_t)table[index].distance_mm;
        if ((adc_count <= x1) && (x1 > x0)) {
            int32_t numerator = (int32_t)(adc_count - x0) * (y1 - y0);
            int32_t distance = y0 + (numerator / (int32_t)(x1 - x0));
            if ((distance < 0) || (distance >= (int32_t)AMR_RANGE_INVALID_MM)) {
                return AMR_RANGE_INVALID_MM;
            }
            return (uint16_t)distance;
        }
    }
    return AMR_RANGE_INVALID_MM;
}

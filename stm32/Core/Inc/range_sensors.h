#ifndef RANGE_SENSORS_H
#define RANGE_SENSORS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AMR_RANGE_INVALID_MM UINT16_MAX
#define AMR_RANGE_FILTER_SIZE 5U

typedef struct {
    uint16_t samples[AMR_RANGE_FILTER_SIZE];
    uint8_t count;
    uint8_t next;
} AmrRangeMedianFilter;

typedef struct {
    uint16_t adc_count;
    uint16_t distance_mm;
} AmrSharpCalibrationPoint;

void RangeSensors_FilterInit(AmrRangeMedianFilter *filter);
uint16_t RangeSensors_FilterUpdate(
    AmrRangeMedianFilter *filter,
    uint16_t distance_mm
);

/* HC-SR04-class echo pulse conversion with temperature compensation. */
uint16_t RangeSensors_UltrasonicPulseToMm(
    uint32_t echo_pulse_us,
    int16_t temperature_c_x10,
    uint16_t minimum_mm,
    uint16_t maximum_mm
);

/* Piecewise-linear ADC calibration; points must be sorted by ADC count. */
uint16_t RangeSensors_SharpAdcToMm(
    uint16_t adc_count,
    const AmrSharpCalibrationPoint *table,
    size_t table_length
);

#endif

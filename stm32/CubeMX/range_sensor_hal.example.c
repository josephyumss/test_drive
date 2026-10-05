/*
 * Copy the relevant functions into the CubeMX application after assigning
 * the ultrasonic Input Capture timer and the two SHARP ADC channels.
 * Do not compile this example unchanged: calibration values are vehicle-specific.
 */

#include "board_io.h"
#include "range_sensors.h"

static AmrRangeMedianFilter ultrasonic_filter;
static AmrRangeMedianFilter sharp_left_filter;
static AmrRangeMedianFilter sharp_right_filter;
static volatile uint16_t ultrasonic_mm = AMR_RANGE_INVALID_MM;
static volatile uint16_t sharp_left_mm = AMR_RANGE_INVALID_MM;
static volatile uint16_t sharp_right_mm = AMR_RANGE_INVALID_MM;

/* Replace with measured ADC counts for the exact SHARP model and mounting. */
static const AmrSharpCalibrationPoint sharp_calibration[] = {
    /* ADC count must be increasing; distance may decrease as voltage rises. */
    {500U, 800U},
    {900U, 500U},
    {1500U, 300U},
    {2500U, 150U}
};

void BoardRangeSensors_Init(void)
{
    RangeSensors_FilterInit(&ultrasonic_filter);
    RangeSensors_FilterInit(&sharp_left_filter);
    RangeSensors_FilterInit(&sharp_right_filter);
}

/* Call from the TIM Input Capture callback after calculating pulse width in us. */
void BoardRangeSensors_OnUltrasonicEchoUs(uint32_t echo_us)
{
    uint16_t distance = RangeSensors_UltrasonicPulseToMm(
        echo_us,
        200, /* 20.0 C; replace with measured temperature if available. */
        20U,
        4000U
    );
    if (distance != AMR_RANGE_INVALID_MM) {
        ultrasonic_mm = RangeSensors_FilterUpdate(&ultrasonic_filter, distance);
    }
}

/* Call after ADC DMA supplies one sample for each configured SHARP channel. */
void BoardRangeSensors_OnSharpAdc(uint16_t left_adc, uint16_t right_adc)
{
    uint16_t left = RangeSensors_SharpAdcToMm(
        left_adc,
        sharp_calibration,
        sizeof(sharp_calibration) / sizeof(sharp_calibration[0])
    );
    uint16_t right = RangeSensors_SharpAdcToMm(
        right_adc,
        sharp_calibration,
        sizeof(sharp_calibration) / sizeof(sharp_calibration[0])
    );
    if (left != AMR_RANGE_INVALID_MM) {
        sharp_left_mm = RangeSensors_FilterUpdate(&sharp_left_filter, left);
    }
    if (right != AMR_RANGE_INVALID_MM) {
        sharp_right_mm = RangeSensors_FilterUpdate(&sharp_right_filter, right);
    }
}

uint16_t BoardIO_ReadUltrasonicFrontMm(void)
{
    return ultrasonic_mm;
}

uint16_t BoardIO_ReadSharpLeftMm(void)
{
    return sharp_left_mm;
}

uint16_t BoardIO_ReadSharpRightMm(void)
{
    return sharp_right_mm;
}

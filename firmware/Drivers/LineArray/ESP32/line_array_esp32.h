/**
 * @file    line_array_esp32.h
 * @brief   9-Channel Analog IR Sensor Array Driver for ESP32
 * @details 16x oversampling, EMA filter (alpha=0.7), spike rejection,
 *          NVS calibration, weighted centroid, junction detection.
 *
 * Project: FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL
 *
 * Per sensor: R1=220ohm (IR LED limiter) + R2=10kohm (pull-up) + C1=100nF (decoupling)
 *
 * GPIO map:
 *   S1(Left)  -> GPIO32 (ADC1_CH4)   S6 -> GPIO37 (ADC1_CH1)
 *   S2        -> GPIO33 (ADC1_CH5)   S7 -> GPIO38 (ADC1_CH2)
 *   S3        -> GPIO34 (ADC1_CH6)   S8 -> GPIO39 (ADC1_CH3)
 *   S4        -> GPIO35 (ADC1_CH7)   S9(Right)->GPIO25(ADC2_CH8)
 *   S5(Centre)-> GPIO36 (ADC1_CH0)
 *
 * ADC Logic: HIGH(~4095) = Black(off line), LOW(~0) = White(on line)
 * error: -4.0(left) to +4.0(right), 0.0=centred under S5
 *
 * UART to STM32 (115200, 100Hz): '$'LA,err_x100,active,junc,lost,crc\r\n
 */

#ifndef LINE_ARRAY_ESP32_H
#define LINE_ARRAY_ESP32_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LA_NUM_SENSORS              9
#define LA_CENTRE_INDEX             4.0f
#define LA_SENSOR_SPACING_MM        10.0f
#define LA_OVERSAMPLE_N             16
#define LA_EMA_ALPHA                0.70f
#define LA_SPIKE_THRESHOLD          500
#define LA_ACTIVE_THRESHOLD         0.50f
#define LA_CONTRIB_THRESHOLD        0.15f
#define LA_JUNCTION_MIN_SENSORS     6
#define LA_JUNCTION_CONSEC_SAMPLES  3
#define LA_LOST_TIMEOUT_MS          1500
#define LA_NVS_NAMESPACE            "line_array"
#define LA_NVS_KEY_MIN_FMT          "cal_min_%d"
#define LA_NVS_KEY_MAX_FMT          "cal_max_%d"
#define LA_CAL_DEFAULT_MIN          300
#define LA_CAL_DEFAULT_MAX          3800

typedef struct {
    float    raw_norm[LA_NUM_SENSORS];
    float    line_val[LA_NUM_SENSORS];
    uint16_t adc_raw[LA_NUM_SENSORS];
    int      active_count;
    float    centroid;
    float    error;
    float    error_mm;
    bool     is_junction;
    bool     is_line_lost;
    uint32_t lost_duration_ms;
} LA_Result_t;

typedef struct {
    uint16_t min_val[LA_NUM_SENSORS];
    uint16_t max_val[LA_NUM_SENSORS];
    bool     is_valid;
} LA_Calibration_t;

esp_err_t LA_Init(void);
esp_err_t LA_Calibrate(bool white_surface);
esp_err_t LA_Update(LA_Result_t *result);
void      LA_GetLastResult(LA_Result_t *result);
void      LA_Reset(void);
void      LA_PrintDebug(const LA_Result_t *result);
void      LA_GetCalibration(LA_Calibration_t *cal);

#ifdef __cplusplus
}
#endif
#endif /* LINE_ARRAY_ESP32_H */

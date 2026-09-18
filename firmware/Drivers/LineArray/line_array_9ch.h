/**
 * @file    line_array_9ch.h
 * @brief   9-Channel Analog IR Sensor Array Driver for STM32 (HAL + ADC DMA)
 * @details Uses STM32 12-bit ADC1 in Scan+Continuous+DMA-Circular mode,
 *          16x oversampling, EMA filter, spike rejection, Flash-based
 *          2-point calibration, weighted centroid, and junction detection.
 *
 * Supersedes: line_array.h (8-channel digital TCRT5000 driver)
 *
 * STM32CubeMX configuration required:
 *   ADC1 → Scan Conversion, Continuous, DMA Circular, 12-bit, 9 ranks
 *   Channels (rank order must match sensor index S1→S9):
 *     Rank 1 : PA0  (ADC1_IN0)  → S1 (Left)
 *     Rank 2 : PA1  (ADC1_IN1)  → S2
 *     Rank 3 : PA2  (ADC1_IN2)  → S3
 *     Rank 4 : PA3  (ADC1_IN3)  → S4
 *     Rank 5 : PA4  (ADC1_IN4)  → S5 (Centre)
 *     Rank 6 : PA5  (ADC1_IN5)  → S6
 *     Rank 7 : PA6  (ADC1_IN6)  → S7
 *     Rank 8 : PA7  (ADC1_IN7)  → S8
 *     Rank 9 : PB0  (ADC1_IN8)  → S9 (Right)
 *   DMA1 Stream0 Ch0 → Peripheral→Memory, Circular, Word width
 *   Sampling time per channel: 480 cycles (maximises SNR)
 *
 * Logic: ADC HIGH (near 4095) = Black surface = off line
 *        ADC LOW  (near 0)    = White surface = on line
 *
 * Centroid convention:
 *   error < 0  → line is LEFT  of centre → steer left
 *   error > 0  → line is RIGHT of centre → steer right
 *   error = 0  → perfectly centred (S5 on line)
 *   error = NAN → line lost
 */

#ifndef LINE_ARRAY_9CH_H
#define LINE_ARRAY_9CH_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>      /* NAN, isnan() */

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Configuration Constants ─────────────────────────────────────────────── */

#define LA_NUM_SENSORS              9       ///< Total sensors in array
#define LA_CENTRE_INDEX             4.0f    ///< Centre sensor = index 4 (S5)
#define LA_SENSOR_SPACING_MM        10.0f   ///< Physical spacing between sensors (mm)

/* Oversampling: DMA fills LA_DMA_BUFFER_SIZE raw values per update cycle */
#define LA_OVERSAMPLE_N             16      ///< ADC scans averaged per LA_Update() call
#define LA_DMA_BUFFER_SIZE          (LA_NUM_SENSORS * LA_OVERSAMPLE_N)  ///< 144 words

/* EMA filter alpha — 0.0 = frozen, 1.0 = raw */
#define LA_EMA_ALPHA                0.70f   ///< Recommended: 0.5–0.8

/* Spike rejection: max valid single-sample ADC change (0–4095 counts) */
#define LA_SPIKE_THRESHOLD          500

/* Line detection thresholds (normalised 0.0–1.0 after calibration) */
#define LA_ACTIVE_THRESHOLD         0.50f   ///< Sensor is "on line" if above this
#define LA_CONTRIB_THRESHOLD        0.15f   ///< Min value to contribute to centroid

/* Junction detection */
#define LA_JUNCTION_MIN_SENSORS     6       ///< Active sensors required to flag junction
#define LA_JUNCTION_CONSEC_SAMPLES  3       ///< Consecutive samples required (30 ms @ 100 Hz)

/* Line-lost */
#define LA_UPDATE_RATE_HZ           100     ///< Expected LA_Update() call rate
#define LA_LOST_TIMEOUT_MS          1500    ///< ms before LINE_LOST is declared

/* Calibration Flash storage */
#define LA_CAL_FLASH_ADDR           0x0807F800UL  ///< Last sector of STM32F401 (sector 7)
#define LA_CAL_FLASH_SECTOR         FLASH_SECTOR_7
#define LA_CAL_MAGIC                0xCAFEBEEFUL  ///< Magic word to validate stored cal

/* Default calibration (fallback when no Flash calibration found) */
#define LA_CAL_DEFAULT_MIN          300     ///< Estimated ADC counts over white surface
#define LA_CAL_DEFAULT_MAX          3800    ///< Estimated ADC counts over black surface

/* ─── Data Types ───────────────────────────────────────────────────────────── */

/**
 * @brief  Per-update processed output from LA_Update()
 */
typedef struct {
    float    raw_norm[LA_NUM_SENSORS];  ///< Normalised ADC: 0.0=white, 1.0=black
    float    line_val[LA_NUM_SENSORS];  ///< Inverted: 1.0=on line, 0.0=off line
    uint16_t adc_raw[LA_NUM_SENSORS];   ///< Raw 12-bit averaged ADC values (debug)
    int      active_count;              ///< Sensors above LA_ACTIVE_THRESHOLD
    float    centroid;                  ///< Position index 0.0–8.0 (NAN if lost)
    float    error;                     ///< centroid − 4.0  range [−4.0, +4.0]
    float    error_mm;                  ///< error × LA_SENSOR_SPACING_MM (mm)
    bool     is_junction;               ///< Intersection/junction confirmed
    bool     is_line_lost;              ///< No sensor sees the line
    uint32_t lost_duration_ms;          ///< How long line has been lost (ms)
} LA_Result_t;

/**
 * @brief  Per-sensor calibration data (stored in and loaded from Flash)
 */
typedef struct {
    uint32_t magic;                       ///< Must equal LA_CAL_MAGIC to be valid
    uint16_t min_val[LA_NUM_SENSORS];     ///< ADC counts measured over white surface
    uint16_t max_val[LA_NUM_SENSORS];     ///< ADC counts measured over black surface
    bool     is_valid;                    ///< True after successful calibration
    uint8_t  _pad[3];                     ///< Padding for 4-byte alignment
} LA_Calibration_t;

/* ─── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the 9-channel analog sensor array driver.
 *         Registers the ADC handle, starts DMA in circular mode, and loads
 *         calibration from Flash (falls back to defaults if not found).
 *         Call once from main() AFTER MX_ADC1_Init() and MX_DMA_Init().
 *
 * @param  hadc  Pointer to the STM32 HAL ADC1 handle (must be configured in
 *               Scan + Continuous + DMA Circular mode, 9 channels, 12-bit).
 */
void LA_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief  Perform one calibration step. Call twice:
 *         1. Robot over WHITE surface → LA_Calibrate(true)
 *         2. Robot over BLACK surface → LA_Calibrate(false)
 *         After the BLACK step the calibration is validated and written to Flash.
 *
 * @param  white_surface  true = currently over white; false = currently over black
 */
void LA_Calibrate(bool white_surface);

/**
 * @brief  Process the latest DMA ADC buffer into LA_Result_t.
 *         Applies oversampling, EMA filter, spike rejection, normalisation,
 *         centroid computation, and junction detection.
 *         Call every 10 ms (100 Hz) from a FreeRTOS task or timer callback.
 *
 * @param  result  Pointer to output structure filled by this function
 */
void LA_Update(LA_Result_t *result);

/**
 * @brief  Reset internal junction counter and line-lost counter.
 *         Call when re-entering a line-following state after a pause.
 */
void LA_Reset(void);

/**
 * @brief  Print sensor state over UART as a human-readable ASCII bar chart.
 *         Use at 5–10 Hz for debugging; not during competition.
 *
 * @param  result  Pointer to most recent LA_Result_t
 * @param  huart   STM32 UART handle for debug output (e.g., &huart2)
 */
void LA_PrintDebug(const LA_Result_t *result, UART_HandleTypeDef *huart);

/**
 * @brief  Copy current calibration data to caller's struct.
 * @param  cal  Output calibration struct
 */
void LA_GetCalibration(LA_Calibration_t *cal);

/* ─── PD Line Following Controller ────────────────────────────────────────── */

/**
 * @brief  PD controller state. One instance per robot.
 *         Initialise with LF_Init() before first use.
 */
typedef struct {
    float kp;           ///< Proportional gain
    float kd;           ///< Derivative gain
    float wz_max;       ///< Output clamp — max angular velocity (rad/s or normalised)
    float dt;           ///< Sample period (s) — must match LA_Update() call rate
    float prev_error;   ///< Previous error for derivative term
    bool  has_prev;     ///< True after first LF_ComputeOmega() call
} LF_Controller_t;

/* Recommended starting gains (tune empirically per robot) */
#define LF_KP_DEFAULT       0.60f   ///< Proportional gain starting point
#define LF_KD_DEFAULT       0.08f   ///< Derivative gain starting point
#define LF_WZ_MAX_DEFAULT   1.20f   ///< Max angular velocity (rad/s)
#define LF_DT_DEFAULT       0.01f   ///< 100 Hz = 10 ms period

/**
 * @brief  Initialise PD controller with given gains.
 * @param  lf      Controller struct to initialise
 * @param  kp      Proportional gain (start: LF_KP_DEFAULT = 0.60)
 * @param  kd      Derivative gain   (start: LF_KD_DEFAULT = 0.08)
 * @param  wz_max  Output clamp      (start: LF_WZ_MAX_DEFAULT = 1.20 rad/s)
 * @param  dt      Sample period     (LF_DT_DEFAULT = 0.01 for 100 Hz)
 */
void  LF_Init(LF_Controller_t *lf, float kp, float kd, float wz_max, float dt);

/**
 * @brief  Compute steering angular velocity from centroid error.
 *         PD formula: omega = Kp × error + Kd × (Δerror / dt)
 *         Output is clamped to [-wz_max, +wz_max].
 *
 * @param  lf     Controller state (must be initialised with LF_Init)
 * @param  error  Centroid error from LA_Result_t.error (NAN if line lost)
 * @retval omega  Angular velocity; 0.0 if line lost (NAN input)
 */
float LF_ComputeOmega(LF_Controller_t *lf, float error);

/**
 * @brief  Reset derivative state. Call after line lost or motion pause.
 * @param  lf  Controller to reset
 */
void  LF_Reset(LF_Controller_t *lf);

#ifdef __cplusplus
}
#endif

#endif /* LINE_ARRAY_9CH_H */

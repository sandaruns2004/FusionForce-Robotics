/**
 * @file    line_array.h
 * @brief   8-Channel TCRT5000 IR Line Sensor Array Driver for STM32 HAL
 * @details Reads 8 digital GPIO inputs (PA0–PA7) simultaneously.
 *          Provides weighted centroid, intersection detection, and line-lost detection.
 *
 * Wiring:
 *   Sensor S1 OUT -> PA0   (Leftmost)
 *   Sensor S2 OUT -> PA1
 *   Sensor S3 OUT -> PA2
 *   Sensor S4 OUT -> PA3
 *   Sensor S5 OUT -> PA4
 *   Sensor S6 OUT -> PA5
 *   Sensor S7 OUT -> PA6
 *   Sensor S8 OUT -> PA7   (Rightmost)
 *   VCC           -> 3.3V  (verify module VCC rating)
 *   GND           -> GND
 *
 * Logic: HIGH (1) = white/reflective surface detected
 *        LOW  (0) = black/absorptive surface
 *
 * Intersection detection uses a temporal filter:
 *   Requires LINEARRAY_INTERSECTION_THRESHOLD sensors active
 *   for LINEARRAY_INTERSECTION_CONSEC consecutive 20ms samples.
 */

#ifndef LINE_ARRAY_H
#define LINE_ARRAY_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>       /* NAN */

/* ─── Configuration ──────────────────────────────────────────────────────── */
#define LINEARRAY_NUM_SENSORS               8
#define LINEARRAY_INTERSECTION_THRESHOLD    6   /* Min active sensors for junction */
#define LINEARRAY_INTERSECTION_CONSEC       3   /* Consecutive cycles required     */
#define LINEARRAY_LOST_TIMEOUT_CYCLES       150 /* 150 × 20ms = 3 seconds          */
#define LINEARRAY_CENTER_INDEX              3.5f/* Centre of 0..7 index range      */

/* ─── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise GPIO PA0–PA7 as digital inputs with pull-down resistors.
 *         Call once during system init.
 */
void LineArray_Init(void);

/**
 * @brief  Read all 8 sensor outputs in one GPIO register access.
 * @retval uint8_t bitmask: bit0=S1(left)...bit7=S8(right). 1=white, 0=black.
 */
uint8_t LineArray_Read(void);

/**
 * @brief  Count the number of active (HIGH) sensors.
 * @param  bits  Raw bitmask from LineArray_Read()
 * @retval Number of sensors reading white (0–8)
 */
uint8_t LineArray_ActiveCount(uint8_t bits);

/**
 * @brief  Compute the weighted centroid of active sensors.
 * @details centroid = Σ(i × val[i]) / Σ(val[i]),  i = 0..7
 *          error = centroid – 3.5  (negative = line left, positive = line right)
 * @param  bits  Raw bitmask from LineArray_Read()
 * @retval float error in range [–3.5, +3.5], or NAN if all sensors read 0.
 */
float LineArray_GetCentroid(uint8_t bits);

/**
 * @brief  Detect intersection or T/+ junction using temporal filter.
 * @details Call every 20ms. Returns true when ≥LINEARRAY_INTERSECTION_THRESHOLD
 *          sensors have been active for ≥LINEARRAY_INTERSECTION_CONSEC consecutive
 *          calls. Resets counter when condition is no longer met.
 * @param  bits  Raw bitmask from LineArray_Read()
 * @retval true if intersection confirmed, false otherwise
 */
bool LineArray_IsIntersection(uint8_t bits);

/**
 * @brief  Check if all sensors read black (line lost).
 * @param  bits  Raw bitmask from LineArray_Read()
 * @retval true if all 8 sensors read 0
 */
bool LineArray_IsAllBlack(uint8_t bits);

/**
 * @brief  Track time since last line detection. Call every 20ms.
 * @details Increments an internal counter when all sensors are black.
 *          Resets counter when any sensor reads white.
 * @param  bits  Raw bitmask from LineArray_Read()
 * @retval Number of consecutive all-black cycles (multiply by 20ms for seconds)
 */
uint32_t LineArray_GetLostCycles(uint8_t bits);

/**
 * @brief  Reset all internal state (intersection counter, lost counter).
 *         Call when re-entering a line-following state.
 */
void LineArray_Reset(void);

#endif /* LINE_ARRAY_H */

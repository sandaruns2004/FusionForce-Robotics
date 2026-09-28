/**
 * @file    sensor_task.h
 * @brief   Sensor task public API (ESP32 line array co-processor)
 * @details FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL
 */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>

/**
 * @brief  Check if calibration was requested at boot (hold BOOT button 3s).
 * @retval true if calibration should run, false for normal operation
 */
bool sensor_task_check_calibration_request(void);

/**
 * @brief  Blocking calibration routine. Never returns.
 *         Blinks onboard LED when complete.
 */
void sensor_task_run_calibration(void);

/**
 * @brief  Main 100Hz FreeRTOS sensor task.
 *         Reads 9 analog sensors, computes centroid, streams via UART.
 * @param  arg  Unused (pass NULL when creating task)
 */
void sensor_task(void *arg);

#endif /* SENSOR_TASK_H */

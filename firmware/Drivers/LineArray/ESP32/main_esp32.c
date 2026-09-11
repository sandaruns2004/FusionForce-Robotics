/**
 * @file    main_esp32.c
 * @brief   ESP32 entry point -- Line Array Sensor Co-Processor
 * @details FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL
 *
 * Startup:
 *   1. NVS init
 *   2. Check BOOT button (GPIO0) -- hold 3s for calibration mode
 *   3. Launch 100Hz sensor_task on Core 0 (PRO_CPU)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sensor_task.h"

static const char *TAG = "Main";

void app_main(void) {
    ESP_LOGI(TAG, "FusionForce RUNNER-4 -- Line Array Co-Processor");
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);

    /* Initialise NVS (required for calibration persistence) */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erased and re-inited");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Check if calibration mode requested */
    if (sensor_task_check_calibration_request()) {
        sensor_task_run_calibration(); /* never returns */
    }

    /* Normal mode: start 100Hz sensor task pinned to Core 0 */
    ESP_LOGI(TAG, "Starting 100Hz sensor task on Core 0...");
    xTaskCreatePinnedToCore(
        sensor_task,     /* task function */
        "sensor_task",   /* name */
        4096,            /* stack bytes */
        NULL,            /* arg */
        10,              /* priority (high, below interrupts) */
        NULL,            /* handle (not needed) */
        0                /* core 0 = PRO_CPU */
    );

    ESP_LOGI(TAG, "System running. Streaming at 100Hz to STM32.");
}

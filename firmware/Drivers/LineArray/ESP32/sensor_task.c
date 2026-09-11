/**
 * @file    sensor_task.c
 * @brief   FreeRTOS task: 100Hz line array update, UART stream to STM32
 *
 * Project: FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL
 *
 * UART packet format (to STM32 at 115200 baud, 100Hz):
 *   '$'LA,<err_x100>,<active>,<junc>,<lost>,<crc8>\r\n
 *
 *   err_x100  : int16, error*100  (e.g. -1.25 -> -125, line lost -> -32768)
 *   active    : int, sensors above threshold (0-9)
 *   junc      : 0 or 1
 *   lost      : 0 or 1
 *   crc8      : CRC-8/MAXIM of all chars after '$' up to (not including) the ','crc field
 *
 * Calibration mode (at power-on):
 *   Hold BOOT button (GPIO0) for 3 seconds -> enters calibration routine
 */

#include "sensor_task.h"
#include "line_array_esp32.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "SensorTask";

/* UART configuration */
#define ST_UART_PORT    UART_NUM_1
#define ST_UART_TX      17      /* GPIO17 -> STM32 RX */
#define ST_UART_RX      16      /* GPIO16 <- STM32 TX */
#define ST_UART_BAUD    115200

/* Task configuration */
#define ST_RATE_HZ      100
#define ST_PERIOD_MS    (1000 / ST_RATE_HZ)   /* 10ms */
#define ST_DBG_EVERY    20                     /* debug print at 5Hz */

/* Calibration button */
#define CAL_BTN_GPIO    0       /* BOOT button, active LOW */
#define CAL_HOLD_MS     3000

/* CRC-8/MAXIM polynomial 0x07 */
static uint8_t crc8_calc(const uint8_t *d, int len) {
    uint8_t crc = 0xFF;
    while (len--) {
        crc ^= *d++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
}

static void uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate  = ST_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(ST_UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(ST_UART_PORT, &cfg);
    uart_set_pin(ST_UART_PORT, ST_UART_TX, ST_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "UART1 init: %d baud, TX=GPIO%d, RX=GPIO%d", ST_UART_BAUD, ST_UART_TX, ST_UART_RX);
}

/* Check BOOT button at startup -- returns true if held for CAL_HOLD_MS */
bool sensor_task_check_calibration_request(void) {
    gpio_set_direction(CAL_BTN_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CAL_BTN_GPIO, GPIO_PULLUP_ONLY);
    if (gpio_get_level(CAL_BTN_GPIO) == 0) {
        ESP_LOGI(TAG, "BOOT held -- calibration in %dms...", CAL_HOLD_MS);
        vTaskDelay(pdMS_TO_TICKS(CAL_HOLD_MS));
        if (gpio_get_level(CAL_BTN_GPIO) == 0) return true;
    }
    return false;
}

/* Full calibration routine (blocking) */
void sensor_task_run_calibration(void) {
    LA_Init();
    ESP_LOGI(TAG, "=== CALIBRATION MODE ===");
    ESP_LOGI(TAG, "Place ALL sensors over WHITE. Waiting 3s...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Sampling white...");
    LA_Calibrate(true);
    ESP_LOGI(TAG, "Done. Place ALL sensors over BLACK. Waiting 3s...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Sampling black...");
    LA_Calibrate(false);

    LA_Calibration_t cal;
    LA_GetCalibration(&cal);
    for (int i = 0; i < LA_NUM_SENSORS; i++)
        ESP_LOGI(TAG, "S%d: white=%d black=%d range=%d", i+1, cal.min_val[i], cal.max_val[i], cal.max_val[i]-cal.min_val[i]);

    ESP_LOGI(TAG, "Calibration DONE. Reboot for normal operation.");
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(2, 1); vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(2, 0); vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* Main 100Hz sensor task */
void sensor_task(void *arg) {
    ESP_LOGI(TAG, "Sensor task started (%d Hz)", ST_RATE_HZ);
    uart_init();

    if (LA_Init() != ESP_OK) {
        ESP_LOGE(TAG, "LA_Init failed -- task abort");
        vTaskDelete(NULL); return;
    }

    LA_Result_t res;
    int dbg = 0;
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        /* ---- Read & process sensors ------------------------------------ */
        LA_Update(&res);

        /* ---- Build UART packet ----------------------------------------- */
        /* Format: ,err_x100,active,junc,lost,CRC\r\n */
        char body[48];
        int16_t err_enc = isnan(res.error) ? INT16_MIN : (int16_t)(res.error * 100.0f);
        int body_len = snprintf(body, sizeof(body), "LA,%d,%d,%d,%d",
                                err_enc, res.active_count,
                                res.is_junction ? 1 : 0,
                                res.is_line_lost ? 1 : 0);

        uint8_t crc = crc8_calc((uint8_t*)body, body_len);

        char pkt[64];
        int pkt_len = snprintf(pkt, sizeof(pkt), "$%s,%02X\r\n", body, crc);
        uart_write_bytes(ST_UART_PORT, pkt, pkt_len);

        /* ---- Debug print at 5Hz --------------------------------------- */
        if (++dbg >= ST_DBG_EVERY) { LA_PrintDebug(&res); dbg = 0; }

        /* ---- Maintain precise 10ms period ----------------------------- */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ST_PERIOD_MS));
    }
}

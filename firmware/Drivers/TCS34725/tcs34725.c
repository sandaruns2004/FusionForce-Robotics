/**
 * @file    tcs34725.c
 * @brief   TCS34725 RGBC Colour Sensor Driver Implementation
 */

#include "tcs34725.h"
#include "main.h"           /* For LED GPIO definition */

/* ─── LED Control pin (PC0) ──────────────────────────────────────────────── */
#define TCS_LED_GPIO_PORT   GPIOC
#define TCS_LED_GPIO_PIN    GPIO_PIN_0

/* ─── Internal helpers ───────────────────────────────────────────────────── */
static HAL_StatusTypeDef _WriteReg(TCS34725_Handle_t *hdev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { TCS34725_COMMAND_BIT | reg, value };
    return HAL_I2C_Master_Transmit(hdev->hi2c, TCS34725_I2C_ADDR, buf, 2, 10);
}

static HAL_StatusTypeDef _ReadReg(TCS34725_Handle_t *hdev, uint8_t reg,
                                   uint8_t *data, uint16_t len) {
    uint8_t cmd = TCS34725_COMMAND_BIT | 0x20 | reg;  /* auto-increment bit */
    if (HAL_I2C_Master_Transmit(hdev->hi2c, TCS34725_I2C_ADDR, &cmd, 1, 10) != HAL_OK)
        return HAL_ERROR;
    return HAL_I2C_Master_Receive(hdev->hi2c, TCS34725_I2C_ADDR, data, len, 10);
}

/* ─── Public Functions ───────────────────────────────────────────────────── */

HAL_StatusTypeDef TCS34725_Init(TCS34725_Handle_t *hdev,
                                 I2C_HandleTypeDef *hi2c,
                                 uint8_t atime,
                                 uint8_t gain) {
    hdev->hi2c = hi2c;
    hdev->integration_started = false;
    hdev->cycle_counter = 0;
    hdev->last_color = COLOR_UNKNOWN;

    /* Power on */
    if (_WriteReg(hdev, TCS34725_ENABLE, TCS34725_ENABLE_PON) != HAL_OK) return HAL_ERROR;
    HAL_Delay(3);   /* 2.4ms typ power-on time */

    /* Enable ADC */
    if (_WriteReg(hdev, TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN) != HAL_OK)
        return HAL_ERROR;

    /* Set integration time and gain */
    if (_WriteReg(hdev, TCS34725_ATIME, atime) != HAL_OK) return HAL_ERROR;
    if (_WriteReg(hdev, TCS34725_CONTROL, gain) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef TCS34725_StartIntegration(TCS34725_Handle_t *hdev) {
    hdev->cycle_counter = 0;
    hdev->integration_started = true;
    /* AEN already enabled in Init; sensor is continuously integrating.
       We just reset our cycle counter to know when to read. */
    return HAL_OK;
}

bool TCS34725_PollNonBlocking(TCS34725_Handle_t *hdev, ColorID_t *out) {
    if (!hdev->integration_started) {
        TCS34725_StartIntegration(hdev);
        return false;
    }

    hdev->cycle_counter++;

    /* 50ms integration = 2.5 × 20ms cycles → wait 3 cycles to be safe */
    if (hdev->cycle_counter < 3) return false;

    /* Check AVALID bit */
    uint8_t status;
    if (_ReadReg(hdev, TCS34725_STATUS, &status, 1) != HAL_OK) return false;
    if (!(status & TCS34725_STATUS_AVALID)) return false;

    /* Read 8 bytes: C_L, C_H, R_L, R_H, G_L, G_H, B_L, B_H */
    uint8_t raw[8];
    if (_ReadReg(hdev, TCS34725_CDATAL, raw, 8) != HAL_OK) return false;

    hdev->last_raw.c = (uint16_t)(raw[0] | (raw[1] << 8));
    hdev->last_raw.r = (uint16_t)(raw[2] | (raw[3] << 8));
    hdev->last_raw.g = (uint16_t)(raw[4] | (raw[5] << 8));
    hdev->last_raw.b = (uint16_t)(raw[6] | (raw[7] << 8));

    hdev->last_color = TCS34725_ClassifyColor(&hdev->last_raw);
    if (out) *out = hdev->last_color;

    /* Reset for next reading */
    hdev->integration_started = false;
    hdev->cycle_counter = 0;

    return true;
}

HAL_StatusTypeDef TCS34725_ReadRGBC_Blocking(TCS34725_Handle_t *hdev,
                                              TCS34725_RGBC_t *out) {
    /* Wait for AVALID */
    uint8_t status = 0;
    uint32_t timeout = HAL_GetTick() + 700;
    while (!(status & TCS34725_STATUS_AVALID)) {
        if (HAL_GetTick() > timeout) return HAL_TIMEOUT;
        if (_ReadReg(hdev, TCS34725_STATUS, &status, 1) != HAL_OK) return HAL_ERROR;
        HAL_Delay(10);
    }

    uint8_t raw[8];
    if (_ReadReg(hdev, TCS34725_CDATAL, raw, 8) != HAL_OK) return HAL_ERROR;

    out->c = (uint16_t)(raw[0] | (raw[1] << 8));
    out->r = (uint16_t)(raw[2] | (raw[3] << 8));
    out->g = (uint16_t)(raw[4] | (raw[5] << 8));
    out->b = (uint16_t)(raw[6] | (raw[7] << 8));

    return HAL_OK;
}

ColorID_t TCS34725_ClassifyColor(const TCS34725_RGBC_t *raw) {
    if (raw->c == 0) return COLOR_UNKNOWN;

    float r_n = (float)raw->r / raw->c;
    float g_n = (float)raw->g / raw->c;
    float b_n = (float)raw->b / raw->c;

    /* RED: high red ratio, clearly dominates green and blue */
    if (r_n > 0.40f && r_n > g_n * 1.4f && r_n > b_n * 1.4f)
        return COLOR_RED;

    /* GREEN: high green ratio, clearly dominates */
    if (g_n > 0.35f && g_n > r_n * 1.2f && g_n > b_n * 1.2f)
        return COLOR_GREEN;

    /* BLUE: high blue ratio, clearly dominates */
    if (b_n > 0.30f && b_n > r_n * 1.2f && b_n > g_n * 1.2f)
        return COLOR_BLUE;

    return COLOR_UNKNOWN;
    /* NOTE: Thresholds (0.40, 1.4×, etc.) must be tuned during
       2-minute pre-competition calibration under arena lighting. */
}

void TCS34725_SetLED(bool on) {
    HAL_GPIO_WritePin(TCS_LED_GPIO_PORT, TCS_LED_GPIO_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

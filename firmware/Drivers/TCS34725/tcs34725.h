/**
 * @file    tcs34725.h
 * @brief   TCS34725 RGBC Colour Sensor Driver for STM32 HAL
 * @details I2C1 interface, address 0x29. Dual-mode: ball colour reading
 *          (arm 0 deg) and floor zone reading (arm -70 deg).
 *
 * Wiring:
 *   VCC  -> STM32 3.3V
 *   GND  -> GND
 *   SCL  -> PB6 (I2C1)
 *   SDA  -> PB7 (I2C1)
 *   LED  -> PC0 (GPIO output, HIGH = LED ON)
 */

#ifndef TCS34725_H
#define TCS34725_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ─── I2C Address ─────────────────────────────────────────────────────────── */
#define TCS34725_I2C_ADDR       (0x29 << 1)     /* HAL uses 8-bit addresses   */

/* ─── Register Map ───────────────────────────────────────────────────────── */
#define TCS34725_COMMAND_BIT    0x80
#define TCS34725_ENABLE         0x00
#define TCS34725_ATIME          0x01            /* Integration time            */
#define TCS34725_CONTROL        0x0F            /* Gain control                */
#define TCS34725_STATUS         0x13
#define TCS34725_CDATAL         0x14            /* Clear channel data low      */
#define TCS34725_RDATAL         0x16            /* Red channel data low        */
#define TCS34725_GDATAL         0x18            /* Green channel data low      */
#define TCS34725_BDATAL         0x1A            /* Blue channel data low       */

/* ENABLE register bits */
#define TCS34725_ENABLE_PON     0x01            /* Power ON                    */
#define TCS34725_ENABLE_AEN     0x02            /* ADC Enable                  */

/* STATUS register bits */
#define TCS34725_STATUS_AVALID  0x01            /* ADC valid (integration done)*/

/* Integration time settings (ATIME register) */
#define TCS34725_INTEGRATIONTIME_50MS   0xEB    /* 50ms — recommended          */
#define TCS34725_INTEGRATIONTIME_101MS  0xD6    /* 101ms — more accurate       */
#define TCS34725_INTEGRATIONTIME_700MS  0x00    /* 700ms — max accuracy        */

/* Gain settings (CONTROL register) */
#define TCS34725_GAIN_1X        0x00
#define TCS34725_GAIN_4X        0x01            /* Recommended for indoor      */
#define TCS34725_GAIN_16X       0x02            /* For dim lighting            */
#define TCS34725_GAIN_60X       0x03

/* ─── Colour Result Enum ─────────────────────────────────────────────────── */
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_RED     = 1,
    COLOR_GREEN   = 2,
    COLOR_BLUE    = 3
} ColorID_t;

/* ─── Raw RGBC data ──────────────────────────────────────────────────────── */
typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;     /* Clear (luminance) channel */
} TCS34725_RGBC_t;

/* ─── Driver State ───────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    bool     integration_started;
    uint8_t  cycle_counter;
    TCS34725_RGBC_t last_raw;
    ColorID_t        last_color;
} TCS34725_Handle_t;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise TCS34725: power on, set integration time + gain, enable LED.
 * @param  hdev  Pointer to driver handle (must be pre-filled with hi2c)
 * @param  atime Integration time constant (TCS34725_INTEGRATIONTIME_50MS etc.)
 * @param  gain  Gain constant (TCS34725_GAIN_4X etc.)
 * @retval HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef TCS34725_Init(TCS34725_Handle_t *hdev,
                                 I2C_HandleTypeDef *hi2c,
                                 uint8_t atime,
                                 uint8_t gain);

/**
 * @brief  Start non-blocking RGBC integration (call once to begin).
 */
HAL_StatusTypeDef TCS34725_StartIntegration(TCS34725_Handle_t *hdev);

/**
 * @brief  Poll for integration completion; reads RGBC if ready.
 *         Call this every 20ms in the main loop.
 * @param  hdev   Driver handle
 * @param  out    Pointer to colour result (updated when new reading available)
 * @retval true if new colour result available, false if still integrating
 */
bool TCS34725_PollNonBlocking(TCS34725_Handle_t *hdev, ColorID_t *out);

/**
 * @brief  Blocking read (use only outside the 50Hz loop, e.g., init tests).
 */
HAL_StatusTypeDef TCS34725_ReadRGBC_Blocking(TCS34725_Handle_t *hdev,
                                              TCS34725_RGBC_t *out);

/**
 * @brief  Classify RGBC raw values into a colour ID.
 * @param  raw  Pointer to raw RGBC values
 * @retval ColorID_t
 */
ColorID_t TCS34725_ClassifyColor(const TCS34725_RGBC_t *raw);

/**
 * @brief  Enable/disable the built-in LED illuminator (PC0).
 * @param  on  true = LED ON, false = LED OFF
 */
void TCS34725_SetLED(bool on);

#endif /* TCS34725_H */

/**
 * @file    line_array.c
 * @brief   8-Channel TCRT5000 IR Line Sensor Array Driver Implementation
 */

#include "line_array.h"

/* ─── Internal State ─────────────────────────────────────────────────────── */
static uint8_t  _intersection_consec = 0;   /* Consecutive intersection samples */
static uint32_t _lost_cycles         = 0;   /* Consecutive all-black cycles     */

/* ─── Public Functions ───────────────────────────────────────────────────── */

void LineArray_Init(void) {
    /*
     * GPIO PA0–PA7 configured via STM32CubeMX as:
     *   Mode:  Input
     *   Pull:  Pull-down (ensures LOW when disconnected or sensor inactive)
     *
     * This function exists as a hook for any runtime re-init needed.
     * CubeMX-generated MX_GPIO_Init() should be called in main() first.
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                 GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    LineArray_Reset();
}

uint8_t LineArray_Read(void) {
    /*
     * Read the entire GPIOA input data register in one instruction.
     * Mask lower 8 bits (PA0–PA7 = S1–S8).
     * This is <100ns — no protocol overhead.
     */
    return (uint8_t)(GPIOA->IDR & 0x00FFU);
}

uint8_t LineArray_ActiveCount(uint8_t bits) {
    /* Count set bits (popcount) */
    uint8_t count = 0;
    while (bits) {
        count += (bits & 1u);
        bits >>= 1u;
    }
    return count;
}

float LineArray_GetCentroid(uint8_t bits) {
    float sum_weight = 0.0f;
    float sum_value  = 0.0f;

    for (int i = 0; i < LINEARRAY_NUM_SENSORS; i++) {
        if (bits & (1u << i)) {
            sum_value  += (float)i;
            sum_weight += 1.0f;
        }
    }

    if (sum_weight == 0.0f) return NAN;  /* All sensors black — line lost */

    float centroid = sum_value / sum_weight;   /* 0.0 to 7.0 */
    return centroid - LINEARRAY_CENTER_INDEX;  /* -3.5 to +3.5 */
}

bool LineArray_IsIntersection(uint8_t bits) {
    uint8_t active = LineArray_ActiveCount(bits);

    if (active >= LINEARRAY_INTERSECTION_THRESHOLD) {
        _intersection_consec++;
        if (_intersection_consec >= LINEARRAY_INTERSECTION_CONSEC) {
            /* Keep counter saturated — don't let it overflow */
            if (_intersection_consec > LINEARRAY_INTERSECTION_CONSEC + 10)
                _intersection_consec = LINEARRAY_INTERSECTION_CONSEC;
            return true;
        }
    } else {
        /* Reset — condition no longer met */
        _intersection_consec = 0;
    }

    return false;
}

bool LineArray_IsAllBlack(uint8_t bits) {
    return (bits == 0x00u);
}

uint32_t LineArray_GetLostCycles(uint8_t bits) {
    if (LineArray_IsAllBlack(bits)) {
        _lost_cycles++;
    } else {
        _lost_cycles = 0;
    }
    return _lost_cycles;
}

void LineArray_Reset(void) {
    _intersection_consec = 0;
    _lost_cycles = 0;
}

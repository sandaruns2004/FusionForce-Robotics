/**
 * @file    line_array_esp32.c
 * @brief   9-Channel Analog IR Sensor Array Driver -- ESP32 Implementation
 * @details FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL
 *
 * Signal chain per update():
 *   ADC read (16x oversample) -> EMA filter (alpha=0.7, spike reject) ->
 *   Normalise [0=white,1=black] -> Invert [1=on line] ->
 *   Weighted centroid -> Error [-4.0..+4.0] -> Junction / Lost detection
 */

#include "line_array_esp32.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "LineArray";

/* ---- ADC channel map (9 sensors) ---------------------------------------- */
static const adc1_channel_t ADC1_CH[8] = {
    ADC1_CHANNEL_4,  /* S1  GPIO32 */
    ADC1_CHANNEL_5,  /* S2  GPIO33 */
    ADC1_CHANNEL_6,  /* S3  GPIO34 */
    ADC1_CHANNEL_7,  /* S4  GPIO35 */
    ADC1_CHANNEL_0,  /* S5  GPIO36 centre */
    ADC1_CHANNEL_1,  /* S6  GPIO37 */
    ADC1_CHANNEL_2,  /* S7  GPIO38 */
    ADC1_CHANNEL_3,  /* S8  GPIO39 */
};
/* S9 on ADC2 (Wi-Fi must be OFF) */
static const adc2_channel_t S9_CH = ADC2_CHANNEL_8; /* GPIO25 */

/* ---- Internal state ------------------------------------------------------- */
static LA_Calibration_t  _cal;
static LA_Result_t       _last;
static float             _ema[LA_NUM_SENSORS];
static bool              _ema_init[LA_NUM_SENSORS];
static int               _junc_consec = 0;
static uint32_t          _lost_cycles = 0;
static esp_adc_cal_characteristics_t _adc_chars;
static SemaphoreHandle_t _mutex       = NULL;
static bool              _ready       = false;

/* ---- Private helpers ------------------------------------------------------ */
static uint16_t prv_read_adc1(adc1_channel_t ch) {
    uint32_t s = 0;
    for (int i = 0; i < LA_OVERSAMPLE_N; i++) s += (uint32_t)adc1_get_raw(ch);
    return (uint16_t)(s / LA_OVERSAMPLE_N);
}

static uint16_t prv_read_s9(void) {
    uint32_t s = 0; int raw = 0;
    for (int i = 0; i < LA_OVERSAMPLE_N; i++) {
        if (adc2_get_raw(S9_CH, ADC_WIDTH_BIT_12, &raw) == ESP_OK) s += (uint32_t)raw;
        else s += (uint32_t)_ema[8];
    }
    return (uint16_t)(s / LA_OVERSAMPLE_N);
}

static float prv_ema(int idx, uint16_t raw) {
    float f = (float)raw;
    if (!_ema_init[idx]) { _ema[idx] = f; _ema_init[idx] = true; return f; }
    if (fabsf(f - _ema[idx]) > LA_SPIKE_THRESHOLD) {
        ESP_LOGD(TAG, "S%d spike raw=%d", idx+1, raw);
        f = _ema[idx];
    }
    _ema[idx] = LA_EMA_ALPHA * f + (1.0f - LA_EMA_ALPHA) * _ema[idx];
    return _ema[idx];
}

static float prv_norm(int idx, float filt) {
    float rng = (float)(_cal.max_val[idx] - _cal.min_val[idx]);
    if (rng < 10.0f) rng = 10.0f;
    float n = (filt - (float)_cal.min_val[idx]) / rng;
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}

static esp_err_t prv_load_nvs(void) {
    nvs_handle_t h;
    esp_err_t r = nvs_open(LA_NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) return r;
    char k[24]; uint16_t v;
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        snprintf(k, sizeof(k), LA_NVS_KEY_MIN_FMT, i);
        if ((r = nvs_get_u16(h, k, &v)) != ESP_OK) { nvs_close(h); return r; }
        _cal.min_val[i] = v;
        snprintf(k, sizeof(k), LA_NVS_KEY_MAX_FMT, i);
        if ((r = nvs_get_u16(h, k, &v)) != ESP_OK) { nvs_close(h); return r; }
        _cal.max_val[i] = v;
    }
    nvs_close(h); _cal.is_valid = true;
    ESP_LOGI(TAG, "Calibration loaded from NVS");
    return ESP_OK;
}

static esp_err_t prv_save_nvs(void) {
    nvs_handle_t h;
    esp_err_t r = nvs_open(LA_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    char k[24];
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        snprintf(k, sizeof(k), LA_NVS_KEY_MIN_FMT, i); nvs_set_u16(h, k, _cal.min_val[i]);
        snprintf(k, sizeof(k), LA_NVS_KEY_MAX_FMT, i); nvs_set_u16(h, k, _cal.max_val[i]);
    }
    r = nvs_commit(h); nvs_close(h);
    if (r == ESP_OK) ESP_LOGI(TAG, "Calibration saved to NVS");
    return r;
}

static void prv_default_cal(void) {
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        _cal.min_val[i] = LA_CAL_DEFAULT_MIN;
        _cal.max_val[i] = LA_CAL_DEFAULT_MAX;
    }
    _cal.is_valid = false;
    ESP_LOGW(TAG, "Using DEFAULT calibration -- run LA_Calibrate() properly!");
}

/* ---- Public API ----------------------------------------------------------- */

esp_err_t LA_Init(void) {
    if (_ready) return ESP_OK;
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) { ESP_LOGE(TAG, "Mutex alloc failed"); return ESP_ERR_NO_MEM; }

    adc1_config_width(ADC_WIDTH_BIT_12);
    for (int i = 0; i < 8; i++) adc1_config_channel_atten(ADC1_CH[i], ADC_ATTEN_DB_11);
    adc2_config_channel_atten(S9_CH, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &_adc_chars);

    memset(_ema,      0, sizeof(_ema));
    memset(_ema_init, 0, sizeof(_ema_init));
    memset(&_last,    0, sizeof(_last));
    _last.centroid = LA_CENTRE_INDEX;
    _last.error    = 0.0f;
    _junc_consec   = 0;
    _lost_cycles   = 0;

    if (prv_load_nvs() != ESP_OK) prv_default_cal();

    _ready = true;
    ESP_LOGI(TAG, "LA init OK: %d sensors, 12-bit, 11dB, %dx oversample", LA_NUM_SENSORS, LA_OVERSAMPLE_N);
    return ESP_OK;
}

esp_err_t LA_Calibrate(bool white_surface) {
    if (!_ready) return ESP_ERR_INVALID_STATE;
    const int NSAMP = 32;
    uint32_t acc[LA_NUM_SENSORS] = {0};
    for (int s = 0; s < NSAMP; s++) {
        for (int i = 0; i < 8; i++) acc[i] += prv_read_adc1(ADC1_CH[i]);
        acc[8] += prv_read_s9();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        uint16_t avg = (uint16_t)(acc[i] / NSAMP);
        if (white_surface) { _cal.min_val[i] = avg; ESP_LOGI(TAG, "S%d white=%d", i+1, avg); }
        else               { _cal.max_val[i] = avg; ESP_LOGI(TAG, "S%d black=%d", i+1, avg); }
    }
    if (!white_surface) {
        bool ok = true;
        for (int i = 0; i < LA_NUM_SENSORS; i++) {
            if (_cal.max_val[i] < _cal.min_val[i] + 200) {
                ESP_LOGE(TAG, "S%d: black(%d) too close to white(%d)!", i+1, _cal.max_val[i], _cal.min_val[i]);
                ok = false;
            }
        }
        if (ok) { _cal.is_valid = true; prv_save_nvs(); ESP_LOGI(TAG, "Calibration COMPLETE"); }
        else    { ESP_LOGW(TAG, "Calibration FAILED -- check mounting height"); }
    }
    return ESP_OK;
}

esp_err_t LA_Update(LA_Result_t *result) {
    if (!_ready || !result) return ESP_ERR_INVALID_ARG;
    LA_Result_t r; memset(&r, 0, sizeof(r));

    /* Step 1: Read ADC (16x oversample) */
    for (int i = 0; i < 8; i++) r.adc_raw[i] = prv_read_adc1(ADC1_CH[i]);
    r.adc_raw[8] = prv_read_s9();

    /* Step 2+3+4: EMA -> normalise -> invert */
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        float filt   = prv_ema(i, r.adc_raw[i]);
        r.raw_norm[i] = prv_norm(i, filt);
        r.line_val[i] = 1.0f - r.raw_norm[i];    /* 1.0 = on white line */
    }

    /* Step 5: Count active sensors */
    r.active_count = 0;
    for (int i = 0; i < LA_NUM_SENSORS; i++)
        if (r.line_val[i] > LA_ACTIVE_THRESHOLD) r.active_count++;

    /* Step 6: Weighted centroid */
    float sp = 0.0f, sw = 0.0f;
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        if (r.line_val[i] > LA_CONTRIB_THRESHOLD) { sp += (float)i * r.line_val[i]; sw += r.line_val[i]; }
    }
    if (sw > 0.0f) {
        r.centroid      = sp / sw;
        r.error         = r.centroid - LA_CENTRE_INDEX;
        r.error_mm      = r.error * LA_SENSOR_SPACING_MM;
        r.is_line_lost  = false;
        _lost_cycles    = 0;
    } else {
        r.centroid      = NAN; r.error = NAN; r.error_mm = NAN;
        r.is_line_lost  = true;
        r.lost_duration_ms = ++_lost_cycles * 10;   /* 100Hz -> 10ms per cycle */
    }

    /* Step 7: Junction temporal filter */
    if (r.active_count >= LA_JUNCTION_MIN_SENSORS) {
        if (++_junc_consec > LA_JUNCTION_CONSEC_SAMPLES + 20) _junc_consec = LA_JUNCTION_CONSEC_SAMPLES;
        r.is_junction = (_junc_consec >= LA_JUNCTION_CONSEC_SAMPLES);
    } else {
        _junc_consec = 0; r.is_junction = false;
    }

    /* Step 8: Thread-safe publish */
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&_last, &r, sizeof(LA_Result_t));
        xSemaphoreGive(_mutex);
    }
    memcpy(result, &r, sizeof(LA_Result_t));
    return ESP_OK;
}

void LA_GetLastResult(LA_Result_t *result) {
    if (!result) return;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(result, &_last, sizeof(LA_Result_t));
        xSemaphoreGive(_mutex);
    }
}

void LA_Reset(void) { _junc_consec = 0; _lost_cycles = 0; ESP_LOGD(TAG, "Reset"); }

void LA_PrintDebug(const LA_Result_t *r) {
    if (!r) return;
    printf("[LA] ");
    for (int i = 0; i < LA_NUM_SENSORS; i++)
        printf("%s", r->line_val[i] > LA_ACTIVE_THRESHOLD ? "\xe2\x96\x88" : "\xe2\x96\x91");
    if (!isnan(r->error))
        printf(" | err=%.2f (%.1fmm) | act=%d%s%s\n",
               r->error, r->error_mm, r->active_count,
               r->is_junction  ? " JUNC" : "",
               r->is_line_lost ? " LOST" : "");
    else
        printf(" | LINE LOST (%lu ms)\n", (unsigned long)r->lost_duration_ms);
}

void LA_GetCalibration(LA_Calibration_t *cal) {
    if (cal) memcpy(cal, &_cal, sizeof(LA_Calibration_t));
}

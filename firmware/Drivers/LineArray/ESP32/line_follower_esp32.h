/**
 * @file    line_follower_esp32.h / .c (combined single-header for simplicity)
 * @brief   PD Line Following Controller using 9-sensor centroid error
 * @details FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL
 *
 * Usage (on STM32, after parsing ESP32 UART):
 *   LF_Controller_t lf;
 *   LF_Init(&lf, 0.60f, 0.08f, 1.20f, 0.01f);  // kp, kd, wz_max, dt(100Hz)
 *   float wz = LF_ComputeOmega(&lf, sensor_error); // call every 10ms
 *
 * Or use on ESP32 directly if motor control is local.
 *
 * Tuning starting point:
 *   Kp = 0.60  (increase until oscillation, then halve)
 *   Kd = 0.08  (increase until oscillations dampen)
 *   Wz_max = 1.20 rad/s (max turn rate, match robot capability)
 */

#ifndef LINE_FOLLOWER_ESP32_H
#define LINE_FOLLOWER_ESP32_H

#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default tuning parameters */
#define LF_KP_DEFAULT       0.60f
#define LF_KD_DEFAULT       0.08f
#define LF_WZ_MAX_DEFAULT   1.20f   /* rad/s */
#define LF_DT_DEFAULT       0.01f   /* 100 Hz */

typedef struct {
    float kp;           /* Proportional gain */
    float kd;           /* Derivative gain */
    float wz_max;       /* Maximum output angular velocity (rad/s) */
    float dt;           /* Time step (seconds) -- must match call rate */
    float prev_error;   /* Previous error for derivative term */
    bool  has_prev;     /* False until first call (no derivative on first sample) */
} LF_Controller_t;

/**
 * @brief  Initialise PD line follower controller.
 * @param  lf       Controller struct pointer
 * @param  kp       Proportional gain
 * @param  kd       Derivative gain
 * @param  wz_max   Maximum angular velocity output (rad/s)
 * @param  dt       Control loop period (seconds); 0.01 for 100Hz
 */
static inline void LF_Init(LF_Controller_t *lf, float kp, float kd, float wz_max, float dt) {
    lf->kp        = kp;
    lf->kd        = kd;
    lf->wz_max    = wz_max;
    lf->dt        = (dt > 0.0f) ? dt : LF_DT_DEFAULT;
    lf->prev_error = 0.0f;
    lf->has_prev  = false;
}

/**
 * @brief  Compute angular velocity Wz from centroid error.
 *
 * omega = Kp * error + Kd * (error - prev_error) / dt
 * Clamped to [-wz_max, +wz_max]
 *
 * Convention (matches robot forward = +X):
 *   error < 0 (line left)  -> omega negative -> turn left
 *   error > 0 (line right) -> omega positive -> turn right
 *
 * @param  lf     Controller state
 * @param  error  Centroid error from LA_Update (-4.0 to +4.0, NAN if lost)
 * @retval omega  Angular velocity in rad/s; 0.0 if error is NAN
 */
static inline float LF_ComputeOmega(LF_Controller_t *lf, float error) {
    if (isnan(error)) {
        lf->has_prev = false;  /* Reset derivative on line-lost */
        return 0.0f;
    }

    float d_term = 0.0f;
    if (lf->has_prev) {
        d_term = (error - lf->prev_error) / lf->dt;
    }

    float omega = lf->kp * error + lf->kd * d_term;

    /* Saturate output */
    if (omega >  lf->wz_max) omega =  lf->wz_max;
    if (omega < -lf->wz_max) omega = -lf->wz_max;

    lf->prev_error = error;
    lf->has_prev   = true;

    return omega;
}

/**
 * @brief  Reset derivative history.
 *         Call when re-entering line-following state after a pause.
 */
static inline void LF_Reset(LF_Controller_t *lf) {
    lf->prev_error = 0.0f;
    lf->has_prev   = false;
}

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWER_ESP32_H */

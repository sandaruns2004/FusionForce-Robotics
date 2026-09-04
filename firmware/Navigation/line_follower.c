/**
 * @file    line_follower.c
 * @brief   PD Line Following Controller Implementation
 */

#include "line_follower.h"

void LineFollower_Init(LineFollower_t *lf, float kp, float kd, float wz_max) {
    lf->kp        = kp;
    lf->kd        = kd;
    lf->wz_max    = wz_max;
    lf->prev_error = 0.0f;
    lf->has_prev  = false;
}

float LineFollower_ComputeOmega(LineFollower_t *lf, float error, float dt) {
    /* Line lost — no steering input */
    if (isnan(error)) {
        lf->has_prev = false;
        return 0.0f;
    }

    float derivative = 0.0f;
    if (lf->has_prev && dt > 0.0f) {
        derivative = (error - lf->prev_error) / dt;
    }

    float wz = lf->kp * error + lf->kd * derivative;

    /* Clamp to max angular velocity */
    if (wz >  lf->wz_max) wz =  lf->wz_max;
    if (wz < -lf->wz_max) wz = -lf->wz_max;

    lf->prev_error = error;
    lf->has_prev   = true;

    return wz;
}

void LineFollower_Reset(LineFollower_t *lf) {
    lf->prev_error = 0.0f;
    lf->has_prev   = false;
}

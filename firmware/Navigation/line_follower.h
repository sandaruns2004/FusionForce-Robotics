/**
 * @file    line_follower.h
 * @brief   PD Line Following Controller using 8-sensor weighted centroid
 * @details Computes angular velocity (Wz) from line centroid error.
 *          Must be called every 20ms (50Hz) for correct derivative term.
 */

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdbool.h>
#include <math.h>

/* ─── Default Tuning Parameters (adjust empirically) ────────────────────── */
#define LINE_FOLLOWER_KP    0.80f   /* Proportional gain                     */
#define LINE_FOLLOWER_KD    0.05f   /* Derivative gain                       */
#define LINE_FOLLOWER_WZ_MAX 0.80f  /* Max angular velocity (rad/s)          */

/* ─── Controller State ───────────────────────────────────────────────────── */
typedef struct {
    float kp;
    float kd;
    float wz_max;
    float prev_error;
    bool  has_prev;
} LineFollower_t;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise line follower with tuning parameters.
 * @param  lf    Pointer to controller state
 * @param  kp    Proportional gain
 * @param  kd    Derivative gain
 * @param  wz_max  Maximum angular velocity output (rad/s)
 */
void LineFollower_Init(LineFollower_t *lf, float kp, float kd, float wz_max);

/**
 * @brief  Compute angular velocity Wz from centroid error.
 * @param  lf     Controller state
 * @param  error  Weighted centroid error (–3.5 to +3.5, or NAN if line lost)
 * @param  dt     Time step in seconds (typically 0.02s for 50Hz)
 * @retval Wz in rad/s. Positive = turn right, negative = turn left.
 *         Returns 0.0 if error is NAN (line lost — caller handles separately).
 */
float LineFollower_ComputeOmega(LineFollower_t *lf, float error, float dt);

/**
 * @brief  Reset controller derivative history.
 *         Call when re-entering a line following state after a pause.
 */
void LineFollower_Reset(LineFollower_t *lf);

#endif /* LINE_FOLLOWER_H */

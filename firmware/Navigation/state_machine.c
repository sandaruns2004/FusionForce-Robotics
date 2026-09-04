/**
 * @file    state_machine.c
 * @brief   Mission HFSM Implementation — all 4 competition subtasks
 * @note    Call StateMachine_Update() every 20ms from main loop.
 *          All motion commands written to sm_Vx, sm_Vy, sm_Wz, sm_arm_position,
 *          sm_gripper, sm_gate — consumed by gait generator.
 */

#include "state_machine.h"
#include "line_follower.h"
#include <string.h>
#include <stdio.h>   /* printf for debug UART — remove in final build */

/* ─── Motion Command Globals ─────────────────────────────────────────────── */
float   sm_Vx           = 0.0f;
float   sm_Vy           = 0.0f;
float   sm_Wz           = 0.0f;
uint8_t sm_arm_position = ARM_HOME;
uint8_t sm_gripper      = GRIPPER_OPEN;
uint8_t sm_gate         = GATE_LOCKED;

/* ─── Private State ──────────────────────────────────────────────────────── */
static MissionState_t   current_state     = STATE_BOOT_INIT;
static ColorID_t        stored_ball_color = COLOR_UNKNOWN;
static uint32_t         state_timer_ms    = 0;  /* Time in current state (ms) */
static uint8_t          color_stable_cnt  = 0;  /* Consecutive stable colour reads */
static LineFollower_t   line_pd;

/* ─── Flash helpers ──────────────────────────────────────────────────────── */
static void _SaveColorToFlash(ColorID_t color) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = BALL_COLOR_FLASH_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t err;
    HAL_FLASHEx_Erase(&erase, &err);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, BALL_COLOR_FLASH_ADDR, (uint8_t)color);
    HAL_FLASH_Lock();
}

static ColorID_t _ReadColorFromFlash(void) {
    uint8_t val = *(volatile uint8_t*)BALL_COLOR_FLASH_ADDR;
    if (val == 0xFF || val > COLOR_BLUE) return COLOR_UNKNOWN;  /* erased / invalid */
    return (ColorID_t)val;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void StateMachine_Init(void) {
    LineFollower_Init(&line_pd, LINE_FOLLOWER_KP, LINE_FOLLOWER_KD, LINE_FOLLOWER_WZ_MAX);
    stored_ball_color = _ReadColorFromFlash();
    current_state     = STATE_BOOT_INIT;
    state_timer_ms    = 0;

#ifdef DEBUG_UART_ENABLED
    printf("[SM] Init. Recalled ball colour: %d\r\n", (int)stored_ball_color);
#endif
}

void StateMachine_Transition(MissionState_t new_state) {
#ifdef DEBUG_UART_ENABLED
    printf("[SM] %d -> %d\r\n", (int)current_state, (int)new_state);
#endif
    current_state  = new_state;
    state_timer_ms = 0;
    color_stable_cnt = 0;
    LineFollower_Reset(&line_pd);
}

MissionState_t StateMachine_GetState(void)       { return current_state; }
ColorID_t      StateMachine_GetStoredColor(void) { return stored_ball_color; }

void StateMachine_SaveBallColor(ColorID_t color) {
    stored_ball_color = color;
    _SaveColorToFlash(color);
}

/* ─── Main Update (call every 20ms) ──────────────────────────────────────── */
void StateMachine_Update(const SM_SensorData_t *s) {
    state_timer_ms += 20;  /* Increment 20ms per call */

    switch (current_state) {

    /* ── BOOT & CALIB ─────────────────────────────────────────────────────── */
    case STATE_BOOT_INIT:
        sm_Vx = 0; sm_Vy = 0; sm_Wz = 0;
        /* Peripherals initialised in main() before this is called */
        StateMachine_Transition(STATE_SENSOR_CALIB);
        break;

    case STATE_SENSOR_CALIB:
        /* Sensor calibration runs passively — just wait */
        sm_Vx = 0; sm_Wz = 0;
        if (state_timer_ms >= 500) {
            StateMachine_Transition(STATE_IDLE_WAIT_START);
        }
        break;

    case STATE_IDLE_WAIT_START:
        sm_Vx = 0; sm_Wz = 0;
        /* Start condition: start button pressed (PA0 checked externally;
           call StateMachine_Transition(STATE_TASK1_LINE_FOLLOW) from main) */
        break;

    /* ── TASK 1: GRID SEARCH ─────────────────────────────────────────────── */
    case STATE_TASK1_LINE_FOLLOW:
        sm_Wz = LineFollower_ComputeOmega(&line_pd, s->line_centroid, 0.02f);
        sm_Vx = 60.0f;
        sm_arm_position = ARM_HOME;

        if (s->intersection) {
            if (s->tof_front_mm < 80) {
                StateMachine_Transition(STATE_TASK1_BALL_APPROACH);
            }
            /* If no ball at this intersection: continue line follow */
        }

        /* Line lost safety */
        if (LineArray_GetLostCycles(s->line_bits) >= LINEARRAY_LOST_TIMEOUT_CYCLES) {
            StateMachine_Transition(STATE_SAFE_STOP);
        }
        break;

    case STATE_TASK1_BALL_APPROACH:
        sm_Vx = 0; sm_Wz = 0;
        sm_arm_position = ARM_MODE_A;  /* Lower arm to ball level */
        if (state_timer_ms >= 800) {   /* 800ms for servo to reach position */
            StateMachine_Transition(STATE_TASK1_COLOR_ID);
        }
        break;

    case STATE_TASK1_COLOR_ID:
        sm_Vx = 0; sm_Wz = 0;
        sm_arm_position = ARM_MODE_A;

        if (s->last_color != COLOR_UNKNOWN) {
            color_stable_cnt++;
            if (color_stable_cnt >= 3) {
                StateMachine_SaveBallColor(s->last_color);
                StateMachine_Transition(STATE_TASK1_BALL_GRAB);
            }
        } else {
            color_stable_cnt = 0;
        }

        if (state_timer_ms >= 10000) {
            /* No colour detected after 10s — false alarm, resume grid */
            StateMachine_Transition(STATE_TASK1_LINE_FOLLOW);
        }
        break;

    case STATE_TASK1_BALL_GRAB:
        sm_Vx = 0; sm_Wz = 0;
        sm_arm_position = ARM_MODE_A;
        sm_gripper = GRIPPER_CLOSE;
        if (state_timer_ms >= 1200) {
            StateMachine_Transition(STATE_TASK1_STORE);
        }
        break;

    case STATE_TASK1_STORE:
        sm_Vx = 0; sm_Wz = 0;
        sm_gripper = GRIPPER_CLOSE;
        sm_arm_position = ARM_HOME;
        sm_gate = GATE_LOCKED;
        if (state_timer_ms >= 1000) {
            StateMachine_Transition(STATE_TASK1_GRID_EXIT);
        }
        break;

    case STATE_TASK1_GRID_EXIT:
        sm_Wz = LineFollower_ComputeOmega(&line_pd, s->line_centroid, 0.02f);
        sm_Vx = 60.0f;
        /* Transition condition: line lost (left grid) OR specific exit intersection */
        if (LineArray_IsAllBlack(s->line_bits) && state_timer_ms >= 500) {
            StateMachine_Transition(STATE_TASK2_WALL_FOLLOW);
        }
        break;

    /* ── TASK 2: CURVED CORRIDOR ──────────────────────────────────────────── */
    case STATE_TASK2_WALL_FOLLOW: {
        /* Wall PD centering: target = 150mm from each wall */
        float wall_error = (float)s->tof_left_mm - (float)s->tof_right_mm;
        sm_Wz = wall_error * 0.003f;    /* Tune this gain */
        sm_Vx = 80.0f;

        /* Gap rejection: only accept gap if ToF > 250mm for 3 cycles (handled in ToF driver) */
        if (s->tof_front_mm < 150 && state_timer_ms >= 2000) {
            StateMachine_Transition(STATE_TASK3_WALL_FOLLOW);
        }
        break;
    }

    /* ── TASK 3: OBSTACLE PUSH ────────────────────────────────────────────── */
    case STATE_TASK3_WALL_FOLLOW: {
        float wall_error = (float)s->tof_left_mm - (float)s->tof_right_mm;
        sm_Wz = wall_error * 0.003f;
        sm_Vx = 80.0f;
        if (s->tof_front_mm < 150) {
            StateMachine_Transition(STATE_TASK3_OBSTACLE_DETECT);
        }
        break;
    }

    case STATE_TASK3_OBSTACLE_DETECT:
        sm_Vx = 0; sm_Wz = 0;
        if (s->tof_front_mm < 150) {
            color_stable_cnt++;  /* Reuse counter for confirmation cycles */
            if (color_stable_cnt >= 3) {
                StateMachine_Transition(STATE_TASK3_PUSH);
            }
        } else {
            color_stable_cnt = 0;
        }
        break;

    case STATE_TASK3_PUSH:
        sm_Vx = 50.0f; sm_Wz = 0;
        /* Obstacle cleared when front ToF > 300mm, or timeout 8s */
        if (s->tof_front_mm > 300 || state_timer_ms >= 8000) {
            StateMachine_Transition(STATE_TASK3_TURN);
        }
        /* Tilt monitor: if pushing causes tilt, stop briefly */
        if (s->pitch_deg > 20.0f) { sm_Vx = 0; }
        break;

    case STATE_TASK3_TURN:
        sm_Vx = 0;
        sm_Wz = -0.5f;  /* Rotate left at 0.5 rad/s */
        /* TODO: Replace with heading sensor (IMU yaw integration) for 90 deg */
        if (state_timer_ms >= 3200) {  /* Empirical: ~3.2s for 90 deg at 0.5 rad/s */
            sm_Wz = 0;
            StateMachine_Transition(STATE_TASK4_LINE_FOLLOW);
        }
        break;

    /* ── TASK 4: COLOUR SORT DELIVERY ────────────────────────────────────── */
    case STATE_TASK4_LINE_FOLLOW:
        sm_Wz = LineFollower_ComputeOmega(&line_pd, s->line_centroid, 0.02f);
        sm_Vx = 60.0f;
        sm_arm_position = ARM_HOME;

        if (s->intersection) {
            StateMachine_Transition(STATE_TASK4_JUNCTION_DETECT);
        }
        break;

    case STATE_TASK4_JUNCTION_DETECT:
        sm_Vx = 0; sm_Wz = 0;
        sm_arm_position = ARM_MODE_B;  /* Point arm at floor */

        if (s->last_color == stored_ball_color && s->last_color != COLOR_UNKNOWN) {
            color_stable_cnt++;
            if (color_stable_cnt >= 3) {
                StateMachine_Transition(STATE_TASK4_BRANCH_FOLLOW);
            }
        } else {
            color_stable_cnt = 0;
        }

        if (state_timer_ms >= 5000) {
            /* Timeout: colour match failed — try slight rotation to check another branch */
            sm_Wz = 0.3f;
            if (state_timer_ms >= 7000) {
                StateMachine_Transition(STATE_SAFE_STOP);
            }
        }
        break;

    case STATE_TASK4_BRANCH_FOLLOW:
        sm_arm_position = ARM_HOME;
        sm_Wz = LineFollower_ComputeOmega(&line_pd, s->line_centroid, 0.02f);
        sm_Vx = 60.0f;

        /* Branch end: line lost AND far from front obstacle */
        if (LineArray_IsAllBlack(s->line_bits) && s->tof_front_mm > 400) {
            if (state_timer_ms >= 400) {
                StateMachine_Transition(STATE_TASK4_BALL_RELEASE);
            }
        }
        break;

    case STATE_TASK4_BALL_RELEASE:
        sm_Vx = 0; sm_Wz = 0;
        sm_gate = GATE_OPEN;
        if (state_timer_ms >= 1500) {
            StateMachine_Transition(STATE_FINISH);
        }
        break;

    /* ── TERMINAL STATES ─────────────────────────────────────────────────── */
    case STATE_FINISH:
        sm_Vx = 0; sm_Vy = 0; sm_Wz = 0;
        /* Flash LED pattern here (via TIM3 or GPIO toggle in main) */
        break;

    case STATE_SAFE_STOP:
        sm_Vx = 0; sm_Vy = 0; sm_Wz = 0;
        /* Restart: wait for start button press to re-enter SENSOR_CALIB */
        break;

    case STATE_ERROR_RECOVERY:
        /* Slow rotate search for line re-acquisition */
        sm_Vx = 0;
        sm_Wz = (state_timer_ms % 2000 < 1000) ? 0.3f : -0.3f;
        if (!LineArray_IsAllBlack(s->line_bits)) {
            /* Line found — return to previous task state (simplified: Task 1) */
            StateMachine_Transition(STATE_TASK1_LINE_FOLLOW);
        }
        if (state_timer_ms >= 10000) {
            StateMachine_Transition(STATE_SAFE_STOP);
        }
        break;

    default:
        StateMachine_Transition(STATE_SAFE_STOP);
        break;
    }
}

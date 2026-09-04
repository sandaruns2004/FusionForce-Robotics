/**
 * @file    state_machine.h
 * @brief   Mission Hierarchical Finite State Machine (HFSM) for STM32
 * @details Covers all 4 competition subtasks across 18 operational states.
 *          Runs every 20ms in the main loop. Persists ball colour to Flash.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "stm32f4xx_hal.h"
#include "tcs34725.h"
#include "line_array.h"
#include <stdbool.h>
#include <stdint.h>

/* ─── State Definitions ──────────────────────────────────────────────────── */
typedef enum {
    STATE_BOOT_INIT              = 0,
    STATE_SENSOR_CALIB           = 1,
    STATE_IDLE_WAIT_START        = 2,

    /* Task 1: Grid Search + Ball Retrieval */
    STATE_TASK1_LINE_FOLLOW      = 3,
    STATE_TASK1_BALL_APPROACH    = 4,
    STATE_TASK1_COLOR_ID         = 5,
    STATE_TASK1_BALL_GRAB        = 6,
    STATE_TASK1_STORE            = 7,
    STATE_TASK1_GRID_EXIT        = 8,

    /* Task 2: Curved Corridor */
    STATE_TASK2_WALL_FOLLOW      = 9,

    /* Task 3: Obstacle Push */
    STATE_TASK3_WALL_FOLLOW      = 10,
    STATE_TASK3_OBSTACLE_DETECT  = 11,
    STATE_TASK3_PUSH             = 12,
    STATE_TASK3_TURN             = 13,

    /* Task 4: Colour Sort Delivery */
    STATE_TASK4_LINE_FOLLOW      = 14,
    STATE_TASK4_JUNCTION_DETECT  = 15,
    STATE_TASK4_BRANCH_FOLLOW    = 16,
    STATE_TASK4_BALL_RELEASE     = 17,

    STATE_FINISH                 = 18,
    STATE_SAFE_STOP              = 19,
    STATE_ERROR_RECOVERY         = 20,

    STATE_COUNT
} MissionState_t;

/* ─── Sensor Input Bundle ─────────────────────────────────────────────────── */
typedef struct {
    uint8_t   line_bits;
    float     line_centroid;
    bool      intersection;
    uint16_t  tof_front_mm;
    uint16_t  tof_left_mm;
    uint16_t  tof_right_mm;
    ColorID_t last_color;
    float     pitch_deg;
    float     roll_deg;
    uint16_t  batt_mv;
} SM_SensorData_t;

/* ─── Motion Commands (output) ───────────────────────────────────────────── */
/* Written by state machine; consumed by gait generator */
extern float   sm_Vx;           /* mm/s forward velocity                     */
extern float   sm_Vy;           /* mm/s lateral (usually 0)                  */
extern float   sm_Wz;           /* rad/s angular velocity                    */
extern uint8_t sm_arm_position; /* 0=HOME  1=MODE_A(ball)  2=MODE_B(floor)  */
extern uint8_t sm_gripper;      /* 0=OPEN  1=CLOSE                           */
extern uint8_t sm_gate;         /* 0=LOCKED  1=OPEN                          */

/* Arm position constants */
#define ARM_HOME    0
#define ARM_MODE_A  1
#define ARM_MODE_B  2

/* Gripper constants */
#define GRIPPER_OPEN   0
#define GRIPPER_CLOSE  1

/* Gate constants */
#define GATE_LOCKED 0
#define GATE_OPEN   1

/* ─── Flash persistence addresses ────────────────────────────────────────── */
#define BALL_COLOR_FLASH_ADDR   0x0807F800UL    /* Last 2KB of STM32F411 Flash */
#define BALL_COLOR_FLASH_SECTOR FLASH_SECTOR_7  /* 128KB sector 7              */

/* ─── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise state machine. Reads stored_ball_color from Flash.
 *         Call once after all peripherals are initialised.
 */
void StateMachine_Init(void);

/**
 * @brief  Update state machine. Must be called every 20ms (50Hz loop).
 * @param  s  Pointer to current sensor readings
 */
void StateMachine_Update(const SM_SensorData_t *s);

/**
 * @brief  Force transition to a new state (also called internally).
 * @param  new_state  Target state
 */
void StateMachine_Transition(MissionState_t new_state);

/**
 * @brief  Get current state.
 */
MissionState_t StateMachine_GetState(void);

/**
 * @brief  Get the stored ball colour (loaded from Flash on boot).
 */
ColorID_t StateMachine_GetStoredColor(void);

/**
 * @brief  Write ball colour to STM32 Flash for reset persistence.
 * @param  color  Colour ID to store
 */
void StateMachine_SaveBallColor(ColorID_t color);

#endif /* STATE_MACHINE_H */

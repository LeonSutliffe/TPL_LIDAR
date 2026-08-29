#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "mks_servo42d.h"

typedef enum {
    TILT_STATUS_UNHOMED,
    TILT_STATUS_HOMING,
    TILT_STATUS_IDLE,
    TILT_STATUS_MOVING,
    TILT_STATUS_SWEEPING,
    TILT_STATUS_SETTLING,
    TILT_STATUS_SETTLED,
    TILT_STATUS_ERROR,
} tilt_axis_status_t;

typedef struct {
    mks_handle_t *mks;
    tilt_axis_status_t status;
    int homing_substate;
    float position_deg;   /* relative to the home reference set at end of homing */
    float target_deg;
    int64_t state_enter_time_us;

    /* 0 (default) = step-and-stare mode: tilt_axis_move_to_deg() commands a
     * discrete position-mode move and waits for it to finish.
     * >0 = continuous sweep mode: tilt_axis_move_to_deg() commands a
     * velocity-mode run toward target_deg instead, streaming position_deg
     * the whole way there rather than stopping at intermediate stations.
     * Same entry points either way -- only this setting changes behavior. */
    float sweep_rate_deg_s;
    bool sweep_dir_positive;  /* direction of the currently active sweep */
} tilt_axis_t;

void tilt_axis_init(tilt_axis_t *t, mks_handle_t *mks);

/* Triggered once per session by the PC, per the project brief -- not run
 * autonomously on ESP32 boot, so the timing/outcome lands in ROS2 logs. */
void tilt_axis_start_homing(tilt_axis_t *t);

/* Sets the operating mode used by future tilt_axis_move_to_deg() calls.
 * deg_per_sec <= 0 selects step-and-stare (the default); deg_per_sec > 0
 * selects continuous sweep at that angular rate. Doesn't affect a move
 * already in progress. */
void tilt_axis_set_sweep_rate(tilt_axis_t *t, float deg_per_sec);

/* Commands the axis to target_deg. Behavior depends on sweep_rate_deg_s:
 * step-and-stare mode does one position-mode move and settles; sweep mode
 * runs continuously at the configured rate until target_deg is reached,
 * then settles the same way. */
void tilt_axis_move_to_deg(tilt_axis_t *t, float target_deg);

/* Call periodically (e.g. every 100ms) from the main control loop; drives
 * the homing/move/settle state machine and refreshes position_deg. */
void tilt_axis_update(tilt_axis_t *t);

float tilt_axis_get_position_deg(const tilt_axis_t *t);
const char *tilt_axis_get_status_string(const tilt_axis_t *t);

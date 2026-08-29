#include "tilt_axis.h"

#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "tilt_axis";

/*
 * TUNE: every constant below needs empirical calibration on real hardware.
 * SETTLE_TIME_MS in particular is called out in the project brief as the
 * least-known variable and likely the dominant cost in total scan time --
 * measure it on ~20-30 real stops before trusting this placeholder.
 * HOMING_FAST_SPEED/HOMING_SLOW_SPEED/MOVE_SPEED are in RPM (driver's real
 * range is 0-3000, see mks_servo42d.h); MOVE_ACCEL is the driver's ramp-rate
 * unit (0-255, 0 = no ramp). Current numeric values are still placeholders,
 * just now in real, interpretable units instead of unknown "raw" ones.
 */
#define HOMING_CURRENT_MA        300
#define RUN_CURRENT_MA          1000
#define HOMING_FAST_SPEED         200
#define HOMING_SLOW_SPEED          20
#define MOVE_SPEED                400
#define MOVE_ACCEL                 80
#define HOMING_BACKOFF_DEG          3.0f
#define HOMING_SWEEP_DEG           300.0f  /* far enough to guarantee hitting the end-stop from anywhere in range */
#define SETTLE_TIME_MS             500
/* Homing direction: sweeps toward the end-stop first. Flip if your mount's
 * stop is on the other side. */
#define HOMING_DIRECTION_TOWARD_STOP  false

/* Continuous-sweep mode: deg/s converted to the driver's RPM units
 * (1 RPM = 6 deg/s), clamped to the driver's real 0-3000 RPM range. */
#define SWEEP_MIN_DELTA_DEG          0.05f  /* below this, treat as "already there" rather than starting a sweep */

/* Direct-drive, no gearing (per brief). Set MICROSTEPS to match the MKS
 * driver's actual microstep configuration (DIP switches / current firmware
 * default) -- this file doesn't set it over UART. */
#define MOTOR_FULL_STEPS_PER_REV   200
#define MICROSTEPS                  16
#define STEPS_PER_DEG   ((MOTOR_FULL_STEPS_PER_REV * MICROSTEPS) / 360.0f)

static uint32_t deg_to_pulses(float deg)
{
    if (deg < 0) deg = -deg;
    return (uint32_t)(deg * STEPS_PER_DEG + 0.5f);
}

static uint16_t sweep_rate_to_driver_speed(float deg_per_sec)
{
    float rpm = deg_per_sec / 6.0f;
    if (rpm < 0.0f) rpm = 0.0f;
    if (rpm > 3000.0f) rpm = 3000.0f;
    return (uint16_t)rpm;
}

void tilt_axis_init(tilt_axis_t *t, mks_handle_t *mks)
{
    memset(t, 0, sizeof(*t));
    t->mks = mks;
    t->status = TILT_STATUS_UNHOMED;
}

void tilt_axis_start_homing(tilt_axis_t *t)
{
    if (t->status == TILT_STATUS_HOMING) {
        return;
    }
    ESP_LOGI(TAG, "starting homing sequence");
    t->status = TILT_STATUS_HOMING;
    t->homing_substate = 0;
    t->state_enter_time_us = esp_timer_get_time();
}

void tilt_axis_set_sweep_rate(tilt_axis_t *t, float deg_per_sec)
{
    t->sweep_rate_deg_s = deg_per_sec > 0.0f ? deg_per_sec : 0.0f;
}

void tilt_axis_move_to_deg(tilt_axis_t *t, float target_deg)
{
    if (t->status == TILT_STATUS_HOMING) {
        ESP_LOGW(TAG, "ignoring move command while homing");
        return;
    }
    if (t->status == TILT_STATUS_UNHOMED) {
        ESP_LOGW(TAG, "ignoring move command before homing");
        return;
    }
    if (t->status == TILT_STATUS_MOVING || t->status == TILT_STATUS_SWEEPING) {
        ESP_LOGW(TAG, "ignoring move command while already moving/sweeping");
        return;
    }

    t->target_deg = target_deg;
    float delta = target_deg - t->position_deg;

    if (t->sweep_rate_deg_s > 0.0f) {
        if (fabsf(delta) < SWEEP_MIN_DELTA_DEG) {
            t->status = TILT_STATUS_SETTLED;
            return;
        }
        t->sweep_dir_positive = delta >= 0;
        mks_run_continuous(t->mks, t->sweep_dir_positive, sweep_rate_to_driver_speed(t->sweep_rate_deg_s),
                            MOVE_ACCEL);
        t->status = TILT_STATUS_SWEEPING;
        t->state_enter_time_us = esp_timer_get_time();
        return;
    }

    bool dir_positive = delta >= 0;
    uint32_t pulses = deg_to_pulses(delta);

    if (pulses == 0) {
        t->status = TILT_STATUS_SETTLED;
        return;
    }

    mks_run_relative(t->mks, dir_positive, MOVE_SPEED, MOVE_ACCEL, pulses);
    t->status = TILT_STATUS_MOVING;
    t->state_enter_time_us = esp_timer_get_time();
}

void tilt_axis_update(tilt_axis_t *t)
{
    mks_status_t status;

    switch (t->status) {
    case TILT_STATUS_HOMING:
        switch (t->homing_substate) {
        case 0: /* enter: drop to homing current, sweep toward the stop */
            mks_set_working_current(t->mks, HOMING_CURRENT_MA);
            mks_run_relative(t->mks, HOMING_DIRECTION_TOWARD_STOP, HOMING_FAST_SPEED, MOVE_ACCEL,
                              deg_to_pulses(HOMING_SWEEP_DEG));
            t->homing_substate = 1;
            break;

        case 1: /* fast approach: wait for the stall flag */
            if (mks_read_status(t->mks, &status) == ESP_OK && status.valid) {
                if (status.stalled) {
                    mks_stop(t->mks);
                    /* Stall protection de-energizes the motor when it trips --
                     * release it or the backoff move below silently no-ops. */
                    mks_release_stall(t->mks);
                    mks_run_relative(t->mks, !HOMING_DIRECTION_TOWARD_STOP, HOMING_SLOW_SPEED, MOVE_ACCEL,
                                      deg_to_pulses(HOMING_BACKOFF_DEG));
                    t->homing_substate = 2;
                } else if (!status.moving) {
                    /* Sweep finished without ever hitting the end-stop -- most likely
                     * HOMING_DIRECTION_TOWARD_STOP is wrong for this mount, or
                     * HOMING_SWEEP_DEG doesn't reach the stop from this position. */
                    ESP_LOGE(TAG, "homing fast approach completed without a stall -- check "
                                  "HOMING_DIRECTION_TOWARD_STOP / HOMING_SWEEP_DEG");
                    t->status = TILT_STATUS_ERROR;
                }
            }
            break;

        case 2: /* backing off a few degrees before the slow re-approach */
            if (mks_read_status(t->mks, &status) == ESP_OK && status.valid && !status.moving) {
                mks_run_relative(t->mks, HOMING_DIRECTION_TOWARD_STOP, HOMING_SLOW_SPEED, MOVE_ACCEL,
                                  deg_to_pulses(HOMING_BACKOFF_DEG * 2));
                t->homing_substate = 3;
            }
            break;

        case 3: /* slow re-approach: this stall is the actual zero reference */
            if (mks_read_status(t->mks, &status) == ESP_OK && status.valid) {
                if (status.stalled) {
                    mks_stop(t->mks);
                    mks_release_stall(t->mks);
                    mks_set_zero_position(t->mks);
                    mks_set_working_current(t->mks, RUN_CURRENT_MA);
                    t->position_deg = 0.0f;
                    t->status = TILT_STATUS_IDLE;
                    ESP_LOGI(TAG, "homing complete, zero set");
                } else if (!status.moving) {
                    /* Re-approach shouldn't be able to finish without a stall since it
                     * travels further than the backoff did -- if it does, something's
                     * inconsistent (slip, wrong backoff distance, noisy stall detection). */
                    ESP_LOGE(TAG, "homing slow re-approach completed without a stall");
                    t->status = TILT_STATUS_ERROR;
                }
            }
            break;
        }
        break;

    case TILT_STATUS_MOVING: {
        float pos;
        if (mks_read_encoder_deg(t->mks, &pos) == ESP_OK) {
            t->position_deg = pos;
        }
        if (mks_read_status(t->mks, &status) == ESP_OK && status.valid && !status.moving) {
            t->status = TILT_STATUS_SETTLING;
            t->state_enter_time_us = esp_timer_get_time();
        }
        break;
    }

    case TILT_STATUS_SWEEPING: {
        float pos;
        if (mks_read_encoder_deg(t->mks, &pos) == ESP_OK) {
            t->position_deg = pos;
        }
        /* Direction is latched from when the sweep started rather than
         * recomputed each tick, so a noisy reading near the crossing can't
         * flip which side counts as "reached". */
        bool reached = t->sweep_dir_positive ? (t->position_deg >= t->target_deg)
                                              : (t->position_deg <= t->target_deg);
        if (reached) {
            mks_stop(t->mks);
            t->status = TILT_STATUS_SETTLING;
            t->state_enter_time_us = esp_timer_get_time();
        }
        break;
    }

    case TILT_STATUS_SETTLING: {
        float pos;
        if (mks_read_encoder_deg(t->mks, &pos) == ESP_OK) {
            t->position_deg = pos;
        }
        if ((esp_timer_get_time() - t->state_enter_time_us) / 1000 >= SETTLE_TIME_MS) {
            t->status = TILT_STATUS_SETTLED;
        }
        break;
    }

    case TILT_STATUS_IDLE:
    case TILT_STATUS_SETTLED: {
        float pos;
        if (mks_read_encoder_deg(t->mks, &pos) == ESP_OK) {
            t->position_deg = pos;
        }
        break;
    }

    case TILT_STATUS_UNHOMED:
    case TILT_STATUS_ERROR:
    default:
        break;
    }
}

float tilt_axis_get_position_deg(const tilt_axis_t *t)
{
    return t->position_deg;
}

const char *tilt_axis_get_status_string(const tilt_axis_t *t)
{
    switch (t->status) {
    case TILT_STATUS_UNHOMED:  return "unhomed";
    case TILT_STATUS_HOMING:   return "homing";
    case TILT_STATUS_IDLE:     return "idle";
    case TILT_STATUS_MOVING:   return "moving";
    case TILT_STATUS_SWEEPING: return "sweeping";
    case TILT_STATUS_SETTLING: return "settling";
    case TILT_STATUS_SETTLED:  return "settled";
    case TILT_STATUS_ERROR:    return "error";
    default:                   return "unknown";
    }
}

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

/*
 * Driver for an MKS SERVO42D closed-loop stepper driver over RS485, using
 * standard Modbus-RTU (function codes 0x04 read-input-registers /
 * 0x06 write-single-register / 0x10 write-multiple-registers, CRC16).
 *
 * Register map, function codes, and framing below are taken directly from
 * "MKS SERVO42&57D_Modbus RTU User Manual V1.0.9" and cross-checked against
 * several of the manual's own worked examples (CRC16 byte order confirmed
 * against 4 independent example frames). This driver puts the motor into
 * bus control mode (SR_vFOC) and stays there for the lifetime of the
 * handle -- all the position/speed/status registers used here are
 * documented as valid only in that mode.
 *
 * Still unverified / needs real-hardware confirmation (see README.md):
 *   - which physical direction (CW vs CCW) corresponds to direction_positive
 *   - the driver's configured microstep count (STEPS_PER_DEG in tilt_axis.c)
 *   - default baud rate (38400) and slave address (1) actually match this unit
 */

typedef struct {
    uart_port_t uart_port;
    uint8_t slave_addr;
} mks_handle_t;

typedef struct {
    bool valid;          /* response parsed and checksum OK */
    bool moving;          /* bus operating status != "stop" */
    bool stalled;         /* stall-protection flag -- used as the sensorless homing trigger */
} mks_status_t;

/* Configures RS485_UART_PORT in UART_MODE_RS485_HALF_DUPLEX with the given
 * pins/baud, then puts the driver into SR_vFOC bus control mode and enables
 * stall protection (required for mks_read_status()'s `stalled` flag to ever
 * be set -- without it, stall detection never triggers). */
esp_err_t mks_init(mks_handle_t *h, uart_port_t uart_port, int tx_pin, int rx_pin, int rts_pin,
                    int baud_rate, uint8_t slave_addr);

esp_err_t mks_set_working_current(mks_handle_t *h, uint16_t milliamps);

/* Marks the motor's current physical position as coordinate zero. */
esp_err_t mks_set_zero_position(mks_handle_t *h);

/* Stall protection unlocks/de-energizes the motor when triggered -- call
 * this after observing mks_status_t.stalled and before issuing another move,
 * or the next move command will silently do nothing. */
esp_err_t mks_release_stall(mks_handle_t *h);

/* Relative move. speed is in RPM (0-3000), accel is a driver-internal ramp
 * rate (0-255, 0 = jump straight to speed, higher = faster ramp) -- see
 * MKS_SERVO manual Part 7.1 for the exact accel/time relationship. pulses is
 * in full driver microsteps (see STEPS_PER_DEG in tilt_axis.c for the
 * degrees<->pulses conversion, which depends on the driver's microstep
 * setting -- not configured over UART by this firmware). */
esp_err_t mks_run_relative(mks_handle_t *h, bool direction_positive, uint16_t speed, uint8_t accel,
                            uint32_t pulses);

/* Continuous (velocity-mode) run: rotates at `speed` RPM (0-3000) in the
 * given direction, ramping at `accel` (0-255), until mks_stop() is called --
 * no fixed pulse target, unlike mks_run_relative(). Used for
 * continuous-sweep operation instead of step-and-stare's discrete moves. */
esp_err_t mks_run_continuous(mks_handle_t *h, bool direction_positive, uint16_t speed, uint8_t accel);

/* Emergency stop -- immediate, not a decelerated stop. */
esp_err_t mks_stop(mks_handle_t *h);

esp_err_t mks_read_status(mks_handle_t *h, mks_status_t *out);

/* Reads the motor-shaft multi-turn encoder position in degrees (0 at
 * power-on / last mks_set_zero_position() call). This is motor-shaft
 * degrees, not necessarily tilt-axis output degrees -- apply any gear ratio
 * at the caller if the mount isn't direct-drive. */
esp_err_t mks_read_encoder_deg(mks_handle_t *h, float *out_deg);

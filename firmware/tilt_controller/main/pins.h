#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/*
 * ESP32-S2 (Wemos/Lolin S2 Mini) pin assignments for the tilt-axis controller.
 *
 * RS485 pins below are confirmed against actual wiring. Limit switch / status
 * LED are still placeholders (open item in the project brief) -- picked only
 * to avoid the reserved pins listed there:
 *   GPIO0        boot/strapping
 *   GPIO45/46    strapping
 *   GPIO43/44    native USB-Serial-JTAG console (flashing/monitor)
 *   GPIO19/20    native USB D+/D- (reserved here for the micro-ROS USB CDC transport)
 */

#define RS485_UART_PORT   UART_NUM_1
#define RS485_TX_PIN      GPIO_NUM_33   /* -> RS485 board DI */
#define RS485_RX_PIN      GPIO_NUM_18   /* <- RS485 board RO */
#define RS485_RTS_PIN     GPIO_NUM_16   /* -> RS485 board DE+RE (tied) "EN", driven by UART RS485 half-duplex mode */

#define LIMIT_SWITCH_PIN  GPIO_NUM_5    /* optional mechanical backup to sensorless homing */
#define STATUS_LED_PIN    GPIO_NUM_15   /* onboard/aux LED; verify against board silkscreen */

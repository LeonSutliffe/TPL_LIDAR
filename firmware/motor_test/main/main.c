#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "pins.h"

/*
 * ESP32-S2 RS485 <-> USB-CDC transparent bridge, modeled directly on the
 * already-proven D:\Downloads\MKS_Servo_Tester Pico firmware: no protocol
 * logic here at all, just bytes shipped in both directions. All Modbus/MKS
 * framing lives on the PC side instead (see MksServoTester.Protocol for a
 * ready implementation to port). This sidesteps the still-unresolved
 * question of why THIS board's own Modbus/MKS-protocol test firmware never
 * got a response, by not needing this firmware to understand either
 * protocol in the first place -- if the underlying UART/DE-RE path itself
 * is sound, a PC-side terminal or script talking raw bytes through this
 * bridge should behave identically to the Pico bridge.
 *
 * Uses ESP32-S2's native USB (TinyUSB CDC-ACM) as the PC-facing link, same
 * as the tilt_controller firmware's micro-ROS transport, but as a plain
 * virtual COM port here -- no micro-ROS/uxr involved.
 */

#define MKS_BAUD          38400
#define BRIDGE_BUF_SIZE   256

static const tinyusb_cdcacm_itf_t cdc_itf = TINYUSB_CDC_ACM_0;

/* Diagnostic feedback since there's no way to see console output on this
 * board: default idle state is solid ON. A single ~100ms dip means "USB
 * bytes arrived from the PC and got forwarded to RS485" (proves the USB-CDC
 * link itself is alive, independent of whether RS485 gets a reply). A fast
 * triple-blink means "RS485 bytes came back and got forwarded to USB"
 * (proves the driver actually responded). */
static void led_blip(int count, int ms)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(STATUS_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(ms));
        gpio_set_level(STATUS_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static void pump_usb_to_rs485(void)
{
    uint8_t buf[BRIDGE_BUF_SIZE];
    size_t rx_size = 0;

    esp_err_t ret = tinyusb_cdcacm_read(cdc_itf, buf, sizeof(buf), &rx_size);
    if (ret != ESP_OK || rx_size == 0) {
        return;
    }

    led_blip(1, 100);

    uart_flush_input(RS485_UART_PORT);
    gpio_set_level(RS485_RTS_PIN, 1); /* DE/RE high -> transmit */
    vTaskDelay(pdMS_TO_TICKS(1));
    uart_write_bytes(RS485_UART_PORT, (const char *)buf, rx_size);
    uart_wait_tx_done(RS485_UART_PORT, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(RS485_RTS_PIN, 0); /* DE/RE low -> listen */
}

static void pump_rs485_to_usb(void)
{
    uint8_t buf[BRIDGE_BUF_SIZE];
    int len = uart_read_bytes(RS485_UART_PORT, buf, sizeof(buf), 0);
    if (len <= 0) {
        return;
    }

    led_blip(3, 60);

    tinyusb_cdcacm_write_queue(cdc_itf, buf, (size_t)len);
    tinyusb_cdcacm_write_flush(cdc_itf, 0);
}

void app_main(void)
{
    gpio_reset_pin(RS485_RTS_PIN);
    gpio_set_direction(RS485_RTS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_RTS_PIN, 0); /* idle in listen mode */

    uart_config_t uart_config = {
        .baud_rate = MKS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_PORT, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_PORT, RS485_TX_PIN, RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = cdc_itf,
        .rx_unread_buf_sz = BRIDGE_BUF_SIZE,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    gpio_reset_pin(STATUS_LED_PIN);
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_PIN, 1); /* solid on: bridge is up */

    while (true) {
        pump_usb_to_rs485();
        pump_rs485_to_usb();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

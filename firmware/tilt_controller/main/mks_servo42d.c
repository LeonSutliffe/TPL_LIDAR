#include "mks_servo42d.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mks_servo42d";

/* ---------------------------------------------------------------------
 * Register map, from "MKS SERVO42&57D_Modbus RTU User Manual V1.0.9".
 * All addresses/field layouts below are transcribed directly from that
 * manual's worked examples, not guessed.
 * ------------------------------------------------------------------- */
#define MKS_REG_WORK_MODE           0x0082  /* write: 0-5, see MKS_WORK_MODE_* */
#define MKS_REG_WORKING_CURRENT     0x0083  /* write: mA */
#define MKS_REG_STALL_PROT_EN       0x0088  /* write: 0=disable, 1=enable stall protection */
#define MKS_REG_SET_ZERO_POINT      0x0092  /* write 1: mark current position as coordinate zero */
#define MKS_REG_RELEASE_STALL       0x003D  /* write 1: clear stall-protection lock */
#define MKS_REG_STALL_STATUS        0x003E  /* read (1 reg): reserve(hi)=0, stalled(lo) 0/1 */
#define MKS_REG_ENC_MULTI_TURN      0x0030  /* read (3 regs): carry(int32), value(uint16, 0..0x4000) */

/* Bus control mode only (SR_OPEN/SR_CLOSE/SR_vFOC) -- this driver always
 * runs in SR_vFOC, set during mks_init(). */
#define MKS_REG_BUS_STATUS          0x00F1  /* read (1 reg): reserve(hi)=0, status(lo), see MKS_BUS_STATUS_* */
#define MKS_REG_BUS_ENABLE          0x00F3  /* write: 0/1 */
#define MKS_REG_BUS_ESTOP           0x00F7  /* write 1: immediate stop */
#define MKS_REG_BUS_SPEED_MODE      0x00F6  /* write (2 regs): [dir<<8|acc], speed -- continuous run */
#define MKS_REG_BUS_POS_RELATIVE    0x00FD  /* write (4 regs): [dir<<8|acc], speed, pulses_hi, pulses_lo */

#define MKS_WORK_MODE_SR_vFOC       0x05

#define MKS_BUS_STATUS_STOP         1

#define MKS_DIR_CCW                 0
#define MKS_DIR_CW                  1

/* Encoder is 14-bit per revolution (0..0x3FFF, manual: "Encoder single-turn
 * value range is 0~0x4000"). */
#define MKS_ENCODER_COUNTS_PER_REV     16384

#define MKS_FUNC_READ_INPUT_REGISTERS     0x04
#define MKS_FUNC_WRITE_SINGLE_REGISTER    0x06
#define MKS_FUNC_WRITE_MULTIPLE_REGISTERS 0x10

#define MKS_UART_BUF_SIZE   256
#define MKS_TX_TIMEOUT_MS   50
#define MKS_RX_TIMEOUT_MS   100

/* Standard Modbus-RTU CRC16 (poly 0xA001, init 0xFFFF, little-endian on the
 * wire) -- byte order verified against 4 independent example frames from
 * the MKS manual. */
static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static esp_err_t modbus_transact(mks_handle_t *h, const uint8_t *req, size_t req_len,
                                  uint8_t *resp, size_t resp_len)
{
    uart_flush_input(h->uart_port);
    int written = uart_write_bytes(h->uart_port, (const char *)req, req_len);
    if (written != (int)req_len) {
        ESP_LOGE(TAG, "uart_write_bytes short write (%d/%d)", written, (int)req_len);
        return ESP_FAIL;
    }
    /* Half-duplex mode + RTS-driven DE/RE handles direction switching automatically;
     * uart_wait_tx_done ensures the frame has actually left the FIFO before we
     * start listening for a response. */
    uart_wait_tx_done(h->uart_port, pdMS_TO_TICKS(MKS_TX_TIMEOUT_MS));

    int read = uart_read_bytes(h->uart_port, resp, resp_len, pdMS_TO_TICKS(MKS_RX_TIMEOUT_MS));
    if (read != (int)resp_len) {
        ESP_LOGW(TAG, "modbus response timeout/short (%d/%d)", read, (int)resp_len);
        return ESP_ERR_TIMEOUT;
    }
    uint16_t expected_crc = modbus_crc16(resp, resp_len - 2);
    uint16_t got_crc = (uint16_t)resp[resp_len - 2] | ((uint16_t)resp[resp_len - 1] << 8);
    if (expected_crc != got_crc || resp[0] != h->slave_addr) {
        ESP_LOGW(TAG, "modbus response CRC/addr mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

static esp_err_t mks_read_registers(mks_handle_t *h, uint16_t start_reg, uint16_t count, uint16_t *out)
{
    uint8_t req[8];
    req[0] = h->slave_addr;
    req[1] = MKS_FUNC_READ_INPUT_REGISTERS;
    req[2] = (uint8_t)(start_reg >> 8);
    req[3] = (uint8_t)(start_reg & 0xFF);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count & 0xFF);
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    /* addr + func + byte_count + 2*count data bytes + 2 CRC bytes */
    size_t resp_len = 3 + (size_t)count * 2 + 2;
    uint8_t resp[3 + 2 * 8 + 2];
    esp_err_t err = modbus_transact(h, req, sizeof(req), resp, resp_len);
    if (err != ESP_OK) {
        return err;
    }
    if (resp[1] != MKS_FUNC_READ_INPUT_REGISTERS || resp[2] != count * 2) {
        ESP_LOGW(TAG, "unexpected read-registers response (func 0x%02X, byte_count %d)", resp[1], resp[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (uint16_t i = 0; i < count; i++) {
        out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
    }
    return ESP_OK;
}

static esp_err_t mks_write_register(mks_handle_t *h, uint16_t reg, uint16_t value)
{
    uint8_t req[8];
    req[0] = h->slave_addr;
    req[1] = MKS_FUNC_WRITE_SINGLE_REGISTER;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = (uint8_t)(value >> 8);
    req[5] = (uint8_t)(value & 0xFF);
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    /* Success response echoes the request verbatim. */
    uint8_t resp[8];
    esp_err_t err = modbus_transact(h, req, sizeof(req), resp, sizeof(resp));
    if (err != ESP_OK) {
        return err;
    }
    if (memcmp(req, resp, sizeof(req)) != 0) {
        ESP_LOGW(TAG, "write-single-register response didn't echo request");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t mks_write_registers(mks_handle_t *h, uint16_t start_reg, uint16_t count,
                                      const uint16_t *values)
{
    uint8_t req[7 + 2 * 8 + 2];
    size_t idx = 0;
    req[idx++] = h->slave_addr;
    req[idx++] = MKS_FUNC_WRITE_MULTIPLE_REGISTERS;
    req[idx++] = (uint8_t)(start_reg >> 8);
    req[idx++] = (uint8_t)(start_reg & 0xFF);
    req[idx++] = (uint8_t)(count >> 8);
    req[idx++] = (uint8_t)(count & 0xFF);
    req[idx++] = (uint8_t)(count * 2);
    for (uint16_t i = 0; i < count; i++) {
        req[idx++] = (uint8_t)(values[i] >> 8);
        req[idx++] = (uint8_t)(values[i] & 0xFF);
    }
    uint16_t crc = modbus_crc16(req, idx);
    req[idx++] = (uint8_t)(crc & 0xFF);
    req[idx++] = (uint8_t)(crc >> 8);

    /* Success response: addr + func + start_reg + count + CRC (no data echo). */
    uint8_t resp[8];
    esp_err_t err = modbus_transact(h, req, idx, resp, sizeof(resp));
    if (err != ESP_OK) {
        return err;
    }
    if (resp[1] != MKS_FUNC_WRITE_MULTIPLE_REGISTERS) {
        ESP_LOGW(TAG, "unexpected write-multiple-registers response (func 0x%02X)", resp[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t mks_init(mks_handle_t *h, uart_port_t uart_port, int tx_pin, int rx_pin, int rts_pin,
                    int baud_rate, uint8_t slave_addr)
{
    h->uart_port = uart_port;
    h->slave_addr = slave_addr;

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_driver_install(uart_port, MKS_UART_BUF_SIZE, MKS_UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(uart_port, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(uart_port, tx_pin, rx_pin, rts_pin, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    /* Hardware RS485 half-duplex: UART toggles RTS (-> DE/RE) itself around each
     * TX frame instead of us bit-banging a direction pin, avoiding the race
     * where direction flips before the last byte finishes shifting out. */
    err = uart_set_mode(uart_port, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK) return err;

    /* All the position/speed/status registers this driver uses are only
     * valid in bus control mode. */
    err = mks_write_register(h, MKS_REG_WORK_MODE, MKS_WORK_MODE_SR_vFOC);
    if (err != ESP_OK) return err;

    /* Required for mks_read_status()'s `stalled` flag (and hence sensorless
     * homing) to ever trigger. */
    err = mks_write_register(h, MKS_REG_STALL_PROT_EN, 1);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t mks_set_working_current(mks_handle_t *h, uint16_t milliamps)
{
    return mks_write_register(h, MKS_REG_WORKING_CURRENT, milliamps);
}

esp_err_t mks_set_zero_position(mks_handle_t *h)
{
    return mks_write_register(h, MKS_REG_SET_ZERO_POINT, 1);
}

esp_err_t mks_release_stall(mks_handle_t *h)
{
    return mks_write_register(h, MKS_REG_RELEASE_STALL, 1);
}

esp_err_t mks_run_relative(mks_handle_t *h, bool direction_positive, uint16_t speed, uint8_t accel,
                            uint32_t pulses)
{
    uint16_t dir_accel = ((uint16_t)(direction_positive ? MKS_DIR_CW : MKS_DIR_CCW) << 8) | accel;
    uint16_t values[4] = {
        dir_accel,
        speed,
        (uint16_t)(pulses >> 16),
        (uint16_t)(pulses & 0xFFFF),
    };
    return mks_write_registers(h, MKS_REG_BUS_POS_RELATIVE, 4, values);
}

esp_err_t mks_run_continuous(mks_handle_t *h, bool direction_positive, uint16_t speed, uint8_t accel)
{
    uint16_t dir_accel = ((uint16_t)(direction_positive ? MKS_DIR_CW : MKS_DIR_CCW) << 8) | accel;
    uint16_t values[2] = { dir_accel, speed };
    return mks_write_registers(h, MKS_REG_BUS_SPEED_MODE, 2, values);
}

esp_err_t mks_stop(mks_handle_t *h)
{
    return mks_write_register(h, MKS_REG_BUS_ESTOP, 1);
}

esp_err_t mks_read_status(mks_handle_t *h, mks_status_t *out)
{
    out->valid = false;

    uint16_t status_reg;
    esp_err_t err = mks_read_registers(h, MKS_REG_BUS_STATUS, 1, &status_reg);
    if (err != ESP_OK) {
        return err;
    }
    uint16_t stall_reg;
    err = mks_read_registers(h, MKS_REG_STALL_STATUS, 1, &stall_reg);
    if (err != ESP_OK) {
        return err;
    }

    out->valid = true;
    out->moving = (status_reg & 0xFF) != MKS_BUS_STATUS_STOP;
    out->stalled = (stall_reg & 0xFF) != 0;
    return ESP_OK;
}

esp_err_t mks_read_encoder_deg(mks_handle_t *h, float *out_deg)
{
    uint16_t regs[3];
    esp_err_t err = mks_read_registers(h, MKS_REG_ENC_MULTI_TURN, 3, regs);
    if (err != ESP_OK) {
        return err;
    }
    int32_t carry = (int32_t)(((uint32_t)regs[0] << 16) | regs[1]);
    uint16_t value = regs[2];
    int64_t counts = (int64_t)carry * MKS_ENCODER_COUNTS_PER_REV + (int64_t)value;
    *out_deg = (float)((double)counts * 360.0 / (double)MKS_ENCODER_COUNTS_PER_REV);
    return ESP_OK;
}

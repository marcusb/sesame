#include "pic_uart.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "controller.h"
#include "idcm_msg.h"

LOG_MODULE_REGISTER(pic_uart, LOG_LEVEL_DBG);

K_MSGQ_DEFINE(pic_queue, sizeof(pic_cmd_t), 32, 4);

#define READ_TIMEOUT_TICKS 10000
#define DOOR_POLL_TICKS (30 * 1000)
#define DOOR_MOVE_ALERT_TICKS 6000
#define STATE_UPDATE_INTERVAL (30 * 1000)

#if DT_NODE_EXISTS(DT_NODELABEL(pic_rst))
static bool self_test_done;
static bool s_log_pic_version = true;
static door_open_state_t state = DCM_DOOR_STATE_UNKNOWN;
static door_direction_t direction = DCM_DOOR_DIR_UNKNOWN;
static uint16_t pos;
static uint16_t down_limit;
static uint16_t up_limit;
static uint32_t last_state_pub_time;

static const struct device* uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
static const struct gpio_dt_spec pic_rst =
    GPIO_DT_SPEC_GET(DT_NODELABEL(pic_rst), gpios);
static const struct gpio_dt_spec pic_wake =
    GPIO_DT_SPEC_GET(DT_NODELABEL(pic_wake), gpios);
#if DT_NODE_EXISTS(DT_NODELABEL(pic_sbl))
static const struct gpio_dt_spec pic_sbl =
    GPIO_DT_SPEC_GET(DT_NODELABEL(pic_sbl), gpios);
#endif

static bool current_light_state;
static volatile bool sbl_active;
K_MSGQ_DEFINE(sbl_rx_q, sizeof(uint8_t), 256, 4);

static struct k_work_delayable close_work;
static bool close_scheduled;
static uint16_t last_raw_pos;
static bool has_last_raw_pos;
static pic_version_info_t s_pic_version;
static pic_stats_t s_stats;

static void cancel_scheduled_close(const char* reason) {
    if (close_scheduled || k_work_delayable_is_pending(&close_work)) {
        LOG_INF("Cancelling scheduled close: %s", reason);
        close_scheduled = false;
        k_work_cancel_delayable(&close_work);
    }
}

static void close_work_handler(struct k_work* work) {
    if (close_scheduled) {
        pic_cmd_t cmd = PIC_CMD_CLOSE_EXEC;
        int ret = k_msgq_put(&pic_queue, &cmd, K_NO_WAIT);
        if (ret != 0) {
            LOG_ERR("Failed to enqueue PIC_CMD_CLOSE_EXEC: %d", ret);
        }
    }
}

#define RX_BUF_SIZE 128
static uint8_t rx_buf[RX_BUF_SIZE];
static int rx_idx = 0;
typedef enum { READ_HEADER, READ_BODY } read_state_t;
static read_state_t read_state;
static uint8_t expected_len;

static uint8_t next_token() {
    static int seq = -1;
    if (seq == -1 || seq == 76 || seq == 84) {
        seq += 2;
    } else {
        seq += 1;
    }
    if (seq > 252) {
        seq = 1;
    }
    return seq;
}

static void send_door_status_req(void);

static uint8_t dcm_calc_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xB2;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc ^ 0xFF;
}

static uint8_t sbl_calc_chk_sum(const uint8_t* p, int n) {
    if (n < 2) {
        return 0;
    }
    uint8_t res = 0;
    p++;
    for (; n > 1; n--, p++) {
        res ^= *p;
    }
    return res;
}

static void door_state_update(door_open_state_t new_state, door_direction_t dir,
                              uint16_t raw_pos) {
    if (close_scheduled) {
        if (dir != DCM_DOOR_DIR_STOPPED && dir != DCM_DOOR_DIR_UNKNOWN) {
            cancel_scheduled_close("door direction not stopped");
        } else if (new_state == DCM_DOOR_STATE_CLOSED) {
            cancel_scheduled_close("door reached closed");
        } else if (has_last_raw_pos && raw_pos != last_raw_pos) {
            cancel_scheduled_close("door position changed");
        }
    }
    last_raw_pos = raw_pos;
    has_last_raw_pos = true;

    bool update = state != new_state || direction != dir;
    state = new_state;
    direction = dir;
    LOG_INF("door status: state=%d, dir=%d, pos=%u, down_lim=%u, up_lim=%u",
            state, dir, raw_pos, down_limit, up_limit);
    if (up_limit != down_limit) {
        if (up_limit > down_limit && raw_pos + 4 >= up_limit) {
            pos = 100;
        } else if (raw_pos <= down_limit + 4) {
            pos = 0;
        } else {
            int32_t val = 100 * (int32_t)(raw_pos - down_limit) /
                          (int32_t)(up_limit - down_limit);
            if (val < 0) {
                val = 0;
            }
            if (val > 100) {
                val = 100;
            }
            pos = val;
        }
    }
    uint32_t now = k_uptime_get_32();
    if (update || dir != DCM_DOOR_DIR_STOPPED ||
        now > last_state_pub_time + STATE_UPDATE_INTERVAL) {
        last_state_pub_time = now;
        ctrl_msg_t state_upd = {CTRL_MSG_DOOR_STATE_UPDATE,
                                .msg.door_state = {state, dir, pos}};
        k_msgq_put(&ctrl_queue, &state_upd, K_NO_WAIT);
    }
}

static void handle_msg(const dcm_msg_t* msg) {
    switch (msg->type) {
        case DCM_MSG_STATUS_UPDATE: {
            if (msg->len < sizeof(dcm_status_update_msg_t)) {
                break;
            }
            const dcm_status_update_msg_t* p = &msg->payload.door_status;
            down_limit = p->down_limit;
            up_limit = p->up_limit;
            door_state_update(p->state, p->direction, p->pos);
            s_stats.status_count++;
            break;
        }

        case DCM_MSG_STATUS_UPDATE_EXT: {
            if (msg->len < sizeof(dcm_status_update_ext_msg_t)) {
                break;
            }
            const dcm_status_update_ext_msg_t* p =
                &msg->payload.door_status_ext;
            down_limit = p->down_limit;
            up_limit = p->up_limit;
            door_state_update(p->state, p->direction, p->pos);
            s_stats.status_count++;
            break;
        }

        case DCM_MSG_DOOR_ACK:
            LOG_INF("PIC door command ACK");
            s_stats.door_ack_count++;
            break;

        case DCM_MSG_ALERT_ACK:
            LOG_INF("PIC alert command ACK");
            s_stats.alert_ack_count++;
            break;

        case DCM_MSG_AUDIO_ACK:
            self_test_done = true;
            LOG_INF("PIC audio command ACK");
            s_stats.audio_ack_count++;
            break;

        case DCM_MSG_LIGHT_ACK: {
            const dcm_light_ack_msg_t* p = &msg->payload.light_ack;
            bool on = (p->state != 0);
            current_light_state = on;
            ctrl_msg_t state_upd = {.type = CTRL_MSG_LIGHT_STATE_UPDATE,
                                    .msg.light_state = {.on = on}};
            k_msgq_put(&ctrl_queue, &state_upd, K_NO_WAIT);
            LOG_INF("PIC light state ACK: %s (%u)", on ? "ON" : "OFF",
                    p->state);
            s_stats.light_ack_count++;
            break;
        }

        case DCM_MSG_SENSOR_VERSION: {
            const dcm_sensor_version_msg_t* p = &msg->payload.sensor_version;
            s_pic_version.major = p->major;
            s_pic_version.minor = p->minor;
            s_pic_version.patch = p->patch;
            s_pic_version.suffix = p->suffix;
            s_pic_version.received = true;
            up_limit = p->up_limit;
            down_limit = p->down_limit;
            door_state_update(p->state, p->direction, p->pos);
            if (s_log_pic_version) {
                if (p->suffix >= 'A' && p->suffix <= 'Z') {
                    LOG_INF("PIC firmware version: %u.%u.%u%c", p->major,
                            p->minor, p->patch, p->suffix);
                } else {
                    LOG_INF("PIC firmware version: %u.%u.%u+%u", p->major,
                            p->minor, p->patch, (uint8_t)p->suffix);
                }
                s_log_pic_version = false;
            }
            s_stats.version_count++;
            break;
        }

        case DCM_MSG_OPS_EVENT: {
            s_stats.ops_event_count++;
            const dcm_ops_event_msg_t* p = &msg->payload.ops_event;
            LOG_DBG("ops event %d", p->event);
            break;
        }

        default:
            break;
    }
}

K_MSGQ_DEFINE(uart_rx_q, sizeof(uint8_t), 256, 4);

static uint32_t last_read_tick;

static void start_uart_read() {
    read_state = READ_HEADER;
    rx_idx = 0;
    expected_len = 4;
    gpio_pin_set_dt(&pic_wake, 1);
}

static void log_packet(const char* prefix, const uint8_t* data, size_t len) {
    char buf[128];
    size_t off = 0;
    for (size_t i = 0; i < len && off + 3 < sizeof(buf); i++) {
        off += snprintf(buf + off, sizeof(buf) - off, "%02x ", data[i]);
    }
    if (off > 0 && buf[off - 1] == ' ') {
        buf[off - 1] = '\0';
    }
    LOG_DBG("%s: %s", prefix, buf);
}

static void process_serial_data() {
    uint8_t c;
    while (k_msgq_get(&uart_rx_q, &c, K_NO_WAIT) == 0) {
        if (rx_idx < sizeof(rx_buf)) {
            rx_buf[rx_idx++] = c;
        }

        if (read_state == READ_HEADER) {
            last_read_tick = k_uptime_get_32();
            if (rx_idx == 1 && rx_buf[0] != 0x55) {
                LOG_DBG("unexpected byte %02x", rx_buf[0]);
                rx_idx = 0;
                continue;
            }
            if (rx_idx == 4) {
                const dcm_msg_t* msg = (const dcm_msg_t*)rx_buf;
                gpio_pin_set_dt(&pic_wake, 0);
                if (msg->len + 5 > sizeof(rx_buf)) {
                    LOG_INF("invalid msg length %d", msg->len);
                    start_uart_read();
                    continue;
                }
                read_state = READ_BODY;
                expected_len = msg->len + 5;
            }
        } else if (read_state == READ_BODY) {
            if (rx_idx == expected_len) {
                const dcm_msg_t* msg = (const dcm_msg_t*)rx_buf;
                log_packet("RX", rx_buf, expected_len);
                uint8_t rx_chksum = rx_buf[msg->len + 4];
                uint8_t crc = dcm_calc_crc8(rx_buf, msg->len + 3);

                if (rx_chksum == crc) {
                    handle_msg(msg);
                } else {
                    LOG_WRN("CRC8 mismatch (got 0x%02x, expected 0x%02x)",
                            rx_chksum, crc);
                    log_packet("invalid chksum packet", rx_buf, expected_len);
                }
                start_uart_read();
            }
        }
    }
}

static void uart_cb(const struct device* dev, void* user_data) {
    uart_irq_update(dev);
    while (uart_irq_rx_ready(dev)) {
        uint8_t buf[16];
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len <= 0) break;
        if (sbl_active) {
            for (int i = 0; i < len; i++) {
                k_msgq_put(&sbl_rx_q, &buf[i], K_NO_WAIT);
            }
        } else {
            for (int i = 0; i < len; i++) {
                k_msgq_put(&uart_rx_q, &buf[i], K_NO_WAIT);
            }
            pic_cmd_t cmd = PIC_SERIAL_DATA;
            k_msgq_put(&pic_queue, &cmd, K_NO_WAIT);
        }
    }
}

static bool s_uart_initialized = false;

static int init_uart(void) {
    if (s_uart_initialized) {
        return 0;
    }
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -1;
    }
    if (!device_is_ready(pic_rst.port)) {
        LOG_ERR("PIC Reset GPIO not ready");
        return -1;
    }
    if (!device_is_ready(pic_wake.port)) {
        LOG_ERR("PIC Wake GPIO not ready");
        return -1;
    }

    struct uart_config cfg = {
        .baudrate = 115200,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    int ret = uart_configure(uart_dev, &cfg);
    if (ret != 0) {
        LOG_ERR("Failed to configure UART: %d", ret);
        return ret;
    }

    gpio_pin_configure_dt(&pic_rst, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&pic_wake, GPIO_OUTPUT_ACTIVE);
#if DT_NODE_EXISTS(DT_NODELABEL(pic_sbl))
    if (device_is_ready(pic_sbl.port)) {
        gpio_pin_configure_dt(&pic_sbl, GPIO_OUTPUT_INACTIVE);
    }
#endif

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);

    k_msleep(5);

    uart_irq_rx_enable(uart_dev);
    s_uart_initialized = true;

    return 0;
}

void pic_uart_init(void) { init_uart(); }

static void send_msg(dcm_msg_t* msg) {
    int frame_len = msg->len + 4;
    uint8_t* p = (uint8_t*)msg;
    p[msg->len + 3] = dcm_calc_crc8(p, msg->len + 3);
    log_packet("TX", p, frame_len);
    for (int i = 0; i < frame_len; i++) {
        uart_poll_out(uart_dev, p[i]);
    }
    k_msleep(5);
}

static void alert_cmd(uint8_t val) {
    dcm_msg_t msg = {DCM_HEADER_BYTE, 1 + sizeof(dcm_alert_cmd_msg_t),
                     next_token(), DCM_MSG_ALERT_CMD,
                     .payload.alert_cmd = {0, val, 0}};
    send_msg(&msg);
}

static void audio_cmd(uint8_t val) {
    dcm_msg_t msg = {DCM_HEADER_BYTE, 1 + sizeof(dcm_audio_cmd_msg_t),
                     next_token(), DCM_MSG_AUDIO_CMD,
                     .payload.audio_cmd = {0, val, 0}};
    send_msg(&msg);
}

static void alert() {
    audio_cmd(2);
    alert_cmd(2);
}

static void send_door_cmd(uint8_t val) {
    LOG_INF("Sending door command to PIC: %s (val=%u)", val ? "OPEN" : "CLOSE",
            val);
    dcm_msg_t msg = {DCM_HEADER_BYTE, 1 + sizeof(dcm_door_cmd_msg_t),
                     next_token(), DCM_MSG_DOOR_CMD,
                     .payload.door_cmd = {val, 0}};
    send_msg(&msg);
}

static void send_door_status_req(void) {
    dcm_msg_t msg = {
        DCM_HEADER_BYTE, 1 + sizeof(dcm_door_status_req_msg_t), next_token(),
        DCM_MSG_DOOR_STATUS_REQUEST,
        .payload.door_status_req = {0, 0, 0, pos, up_limit, down_limit}};
    send_msg(&msg);
}

static void cmd_0x04(void) {
    dcm_msg_t msg = {DCM_HEADER_BYTE, 1 + sizeof(dcm_cmd_0x04_msg_t),
                     next_token(), DCM_MSG_0x04};
    send_msg(&msg);
}

static void pic_process_cmd(pic_cmd_t cmd) {
    if (sbl_active && cmd != PIC_SERIAL_DATA) {
        LOG_WRN("Ignoring command %d during PIC update", cmd);
        return;
    }
    switch (cmd) {
        case PIC_CMD_OPEN: {
            cancel_scheduled_close("OPEN command received");
            if (direction == DCM_DOOR_DIR_DOWN) {
                send_door_cmd(1);
            } else if (direction != DCM_DOOR_DIR_UP) {
                bool is_fully_open = (state == DCM_DOOR_STATE_OPEN) &&
                                     (up_limit == down_limit || pos >= 95);
                if (!is_fully_open) {
                    send_door_cmd(1);
                }
            }
            break;
        }

        case PIC_CMD_CLOSE: {
            if (direction == DCM_DOOR_DIR_UP) {
                send_door_cmd(0);
            }
            if (state != DCM_DOOR_STATE_CLOSED &&
                direction != DCM_DOOR_DIR_DOWN &&
                direction != DCM_DOOR_DIR_UP) {
                cancel_scheduled_close("rescheduling close");
                alert();
                close_scheduled = true;
                k_work_schedule(&close_work, K_MSEC(DOOR_MOVE_ALERT_TICKS));
                LOG_INF("Alerting and scheduled door close in %d ms",
                        DOOR_MOVE_ALERT_TICKS);
            }
            break;
        }

        case PIC_CMD_CLOSE_EXEC: {
            if (close_scheduled && state != DCM_DOOR_STATE_CLOSED &&
                direction != DCM_DOOR_DIR_DOWN &&
                direction != DCM_DOOR_DIR_UP) {
                LOG_INF("Alert period complete, sending CLOSE command to PIC");
                send_door_cmd(0);
            } else {
                LOG_INF(
                    "Skipping scheduled close (scheduled=%d, state=%d, "
                    "dir=%d)",
                    close_scheduled, state, direction);
            }
            close_scheduled = false;
            break;
        }

        case PIC_CMD_STOP: {
            cancel_scheduled_close("STOP command received");
            if (direction != DCM_DOOR_DIR_STOPPED) {
                send_door_cmd(1);
            }
            break;
        }

        case PIC_CMD_LIGHT_ON: {
            LOG_INF("Sending LIGHT ON command to PIC");
            dcm_msg_t msg = {DCM_HEADER_BYTE, 1 + sizeof(dcm_light_cmd_msg_t),
                             next_token(), DCM_MSG_LIGHT_CMD,
                             .payload.light_cmd = {1, 0}};
            send_msg(&msg);
            break;
        }

        case PIC_CMD_LIGHT_OFF: {
            LOG_INF("Sending LIGHT OFF command to PIC");
            dcm_msg_t msg = {DCM_HEADER_BYTE, 1 + sizeof(dcm_light_cmd_msg_t),
                             next_token(), DCM_MSG_LIGHT_CMD,
                             .payload.light_cmd = {0, 0}};
            send_msg(&msg);
            break;
        }

        case PIC_CMD_AUDIO:
            audio_cmd(2);
            break;

        case PIC_CMD_ALERT:
            alert_cmd(2);
            break;

        case PIC_CMD_STATUS_REQ:
            send_door_status_req();
            break;

        case PIC_CMD_0x04:
            cmd_0x04();
            break;

        case PIC_CMD_DOOR_RAW_CLOSE:
            send_door_cmd(0);
            break;

        case PIC_SERIAL_DATA: {
            process_serial_data();
            break;
        }

        default: {
            LOG_ERR("unknown cmd %d", cmd);
            break;
        }
    }
}

static void pic_uart_task(void* p1, void* p2, void* p3) {
    LOG_INF("PIC comm task running");

    if (init_uart() != 0) {
        LOG_ERR("init uart failed");
        return;
    }

    k_work_init_delayable(&close_work, close_work_handler);
    start_uart_read();
    pic_cmd_t cmd;

    self_test_done = true;
    if (!sbl_active) {
        send_door_status_req();
    }

    uint32_t door_poll_tstamp = k_uptime_get_32() - DOOR_POLL_TICKS + 500;
    for (;;) {
        if (sbl_active) {
            k_msleep(100);
            continue;
        }

        uint32_t now = k_uptime_get_32();
        if (read_state == READ_BODY &&
            now > last_read_tick + READ_TIMEOUT_TICKS) {
            LOG_WRN("read timeout");
            start_uart_read();
        }
        if (now - door_poll_tstamp > DOOR_POLL_TICKS) {
            send_door_status_req();
            cmd_0x04();
            door_poll_tstamp = now;
        }

        uint32_t timeout_ms = READ_TIMEOUT_TICKS;
        if (now - door_poll_tstamp < DOOR_POLL_TICKS) {
            uint32_t poll_remain = DOOR_POLL_TICKS - (now - door_poll_tstamp);
            if (poll_remain < timeout_ms) {
                timeout_ms = poll_remain;
            }
        } else {
            timeout_ms = 0;
        }
        if (read_state == READ_BODY) {
            uint32_t elapsed = now - last_read_tick;
            if (elapsed < READ_TIMEOUT_TICKS) {
                uint32_t read_remain = READ_TIMEOUT_TICKS - elapsed;
                if (read_remain < timeout_ms) {
                    timeout_ms = read_remain;
                }
            } else {
                timeout_ms = 0;
            }
        }

        if (k_msgq_get(&pic_queue, &cmd, K_MSEC(timeout_ms)) == 0) {
            pic_process_cmd(cmd);
        }
    }
}

K_THREAD_DEFINE(pic_uart_tid, 1024, pic_uart_task, NULL, NULL, NULL, 7, 0, 0);

int pic_set_light(bool on) {
    if (sbl_active) {
        LOG_WRN("PIC update in progress, rejecting light command");
        return -EBUSY;
    }
    pic_cmd_t cmd = on ? PIC_CMD_LIGHT_ON : PIC_CMD_LIGHT_OFF;
    return k_msgq_put(&pic_queue, &cmd, K_MSEC(1000));
}

bool pic_get_light(void) { return current_light_state; }

bool pic_get_version(uint8_t* major, uint8_t* minor, uint8_t* patch,
                     char* suffix) {
    if (!s_pic_version.received) {
        return false;
    }
    if (major) {
        *major = s_pic_version.major;
    }
    if (minor) {
        *minor = s_pic_version.minor;
    }
    if (patch) {
        *patch = s_pic_version.patch;
    }
    if (suffix) {
        *suffix = s_pic_version.suffix;
    }
    return true;
}

void pic_get_stats(pic_stats_t* stats) {
    if (stats) {
        *stats = s_stats;
    }
}

int pic_send_audio(uint8_t val) {
    ARG_UNUSED(val);
    pic_cmd_t cmd = PIC_CMD_AUDIO;
    return k_msgq_put(&pic_queue, &cmd, K_MSEC(1000));
}

int pic_send_alert(uint8_t val) {
    ARG_UNUSED(val);
    pic_cmd_t cmd = PIC_CMD_ALERT;
    return k_msgq_put(&pic_queue, &cmd, K_MSEC(1000));
}

int pic_send_status_req(void) {
    pic_cmd_t cmd = PIC_CMD_STATUS_REQ;
    return k_msgq_put(&pic_queue, &cmd, K_MSEC(1000));
}

int pic_send_cmd_0x04(void) {
    pic_cmd_t cmd = PIC_CMD_0x04;
    return k_msgq_put(&pic_queue, &cmd, K_MSEC(1000));
}

int pic_send_door_raw(uint8_t val) {
    pic_cmd_t cmd = val ? PIC_CMD_OPEN : PIC_CMD_DOOR_RAW_CLOSE;
    return k_msgq_put(&pic_queue, &cmd, K_MSEC(1000));
}

bool pic_is_updating(void) { return sbl_active; }

int pic_enter_sbl_mode(void) {
    LOG_INF("Entering PIC SBL mode");
    init_uart();
    sbl_active = true;
    cancel_scheduled_close("PIC update started");
    k_msgq_purge(&pic_queue);
    k_msgq_purge(&sbl_rx_q);

#if DT_NODE_EXISTS(DT_NODELABEL(pic_sbl))
    if (device_is_ready(pic_sbl.port)) {
        gpio_pin_configure_dt(&pic_sbl, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&pic_sbl, 1);
    }
#endif
    // In vendor firmware: set_PIC16LF15_SIGNAL_HIGH() (GPIO46=1),
    // then reset_PIC16LF15():
    // set_PIC16LF15_RESET_HOLD() -> GPIO1 = HIGH for 1000ms
    // set_PIC16LF15_RESET_RELEASE() -> GPIO1 = LOW for 1000ms
    // Zephyr pic_rst is GPIO_ACTIVE_HIGH on GPIO1.
    // So 1 = HIGH (RESET), 0 = LOW (RUN).
    gpio_pin_configure_dt(&pic_rst, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&pic_rst, 1);  // Reset hold
    k_msleep(1000);
    gpio_pin_set_dt(&pic_rst, 0);  // Reset release (run)
    k_msleep(1000);

    k_msgq_purge(&sbl_rx_q);

    // Send Handshake Request (0x04)
    uint8_t hs[14] = {
        0x55, 9, 0, DCM_SBL_MSG_HANDSHAKE_REQ, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    hs[13] = sbl_calc_chk_sum(hs, 13);
    log_packet("SBL TX", hs, 14);
    for (int i = 0; i < 14; i++) {
        uart_poll_out(uart_dev, hs[i]);
    }

    // Wait for 0x84 ACK (30 bytes: header 0x55, len 25, seq, type 0x84, 25
    // payload, checksum)
    uint8_t rx_buf[32];
    int rx_cnt = 0;
    uint32_t start = k_uptime_get_32();
    while (rx_cnt < 30 && k_uptime_get_32() - start < 3000) {
        uint8_t b;
        if (k_msgq_get(&sbl_rx_q, &b, K_MSEC(100)) == 0) {
            if (rx_cnt == 0 && b != 0x55) continue;
            rx_buf[rx_cnt++] = b;
        }
    }
    if (rx_cnt > 0) {
        log_packet("SBL RX", rx_buf, rx_cnt);
    }
    if (rx_cnt < 30 || rx_buf[3] != DCM_SBL_RSP_HANDSHAKE_ACK) {
        LOG_ERR("SBL Handshake failed (got %d bytes, type 0x%02x)", rx_cnt,
                rx_cnt >= 4 ? rx_buf[3] : 0);
        pic_exit_sbl_mode();
        return -1;
    }
    LOG_INF("SBL Handshake OK, initiating erase/download");

    // Send Erase / Download Start (0x06)
    uint8_t erase[14] = {0x55, 9,    0,    DCM_SBL_MSG_ERASE_START,
                         0x03, 0xE0, 0x00, 0x55,
                         0xAA, 0x00, 0x04, 0x00,
                         0,    0};
    erase[13] = sbl_calc_chk_sum(erase, 13);
    for (int i = 0; i < 14; i++) {
        uart_poll_out(uart_dev, erase[i]);
    }

    // Wait for 0x86 ACK (14 bytes)
    rx_cnt = 0;
    start = k_uptime_get_32();
    while (rx_cnt < 14 && k_uptime_get_32() - start < 3000) {
        uint8_t b;
        if (k_msgq_get(&sbl_rx_q, &b, K_MSEC(100)) == 0) {
            if (rx_cnt == 0 && b != 0x55) continue;
            rx_buf[rx_cnt++] = b;
        }
    }
    if (rx_cnt < 14 || rx_buf[3] != DCM_SBL_RSP_ERASE_ACK) {
        LOG_ERR("SBL Erase/Start failed (got %d bytes, type 0x%02x)", rx_cnt,
                rx_cnt >= 4 ? rx_buf[3] : 0);
        pic_exit_sbl_mode();
        return -2;
    }
    LOG_INF("SBL ready for data");
    return 0;
}

int pic_sbl_write_chunk(uint32_t word_addr, const uint8_t* data, size_t len) {
    if (!sbl_active || len == 0 || len > 64) {
        return -1;
    }
    k_msgq_purge(&sbl_rx_q);

    uint8_t tx[84];
    tx[0] = 0x55;
    tx[1] = len + 9;
    tx[2] = 0;
    tx[3] = DCM_SBL_MSG_DATA_BLOCK;  // 0x01
    tx[4] = 0x02;                    // subcmd
    tx[5] = (uint8_t)len;
    tx[6] = 0x00;
    tx[7] = 0x55;
    tx[8] = 0xAA;
    tx[9] = (uint8_t)(word_addr & 0xFF);
    tx[10] = (uint8_t)((word_addr >> 8) & 0xFF);
    tx[11] = (uint8_t)((word_addr >> 16) & 0xFF);
    tx[12] = (uint8_t)((word_addr >> 24) & 0xFF);
    memcpy(&tx[13], data, len);
    tx[13 + len] = sbl_calc_chk_sum(tx, len + 13);
    int total_len = len + 14;

    for (int i = 0; i < total_len; i++) {
        uart_poll_out(uart_dev, tx[i]);
    }

    // Wait for 0x81 ACK (15 bytes: 0x55, len=10, tok, 0x81, ... result at [13],
    // chksum at [14])
    uint8_t rx_buf[16];
    int rx_cnt = 0;
    uint32_t start = k_uptime_get_32();
    while (rx_cnt < 15 && k_uptime_get_32() - start < 1500) {
        uint8_t b;
        if (k_msgq_get(&sbl_rx_q, &b, K_MSEC(50)) == 0) {
            if (rx_cnt == 0 && b != 0x55) continue;
            rx_buf[rx_cnt++] = b;
        }
    }
    if (rx_cnt < 15 || rx_buf[3] != DCM_SBL_RSP_DATA_ACK || rx_buf[13] != 1) {
        LOG_ERR(
            "SBL Write chunk failed at 0x%04x (got %d bytes, type 0x%02x, res "
            "%d)",
            word_addr, rx_cnt, rx_cnt >= 4 ? rx_buf[3] : 0,
            rx_cnt >= 14 ? rx_buf[13] : 0);
        return -2;
    }

    // Delay 10 ms before next block, matching original firmware vTaskDelay(10)
    k_msleep(10);
    return 0;
}

int pic_sbl_finish(void) {
    if (!sbl_active) {
        return 0;
    }
    LOG_INF("Sending SBL EOF command");
    k_msgq_purge(&sbl_rx_q);

    // Send EOF (0x08)
    uint8_t eof[14] = {
        0x55, 9, 0, DCM_SBL_MSG_EOF, 0x08, 0x00, 0x38, 0x00, 0x00, 0x00, 0x04,
        0x00, 0, 0};
    eof[13] = sbl_calc_chk_sum(eof, 13);
    log_packet("SBL EOF TX", eof, 14);
    for (int i = 0; i < 14; i++) {
        uart_poll_out(uart_dev, eof[i]);
    }

    // Wait for 0x88 ACK (16 bytes)
    uint8_t rx_buf[20];
    int rx_cnt = 0;
    uint32_t start = k_uptime_get_32();
    while (rx_cnt < 16 && k_uptime_get_32() - start < 2000) {
        uint8_t b;
        if (k_msgq_get(&sbl_rx_q, &b, K_MSEC(50)) == 0) {
            if (rx_cnt == 0 && b != 0x55) continue;
            rx_buf[rx_cnt++] = b;
        }
    }
    if (rx_cnt > 0) {
        log_packet("SBL EOF RX", rx_buf, rx_cnt);
    }
    if (rx_cnt < 16 || rx_buf[3] != DCM_SBL_RSP_EOF_ACK) {
        LOG_ERR(
            "SBL EOF ACK mismatch/timeout (got %d bytes, type 0x%02x), update "
            "failed",
            rx_cnt, rx_cnt >= 4 ? rx_buf[3] : 0);
        pic_exit_sbl_mode();
        return -1;
    }
    LOG_INF("SBL EOF acknowledged successfully");

    pic_exit_sbl_mode();
    return 0;
}

int pic_exit_sbl_mode(void) {
    LOG_INF("Exiting SBL mode, rebooting PIC");
#if DT_NODE_EXISTS(DT_NODELABEL(pic_sbl))
    if (device_is_ready(pic_sbl.port)) {
        gpio_pin_configure_dt(&pic_sbl, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&pic_sbl, 0);
    }
#endif
    gpio_pin_configure_dt(&pic_rst, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&pic_rst, 1);
    k_msleep(1000);
    gpio_pin_set_dt(&pic_rst, 0);
    k_msleep(1000);

    sbl_active = false;
    k_msgq_purge(&pic_queue);
    k_msgq_purge(&sbl_rx_q);
    start_uart_read();
    s_log_pic_version = true;
    pos = 0;
    down_limit = 0;
    up_limit = 0;
    send_door_status_req();
    return 0;
}

#else

void pic_uart_init(void) { LOG_INF("pic_uart stub initialized"); }

int pic_set_light(bool on) {
    ARG_UNUSED(on);
    return 0;
}

bool pic_get_light(void) { return false; }

bool pic_is_updating(void) { return false; }

int pic_enter_sbl_mode(void) { return -ENOTSUP; }

int pic_sbl_write_chunk(uint32_t word_addr, const uint8_t* data, size_t len) {
    ARG_UNUSED(word_addr);
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    return -ENOTSUP;
}

int pic_sbl_finish(void) { return -ENOTSUP; }

int pic_exit_sbl_mode(void) { return -ENOTSUP; }

bool pic_get_version(uint8_t* major, uint8_t* minor, uint8_t* patch,
                     char* suffix) {
    ARG_UNUSED(major);
    ARG_UNUSED(minor);
    ARG_UNUSED(patch);
    ARG_UNUSED(suffix);
    return false;
}

void pic_get_stats(pic_stats_t* stats) {
    if (stats) {
        memset(stats, 0, sizeof(*stats));
    }
}

int pic_send_audio(uint8_t val) {
    ARG_UNUSED(val);
    return -ENOTSUP;
}

int pic_send_alert(uint8_t val) {
    ARG_UNUSED(val);
    return -ENOTSUP;
}

int pic_send_status_req(void) { return -ENOTSUP; }

int pic_send_cmd_0x04(void) { return -ENOTSUP; }

int pic_send_door_raw(uint8_t val) {
    ARG_UNUSED(val);
    return -ENOTSUP;
}

#endif

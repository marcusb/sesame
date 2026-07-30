#include "pic_uart.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "controller.h"
#include "idcm_msg.h"

LOG_MODULE_REGISTER(pic_uart, LOG_LEVEL_INF);

K_MSGQ_DEFINE(pic_queue, sizeof(pic_cmd_t), 10, 4);

#define READ_TIMEOUT_TICKS 10000
#define DOOR_POLL_TICKS (30 * 1000)
#define DOOR_MOVE_ALERT_TICKS 6000
#define STATE_UPDATE_INTERVAL (30 * 1000)

static bool self_test_done;
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

static uint8_t calc_chk_sum(uint8_t* p, int n) {
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
    bool update = state != new_state || direction != dir;
    state = new_state;
    direction = dir;
    LOG_INF("door status: state=%d, dir=%d, pos=%u, down_lim=%u, up_lim=%u",
            state, dir, raw_pos, down_limit, up_limit);
    if (down_limit != up_limit) {
        int32_t val = 100 * (int32_t)(up_limit - raw_pos) /
                      (int32_t)(up_limit - down_limit);
        if (val < 0) val = 0;
        if (val > 100) val = 100;
        pos = val;
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
        case DCM_MSG_DOOR_STATUS_UPDATE: {
            const dcm_door_status_update_msg_t* p = &msg->payload.door_status;
            down_limit = p->down_limit;
            up_limit = p->up_limit;
            door_state_update(p->state, p->direction, p->pos);
            break;
        }

        case DCM_MSG_AUDIO_ACK:
            self_test_done = true;
            break;

        case DCM_MSG_SENSOR_VERSION: {
            const dcm_sensor_version_msg_t* p = &msg->payload.sensor_version;
            door_state_update(p->state, p->direction, p->pos);
            break;
        }

        case DCM_MSG_OPS_EVENT: {
            const dcm_ops_event_msg_t* p = &msg->payload.ops_event;
            LOG_DBG("ops event %d", p->event);
            break;
        }

        default:
            break;
    }
}

static uint32_t last_read_tick;

static void start_uart_read() {
    read_state = READ_HEADER;
    rx_idx = 0;
    expected_len = 4;
    gpio_pin_set_dt(&pic_wake, 1);
    uart_irq_rx_enable(uart_dev);
}

static void process_serial_data() {
    const dcm_msg_t* msg = (const dcm_msg_t*)rx_buf;
    if (read_state == READ_HEADER) {
        last_read_tick = k_uptime_get_32();
        if (msg->header != 0x55) {
            LOG_DBG("unexpected byte %02x", msg->header);
            goto reset;
        }
        gpio_pin_set_dt(&pic_wake, 0);
        if (msg->len + 5 > sizeof(rx_buf)) {
            LOG_INF("invalid msg length %d", msg->len);
            goto reset;
        }
        read_state = READ_BODY;
        expected_len = msg->len + 5;
        uart_irq_rx_enable(uart_dev);
    } else if (read_state == READ_BODY) {
        LOG_HEXDUMP_DBG(rx_buf, msg->len + 5, "RX:");
        uint8_t chksum = calc_chk_sum(rx_buf, msg->len + 4);
        if (chksum == rx_buf[msg->len + 4]) {
            handle_msg(msg);
        } else {
            LOG_WRN("chksum mismatch (got 0x%02x, expected 0x%02x)",
                    rx_buf[msg->len + 4], chksum);
        }
    reset:
        start_uart_read();
    }
}

static void uart_cb(const struct device* dev, void* user_data) {
    uart_irq_update(dev);

    if (uart_irq_rx_ready(dev)) {
        int recv_len =
            uart_fifo_read(dev, rx_buf + rx_idx, expected_len - rx_idx);
        if (recv_len > 0) {
            rx_idx += recv_len;
            if (rx_idx == expected_len) {
                uart_irq_rx_disable(dev);
                pic_cmd_t cmd = PIC_SERIAL_DATA;
                k_msgq_put(&pic_queue, &cmd, K_NO_WAIT);
            }
        }
    }
}

static int init_uart(void) {
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
    gpio_pin_configure_dt(&pic_wake, GPIO_OUTPUT_INACTIVE);

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);

    k_msleep(25);
    gpio_pin_set_dt(&pic_rst, 0);
    k_msleep(5);

    return 0;
}

static void send_msg(dcm_msg_t* msg) {
    int frame_len = msg->len + 5;
    uint8_t* p = (uint8_t*)msg;
    msg->payload.buf[msg->len] = calc_chk_sum(p, frame_len);
    LOG_HEXDUMP_DBG(p, frame_len, "TX:");
    for (int i = 0; i < frame_len; i++) {
        uart_poll_out(uart_dev, p[i]);
    }
}

static void alert_cmd(uint8_t val) {
    dcm_msg_t msg = {DCM_HEADER_BYTE, sizeof(dcm_alert_cmd_msg_t), next_token(),
                     DCM_MSG_ALERT_CMD, .payload.alert_cmd = {val, 5, 0}};
    send_msg(&msg);
}

static void audio_cmd(uint8_t val) {
    dcm_msg_t msg = {DCM_HEADER_BYTE, sizeof(dcm_audio_cmd_msg_t), next_token(),
                     DCM_MSG_AUDIO_CMD,
                     .payload.audio_cmd = {val, self_test_done ? 5 : 0, 0}};
    send_msg(&msg);
}

static void post_test() { audio_cmd(5); }
static void sound_buzzer() { audio_cmd(1); }

static void alert() {
    audio_cmd(2);
    alert_cmd(1);
}

static void send_door_cmd(uint8_t val) {
    LOG_INF("Sending door command to PIC: %s (val=%u)", val ? "OPEN" : "CLOSE",
            val);
    dcm_msg_t msg = {DCM_HEADER_BYTE, sizeof(dcm_door_cmd_msg_t), next_token(),
                     DCM_MSG_DOOR_CMD, .payload.door_cmd = {val, 0}};
    send_msg(&msg);
}

static void send_door_status_req() {
    dcm_msg_t msg = {
        DCM_HEADER_BYTE, sizeof(dcm_door_status_req_msg_t), next_token(),
        DCM_MSG_DOOR_STATUS_REQUEST,
        .payload.door_status_req = {0, 0, 0, pos, up_limit, down_limit}};
    send_msg(&msg);
}

static void cmd_0x04() {
    dcm_msg_t msg = {DCM_HEADER_BYTE, sizeof(dcm_cmd_0x04_msg_t), next_token(),
                     DCM_MSG_0x04};
    send_msg(&msg);
}

static void pic_uart_task(void* p1, void* p2, void* p3) {
    LOG_INF("PIC comm task running");

    if (init_uart() != 0) {
        LOG_ERR("init uart failed");
        return;
    }

    start_uart_read();
    pic_cmd_t cmd;
    post_test();
    while (!self_test_done) {
        if (k_msgq_get(&pic_queue, &cmd, K_FOREVER) == 0 &&
            cmd == PIC_SERIAL_DATA) {
            process_serial_data();
        }
    }
    sound_buzzer();

    uint32_t door_poll_tstamp = 0;
    uint32_t door_move_tstamp = 0;
    pic_cmd_t queued_cmd = 0;
    for (;;) {
        uint32_t now = k_uptime_get_32();
        if (read_state == READ_BODY &&
            now > last_read_tick + READ_TIMEOUT_TICKS) {
            LOG_WRN("read timeout");
            uart_irq_rx_disable(uart_dev);
            start_uart_read();
        }
        if (now - door_poll_tstamp > DOOR_POLL_TICKS) {
            send_door_status_req();
            cmd_0x04();
            door_poll_tstamp = now;
        }
        if (queued_cmd && now > door_move_tstamp) {
            send_door_cmd(queued_cmd == PIC_CMD_OPEN ? 1 : 0);
            queued_cmd = 0;
        }
        if (k_msgq_get(&pic_queue, &cmd, K_MSEC(READ_TIMEOUT_TICKS)) == 0) {
            switch (cmd) {
                case PIC_CMD_OPEN:
                    if (direction == DCM_DOOR_DIR_DOWN) {
                        send_door_cmd(1);
                        queued_cmd = PIC_CMD_OPEN;
                        door_move_tstamp = now + 1000;
                    } else if (state != DCM_DOOR_STATE_OPEN &&
                               direction != DCM_DOOR_DIR_UP) {
                        send_door_cmd(1);
                        queued_cmd = 0;
                    }
                    break;

                case PIC_CMD_CLOSE:
                    if (direction == DCM_DOOR_DIR_UP) {
                        send_door_cmd(0);
                    }
                    if (state != DCM_DOOR_STATE_CLOSED &&
                        direction != DCM_DOOR_DIR_DOWN) {
                        queued_cmd = cmd;
                        door_move_tstamp = now + DOOR_MOVE_ALERT_TICKS;
                        alert();
                    }
                    break;

                case PIC_CMD_STOP:
                    if (direction != DCM_DOOR_DIR_STOPPED) {
                        send_door_cmd(1);
                    }
                    queued_cmd = 0;
                    break;

                case PIC_SERIAL_DATA:
                    process_serial_data();
                    break;

                default:
                    LOG_ERR("unknown cmd %d", cmd);
            }
        }
    }
}

K_THREAD_DEFINE(pic_uart_tid, 1024, pic_uart_task, NULL, NULL, NULL, 7, 0, 0);

#include "pic_uart.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// wmsdk
#include "mdev_gpio.h"
#include "mdev_uart.h"
#include "rtc.h"

// Application
#include "app_logging.h"
#include "controller.h"
#include "idcm_msg.h"
#include "util.h"

#define RX_BUF_SIZE 256
#define READ_TIMEOUT_TICKS pdMS_TO_TICKS(10000)
#define DOOR_POLL_TICKS pdMS_TO_TICKS(5000)
#define DOOR_MOVE_ALERT_TICKS pdMS_TO_TICKS(6000)
#define STATE_UPDATE_INTERVAL pdMS_TO_TICKS(60 * 1000)

typedef enum { READ_START = 0, READ_LENGTH = 1, READ_BODY = 2 } read_state_t;

static mdev_t *gpio_dev;
static mdev_t *uart_dev;
static bool self_test_done;
static door_open_state_t state = DCM_DOOR_STATE_UNKNOWN;
static door_direction_t direction = DCM_DOOR_DIR_UNKNOWN;
static uint16_t down_limit;
static uint16_t up_limit;
static TickType_t last_state_pub_time;

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

static uint8_t calc_chk_sum(uint8_t *p, int n) {
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
                              uint16_t position) {
    bool update = state != new_state || direction != dir;
    state = new_state;
    direction = dir;
    LogInfo(("door status: state=%d, dir=%d, pos=%u, down_lim=%u, up_lim=%u",
             state, dir, position, down_limit, up_limit));
    uint16_t pos = 0;
    if (down_limit != up_limit) {
        pos = 100 * (up_limit - position) / (float)(up_limit - down_limit);
    }
    if (pos < 0) {
        pos = 0;
    }
    if (pos > 100) {
        pos = 100;
    }
    TickType_t now = xTaskGetTickCount();
    if (update || dir != DCM_DOOR_DIR_STOPPED ||
        now > last_state_pub_time + STATE_UPDATE_INTERVAL) {
        last_state_pub_time = now;
        ctrl_msg_t state_upd = {CTRL_MSG_DOOR_STATE_UPDATE,
                                {.door_state = {state, dir, pos}}};
        xQueueSendToBack(ctrl_queue, &state_upd, 0);
    }
}

static void handle_msg(const dcm_msg_t *msg) {
    switch (msg->type) {
        case DCM_MSG_DOOR_STATUS_UPDATE: {
            const dcm_door_status_update_msg_t *p = &msg->payload.door_status;
            down_limit = p->down_limit;
            up_limit = p->up_limit;
            door_state_update(p->state, p->direction, p->pos);
            break;
        }

        case DCM_MSG_AUDIO_ACK:
            self_test_done = true;
            break;

        case DCM_MSG_SENSOR_VERSION: {
            const dcm_sensor_version_msg_t *p = &msg->payload.sensor_version;
            door_state_update(p->state, p->direction, p->pos);
            // LogDebug(("DCM sensor fw version %d.%d.%d%c", p->major, p->minor,
            //           p->patch, p->suffix));
            break;
        }

        case DCM_MSG_OPS_EVENT: {
            const dcm_ops_event_msg_t *p = &msg->payload.ops_event;
            LogDebug(("ops event %d", p->event));
            break;
        }

        default:
            // ignore
    }
}

static void read_uart() {
    static uint8_t buf[80];
    static int n = 0;
    static int msg_len = 0;
    static int more = 1;
    static uint8_t *p = buf;
    static read_state_t state = READ_START;
    static TickType_t last_read_tick;

    TickType_t now = xTaskGetTickCount();
    if (state != READ_START && now > last_read_tick + READ_TIMEOUT_TICKS) {
        LogWarn(("read timeout, read %d chars, waiting for %d more", n, more));
        goto reset;
    }
    while (true) {
        int bytes_read = uart_drv_read(uart_dev, p, more);
        if (bytes_read == 0) {
            return;
        }
        n += bytes_read;
        if (state == READ_START) {
            if (*p++ != 0x55) {
                LogDebug(("unexpected byte %02x", *(p - 1)));
                goto reset;
            }
            last_read_tick = now;
            gpio_drv_write(gpio_dev, GPIO_49, GPIO_IO_LOW);
            state = READ_LENGTH;
        } else if (state == READ_LENGTH) {
            msg_len = *p++ + 5;
            if (msg_len >= sizeof(buf)) {
                LogInfo(("invalid msg length %d", msg_len));
                goto reset;
            }
            more = msg_len - n;
            state = READ_BODY;
        } else {
            p += bytes_read;
            more -= bytes_read;
            if (more <= 0) {
                debug_hexdump('R', buf, n);
                uint8_t chksum = calc_chk_sum(buf, n - 1);
                if (chksum != buf[n - 1]) {
                    LogWarn(("chksum mismatch (got 0x%02x, expected 0x%02x)",
                             buf[n - 1], chksum));
                }
                handle_msg((dcm_msg_t *)buf);
            reset:
                gpio_drv_write(gpio_dev, GPIO_49, GPIO_IO_HIGH);
                n = 0;
                msg_len = 0;
                more = 1;
                p = buf;
                state = READ_START;
            }
        }
    }
}

static int init_gpio(void) {
    gpio_dev = gpio_drv_open("MDEV_GPIO");
    if (gpio_dev == NULL) {
        LogError(("gpio_drv_open failed"));
        return -1;
    }
    return 0;
}

static int init_uart(void) {
    int err = uart_drv_init(UART1_ID, UART_8BIT);
    if (err) {
        LogError(("uart_drv_init failed (%d)\n", err));
        return -1;
    }

    uart_drv_rxbuf_size(UART1_ID, RX_BUF_SIZE);
    vTaskDelay(25);

    uart_dev = uart_drv_open(UART1_ID, 115200);
    if (uart_dev == NULL) {
        LogError(("uart_drv_open failed (%d)\n", err));
        return -1;
    }

    gpio_drv_write(gpio_dev, GPIO_48, GPIO_IO_LOW);
    vTaskDelay(5);

    return 0;
}

static void uart_write(const uint8_t *buf, int n) {
    debug_hexdump('T', buf, n);
    while (n > 0) {
        int n_written = uart_drv_write(uart_dev, buf, n);
        n -= n_written;
        buf += n_written;
        vTaskDelay(1);
    }
}

static void send_msg(dcm_msg_t *msg) {
    // 4 bytes header, 1 byte checksum
    int frame_len = msg->len + 5;
    uint8_t *p = (uint8_t *)msg;
    msg->payload.buf[msg->len] = calc_chk_sum(p, frame_len);
    uart_write(p, frame_len);
}

static void alert_cmd(uint8_t val) {
    dcm_msg_t msg = {DCM_HEADER_BYTE,
                     sizeof(dcm_alert_cmd_msg_t),
                     next_token(),
                     DCM_MSG_ALERT_CMD,
                     {.alert_cmd = {val, 5}}};
    send_msg(&msg);
}

static void audio_cmd(uint8_t val) {
    dcm_msg_t msg = {DCM_HEADER_BYTE,
                     sizeof(dcm_audio_cmd_msg_t),
                     next_token(),
                     DCM_MSG_AUDIO_CMD,
                     {.audio_cmd = {val, self_test_done ? 5 : 0, 0}}};
    send_msg(&msg);
}

static void post_test() {
    audio_cmd(5);
    LogInfo(("PIC POST test"));
    while (!self_test_done) {
        read_uart();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    LogInfo(("PIC POST test successful"));
}

static void sound_buzzer() { audio_cmd(1); }

static void alert() {
    audio_cmd(2);
    alert_cmd(1);
}

static void send_door_cmd(uint8_t val) {
    dcm_msg_t msg = {DCM_HEADER_BYTE,
                     sizeof(dcm_door_cmd_msg_t),
                     next_token(),
                     DCM_MSG_DOOR_CMD,
                     {.door_cmd = {val, 0}}};
    send_msg(&msg);
}

static void send_door_status_req() {
    dcm_msg_t msg = {DCM_HEADER_BYTE,
                     sizeof(dcm_door_status_req_msg_t),
                     next_token(),
                     DCM_MSG_DOOR_STATUS_REQUEST,
                     {.door_status_req = {rtc_time_get()}}};
    send_msg(&msg);
}

void pic_uart_task(void *const params) {
    LogInfo(("PIC comm task running"));
    QueueHandle_t queue = (QueueHandle_t)params;

    if (init_gpio() != 0 || init_uart() != 0) {
        vTaskDelete(NULL);
        return;
    }

    post_test();
    sound_buzzer();

    pic_cmd_t cmd;
    TickType_t door_poll_tstamp = 0;
    TickType_t door_move_tstamp = 0;
    pic_cmd_t queued_cmd = 0;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if (now - door_poll_tstamp > DOOR_POLL_TICKS) {
            send_door_status_req();
            // cmd_0x04();
            door_poll_tstamp = now;
        }
        if (queued_cmd && now > door_move_tstamp) {
            send_door_cmd(queued_cmd == PIC_CMD_OPEN ? 1 : 0);
            queued_cmd = 0;
        }
        read_uart();
        if (xQueueReceive(queue, &cmd, pdMS_TO_TICKS(5)) == pdPASS) {
            switch (cmd) {
                case PIC_CMD_OPEN:
                    if (direction == DCM_DOOR_DIR_DOWN) {
                        // first stop the door
                        send_door_cmd(1);
                        // then open after a short delay
                        queued_cmd = PIC_CMD_OPEN;
                        door_move_tstamp = now + pdMS_TO_TICKS(1000);
                    } else if (state != DCM_DOOR_STATE_OPEN &&
                               direction != DCM_DOOR_DIR_UP) {
                        send_door_cmd(1);
                        queued_cmd = 0;
                    }
                    break;

                case PIC_CMD_CLOSE:
                    if (direction == DCM_DOOR_DIR_UP) {
                        // first stop the door
                        send_door_cmd(0);
                    }
                    if (state != DCM_DOOR_STATE_CLOSED &&
                        direction != DCM_DOOR_DIR_DOWN) {
                        queued_cmd = cmd;
                        door_move_tstamp = now + DOOR_MOVE_ALERT_TICKS;
                        alert();
                    }
                    break;

                default:
                    LogError(("unknown cmd %d", cmd));
            }
        }
    }
}


#include "pic_uart.h"

#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "idcm_msg.h"
#include "mdev_gpio.h"
#include "mdev_uart.h"
#include "mqtt.h"
#include "queue.h"
#include "rtc.h"
#include "task.h"

#define READ_TIMEOUT_TICKS pdMS_TO_TICKS(10000)
#define RX_BUF_SIZE 256
#define DOOR_POLL_TICKS pdMS_TO_TICKS(5000)

typedef enum { READ_START = 0, READ_LENGTH = 1, READ_BODY = 2 } read_state_t;

static mdev_t *gpio_dev;
static mdev_t *uart_dev;
static uint8_t send_buf[32];

static void debug_hexdump(char dir, const uint8_t *data, unsigned len) {
    static char s[3 * 16 + 4];
    char *p = s;

    if (len == 0) {
        return;
    }
    p += sprintf(p, "%c: %02x ", dir, data[0]);
    for (int i = 1; i < len; i++) {
        if ((i & 0x0f) == 0) {
            *p = '\0';
            LogDebug((s));
            p = s;
            p += sprintf(p, "   ");
        }
        p += sprintf(p, "%02x ", data[i]);
    }
    *p = '\0';
    LogDebug((s));
}

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

void handle_msg(const dcm_msg_t *msg) {
    switch (msg->type) {
        case DCM_MSG_DOOR_STATUS_RESPONSE: {
            const dcm_door_status_msg_t *p = &msg->payload.door_status;
            LogInfo((
                "door status: state=%d, dir=%d, pos=%d, up_lim=%d, down_lim=%d",
                p->state, p->direction, p->pos, p->up_limit, p->down_limit));
            publish_state(p);
            break;
        }

        case DCM_AUDIO_RESPONSE: {
            vTaskDelay(2);
            break;
        }

        case DCM_MSG_SENSOR_VERSION: {
            const dcm_sensor_version_msg_t *p = &msg->payload.sensor_version;
            LogInfo(("DCM sensor fw version %d.%d.%d%c", p->major, p->minor,
                     p->patch, p->suffix));
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

static void cmd_0x12(uint8_t val) {
    memset(send_buf, 0, 8);
    send_buf[0] = 0x55;
    send_buf[1] = 3;
    send_buf[2] = next_token();
    send_buf[3] = 0x12;
    send_buf[4] = val;
    send_buf[7] = calc_chk_sum(send_buf, 8);
    uart_write(send_buf, 8);
}

static void post_test() { cmd_0x12(5); }

static void sound_buzzer() { cmd_0x12(1); }

static void buzzer_alert() { cmd_0x12(2); }

static void cmd_0x11(uint8_t val) {
    memset(send_buf, 0, 8);
    send_buf[0] = 0x55;
    send_buf[1] = 3;
    send_buf[2] = next_token();
    send_buf[3] = 0x11;
    send_buf[4] = val;
    send_buf[7] = calc_chk_sum(send_buf, 8);
    uart_write(send_buf, 8);
}

static void cmd_0x04() {
    memset(send_buf, 0, 14);
    send_buf[0] = 0x55;
    send_buf[1] = 9;
    send_buf[2] = next_token();
    send_buf[3] = 0x04;
    send_buf[13] = calc_chk_sum(send_buf, 14);
    uart_write(send_buf, 14);
}

static void send_door_cmd(uint8_t val) {
    memset(send_buf, 0, 7);
    send_buf[0] = 0x55;
    send_buf[1] = 2;
    send_buf[2] = next_token();
    send_buf[3] = 0x10;
    send_buf[4] = val;
    send_buf[5] = 0;
    send_buf[6] = calc_chk_sum(send_buf, 7);
    uart_write(send_buf, 7);
}

static void send_door_status_msg(int16_t last_pos, int16_t up_limit,
                                 int16_t down_limit) {
    memset(send_buf, 0, 17);
    dcm_msg_t *msg = (dcm_msg_t *)send_buf;
    msg->header = 0x55;
    msg->len = sizeof(dcm_door_status_msg_t);
    msg->seq = next_token();
    msg->type = DCM_MSG_DOOR_STATUS_REQUEST;
    msg->payload.door_status.time = rtc_time_get();
    msg->payload.door_status.pos = last_pos;
    msg->payload.door_status.up_limit = up_limit;
    msg->payload.door_status.down_limit = down_limit;
    send_buf[16] = calc_chk_sum(send_buf, 17);
    uart_write(send_buf, 17);
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
    cmd_0x11(1);

    pic_cmd_t cmd;
    TickType_t door_poll_tstamp = 0;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if (now - door_poll_tstamp > DOOR_POLL_TICKS) {
            send_door_status_msg(0, 0, 0);
            // cmd_0x04();
            door_poll_tstamp = now;
        }
        read_uart();
        if (xQueueReceive(queue, &cmd, pdMS_TO_TICKS(5)) == pdPASS) {
            switch (cmd) {
                case PIC_CMD_OPEN:
                    send_door_cmd(1);
                    break;
                case PIC_CMD_CLOSE:
                    send_door_cmd(0);
                    break;
                default:
                    LogError(("unknown cmd %d", cmd));
            }
        }
    }
}

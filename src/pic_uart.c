#include "pic_uart.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// wmsdk
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"

// Application
#include "app_logging.h"
#include "controller.h"
#include "gpio.h"
#include "idcm_msg.h"
#include "util.h"

#define READ_TIMEOUT_TICKS pdMS_TO_TICKS(10000)
#define DOOR_POLL_TICKS pdMS_TO_TICKS(30 * 1000)
#define DOOR_MOVE_ALERT_TICKS pdMS_TO_TICKS(6000)
#define STATE_UPDATE_INTERVAL pdMS_TO_TICKS(60 * 1000)

static volatile QueueHandle_t pic_queue;
static bool self_test_done;
static door_open_state_t state = DCM_DOOR_STATE_UNKNOWN;
static door_direction_t direction = DCM_DOOR_DIR_UNKNOWN;
static uint16_t pos;
static uint16_t down_limit;
static uint16_t up_limit;
static TickType_t last_state_pub_time;
static uart_handle_t uart_hnd;

#define RX_BUF_SIZE 64
static uint8_t rx_buf[RX_BUF_SIZE];
typedef enum { READ_HEADER, READ_BODY } read_state_t;
static read_state_t read_state;

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
                              uint16_t raw_pos) {
    bool update = state != new_state || direction != dir;
    state = new_state;
    direction = dir;
    LogInfo(("door status: state=%d, dir=%d, pos=%u, down_lim=%u, up_lim=%u",
             state, dir, raw_pos, down_limit, up_limit));
    if (down_limit != up_limit) {
        pos = 100 * (up_limit - raw_pos) / (float)(up_limit - down_limit);
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

static pic_cmd_t cmd = PIC_SERIAL_DATA;
static BaseType_t task_to_wake = pdFALSE;

static void uart_cb(UART_Type *base, uart_handle_t *handle, status_t status,
                    void *user_data) {
    if (status == kStatus_UART_RxIdle) {
        xQueueSendToBackFromISR(pic_queue, &cmd, &task_to_wake);
        portYIELD_FROM_ISR(task_to_wake);
    }
}

static TickType_t last_read_tick;

static void start_uart_read() {
    read_state = READ_HEADER;
    gpio_set(49);
    uart_transfer_t xfer = {rx_buf, 4};
    UART_TransferReceiveNonBlocking(UART1, &uart_hnd, &xfer, NULL);
}

static void process_serial_data() {
    const dcm_msg_t *msg = (const dcm_msg_t *)rx_buf;
    if (read_state == READ_HEADER) {
        last_read_tick = xTaskGetTickCount();
        // we have the first 4 bytes
        if (msg->header != 0x55) {
            LogDebug(("unexpected byte %02x", msg->header));
            goto reset;
        }
        gpio_clear(49);
        if (msg->len + 5 > sizeof(rx_buf)) {
            LogInfo(("invalid msg length %d", msg->len));
            goto reset;
        }
        read_state = READ_BODY;
        // read message and checksum
        uart_transfer_t xfer = {rx_buf + 4, msg->len + 1};
        UART_TransferReceiveNonBlocking(UART1, &uart_hnd, &xfer, NULL);
    } else if (read_state == READ_BODY) {
        // we have a complete PDU
        debug_hexdump('R', rx_buf, msg->len + 5);
        uint8_t chksum = calc_chk_sum(rx_buf, msg->len + 4);
        if (chksum == rx_buf[msg->len + 4]) {
            handle_msg(msg);
        } else {
            LogWarn(("chksum mismatch (got 0x%02x, expected 0x%02x)",
                     rx_buf[msg->len + 4], chksum));
        }
    reset:
        start_uart_read();
    }
}

static int init_uart(void) {
    CLOCK_AttachClk(kSYS_CLK_to_SLOW_UART1);
    // need to wait a bit before calling UART_Init, or the clock frequency
    // will not have updated and the UART init fails
    vTaskDelay(25);

    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enable = true;
    status_t res = UART_Init(UART1, &config, CLOCK_GetUartClkFreq(1));
    if (res != kStatus_Success) {
        LogError(("uart init failed %d", res));
    }
    UART_TransferCreateHandle(UART1, &uart_hnd, uart_cb, NULL);
    // lower interrupt priority so we can call FreeRTOS syscalls from ISR
    NVIC_SetPriority(UART1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    vTaskDelay(25);
    gpio_clear(48);
    vTaskDelay(5);

    return 0;
}

static void send_msg(dcm_msg_t *msg) {
    // 4 bytes header, 1 byte checksum
    int frame_len = msg->len + 5;
    uint8_t *p = (uint8_t *)msg;
    msg->payload.buf[msg->len] = calc_chk_sum(p, frame_len);
    debug_hexdump('T', p, frame_len);
    status_t res = UART_WriteBlocking(UART1, p, frame_len);
    if (res != kStatus_Success) {
        LogDebug(("serial write failed: %d", res));
    }
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

static void post_test() { audio_cmd(5); }

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
                     // {.door_status_req = {rtc_time_get(), 0, 0, pos,
                     // up_limit, down_limit}}};
                     {.door_status_req = {0, 0, 0, pos, up_limit, down_limit}}};
    send_msg(&msg);
}

static void cmd_0x04() {
    dcm_msg_t msg = {DCM_HEADER_BYTE, sizeof(dcm_cmd_0x04_msg_t), next_token(),
                     DCM_MSG_0x04};
    send_msg(&msg);
}

void pic_uart_task(void *const params) {
    LogInfo(("PIC comm task running"));
    pic_queue = (QueueHandle_t)params;

    if (init_uart() != 0) {
        LogError(("init uart failed"));
        vTaskDelete(NULL);
        return;
    }

    start_uart_read();
    pic_cmd_t cmd;
    post_test();
    while (!self_test_done) {
        if (xQueueReceive(pic_queue, &cmd, portMAX_DELAY) == pdPASS &&
            cmd == PIC_SERIAL_DATA) {
            process_serial_data();
        }
    }
    sound_buzzer();

    TickType_t door_poll_tstamp = 0;
    TickType_t door_move_tstamp = 0;
    pic_cmd_t queued_cmd = 0;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if (read_state == READ_BODY &&
            now > last_read_tick + READ_TIMEOUT_TICKS) {
            LogWarn(("read timeout"));
            UART_TransferAbortReceive(UART1, &uart_hnd);
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
        if (xQueueReceive(pic_queue, &cmd, READ_TIMEOUT_TICKS) == pdPASS) {
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

                case PIC_SERIAL_DATA:
                    process_serial_data();
                    break;

                default:
                    LogError(("unknown cmd %d", cmd));
            }
        }
    }
}

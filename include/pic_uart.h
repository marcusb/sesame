#pragma once

#include <zephyr/kernel.h>

void pic_uart_init(void);

typedef enum pic_cmd {
    PIC_CMD_UNKNOWN = 0,
    PIC_CMD_OPEN = 1,
    PIC_CMD_CLOSE = 2,
    PIC_SERIAL_DATA = 3,
    PIC_CMD_STOP = 4,
    PIC_CMD_CLOSE_EXEC = 5,
    PIC_CMD_LIGHT_ON = 6,
    PIC_CMD_LIGHT_OFF = 7,
    PIC_CMD_AUDIO = 8,
    PIC_CMD_ALERT = 9,
    PIC_CMD_STATUS_REQ = 10,
    PIC_CMD_0x04 = 11,
    PIC_CMD_DOOR_RAW_CLOSE = 12,
} pic_cmd_t;

extern struct k_msgq pic_queue;

int pic_set_light(bool on);
bool pic_get_light(void);
bool pic_is_updating(void);

int pic_enter_sbl_mode(void);
int pic_sbl_write_chunk(uint32_t word_addr, const uint8_t* data, size_t len);
int pic_sbl_finish(void);
int pic_exit_sbl_mode(void);

typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    char suffix;
    bool received;
} pic_version_info_t;

typedef struct {
    uint32_t door_ack_count;
    uint32_t alert_ack_count;
    uint32_t audio_ack_count;
    uint32_t light_ack_count;
    uint32_t version_count;
    uint32_t status_count;
    uint32_t ops_event_count;
} pic_stats_t;

bool pic_get_version(uint8_t* major, uint8_t* minor, uint8_t* patch,
                     char* suffix);
void pic_get_stats(pic_stats_t* stats);
int pic_send_audio(uint8_t val);
int pic_send_alert(uint8_t val);
int pic_send_status_req(void);
int pic_send_cmd_0x04(void);
int pic_send_door_raw(uint8_t val);

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
} pic_cmd_t;

extern struct k_msgq pic_queue;

#if defined(CONFIG_ZTEST)
#include "idcm_msg.h"

void pic_uart_test_init(void);
void pic_process_cmd(pic_cmd_t cmd);
void pic_handle_msg(const dcm_msg_t* msg);
void pic_door_state_update(door_open_state_t new_state, door_direction_t dir,
                           uint16_t raw_pos);

door_open_state_t pic_get_state(void);
door_direction_t pic_get_direction(void);
uint16_t pic_get_pos(void);
uint16_t pic_get_down_limit(void);
uint16_t pic_get_up_limit(void);
bool pic_is_close_scheduled(void);
struct k_work_delayable* pic_get_close_work(void);

typedef void (*pic_tx_cb_t)(const dcm_msg_t* msg);
void pic_set_test_tx_cb(pic_tx_cb_t cb);
#endif

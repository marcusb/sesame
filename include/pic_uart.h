#pragma once

#include <zephyr/kernel.h>

void pic_uart_init(void);

typedef enum pic_cmd {
    PIC_CMD_UNKNOWN = 0,
    PIC_CMD_OPEN = 1,
    PIC_CMD_CLOSE = 2,
    PIC_CMD_STOP = 4,
    PIC_SERIAL_DATA = 3,
} pic_cmd_t;

extern struct k_msgq pic_queue;

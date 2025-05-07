#pragma once

void pic_uart_task(void *params);

typedef enum pic_cmd {
    PIC_CMD_UNKNOWN = 0,
    PIC_CMD_OPEN = 1,
    PIC_CMD_CLOSE = 2,
    PIC_SERIAL_DATA = 3,
} pic_cmd_t;

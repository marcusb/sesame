#pragma once

void pic_uart_task(void *params);

typedef struct {
    void *ctrl_queue;
    void *pic_queue;
} pic_uart_task_params_t;

typedef enum pic_cmd {
    PIC_CMD_UNKNOWN = 0,
    PIC_CMD_OPEN = 1,
    PIC_CMD_CLOSE = 2,
} pic_cmd_t;

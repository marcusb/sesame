#pragma once

#include "FreeRTOS.h"
#include "queue.h"

void board_init(void);

extern QueueHandle_t ctrl_queue;
extern void* psm_hnd;

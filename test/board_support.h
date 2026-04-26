#pragma once

#include "FreeRTOS.h"
#include "queue.h"

struct mbedtls_ctr_drbg_context;

void board_init(void);

extern QueueHandle_t ctrl_queue;
extern void* psm_hnd;
extern struct mbedtls_ctr_drbg_context ctr_drbg;

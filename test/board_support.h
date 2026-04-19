#pragma once

struct QueueDefinition;
struct mbedtls_ctr_drbg_context;

void test_board_init(void);

extern struct QueueDefinition* ctrl_queue;
extern void* psm_hnd;
extern struct mbedtls_ctr_drbg_context ctr_drbg;

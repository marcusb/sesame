#pragma once

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

/*
 * Common harness API shared by the integration test binaries.
 *
 * Each main_*.c:
 *   1. Calls harness_init().
 *   2. Creates its SUT task(s).
 *   3. Calls harness_run() which starts the kernel and never returns.
 *
 * The harness boots the FreeRTOS+TCP stack (via QEMU eth or WiFi),
 * drains ctrl_queue to stdout, and prints the READY banner once the
 * network is up.
 */

/* Initialise kernel globals (ctrl_queue, logging, inspector). Must be called
   before creating SUT tasks. */
void harness_init(void);

/* Setup the FreeRTOS heap. Must be called first in main(). */
void setup_heap(void);

/* Launch the FreeRTOS scheduler. Does not return. */
void harness_run(void) __attribute__((noreturn));

/* The SUT registers its main task through this callback, which the harness
   calls once the network interface is up (IP assigned via DHCP). */
typedef void (*harness_net_up_cb)(void);
void harness_on_network_up(harness_net_up_cb cb);

/* Command channel: the test driver sends short newline-terminated text
   commands via semihosting stdin. Each received line is passed to
   the handler as a NUL-terminated string, from a dedicated FreeRTOS task. */
typedef void (*harness_cmd_handler_t)(const char* line);
void harness_on_cmd(harness_cmd_handler_t handler);

/* Emit a message to the inspector channel (stdout). */
void host_inspector_emit(const char* line);

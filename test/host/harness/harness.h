#pragma once

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

/*
 * Common harness API shared by the per-module host test binaries.
 *
 * Each main_*.c:
 *   1. Calls harness_init(port) where `port` is the guest TCP/UDP port the
 *      SUT will listen on.  The harness reads HARNESS_HOST_PORT from env
 *      (pytest assigns one) and wires up libslirp port-forwarding via the
 *      --wrap=slirp_new linker trick.
 *   2. Creates its SUT task(s).
 *   3. Calls harness_run() which starts the kernel and never returns.
 *
 * The harness boots the TCP/IP stack, drains ctrl_queue to the inspector UDP
 * socket, and prints the READY banner once the DHCP handshake finishes and
 * the network is up.
 */

/* Register the guest (in-stack) port the SUT listens on.  Call once before
   harness_run(). */
void harness_set_guest_port(int protocol_is_udp, uint16_t guest_port);

/* Initialise kernel globals (ctrl_queue, logging, inspector). Must be called
   before creating SUT tasks. */
void harness_init(void);

/* Launch the FreeRTOS scheduler. Does not return. */
void harness_run(void) __attribute__((noreturn));

/* The SUT registers its main task through this callback, which the harness
   calls once the network interface is up (IP assigned via slirp DHCP). */
typedef void (*harness_net_up_cb)(void);
void harness_on_network_up(harness_net_up_cb cb);

/* Command channel: the test driver sends short newline-terminated text
   commands to a POSIX UDP socket bound by the harness on host loopback
   (port from HARNESS_CMD_PORT env var). Each received line is passed to
   the handler as a NUL-terminated string, from a dedicated FreeRTOS task. */
typedef void (*harness_cmd_handler_t)(const char *line);
void harness_on_cmd(harness_cmd_handler_t handler);

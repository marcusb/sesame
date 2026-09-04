#pragma once

#include <zephyr/kernel.h>

#define L4_UP_EVENT BIT(0)
#define IPV4_UP_EVENT BIT(1)
#define IPV6_UP_EVENT BIT(2)

extern struct k_event network_events;

void network_init(void);
void start_ap(void);
void start_sta(void);
void network_wait_for_up(void);
bool network_is_up(void);
bool network_has_ipv4(void);
bool network_has_ipv6(void);

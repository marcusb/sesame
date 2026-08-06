#pragma once

void network_init(void);
void start_ap(void);
void start_sta(void);
void network_wait_for_up(void);
bool network_is_up(void);

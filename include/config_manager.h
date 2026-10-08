#pragma once

#include <stdint.h>

#if __has_include("app_config.pb.h")
#include "app_config.pb.h"
#define HAVE_APP_CONFIG_PB 1
#endif
int load_config(void);
int save_network_config(void);
int save_mqtt_config(void);
int save_logging_config(void);
int save_limits_config(void);
int save_door_limits(uint16_t up_limit, uint16_t down_limit, uint16_t raw_pos);
int get_door_limits(uint16_t* up_limit, uint16_t* down_limit, uint16_t* raw_pos);

#ifdef HAVE_APP_CONFIG_PB
extern AppConfig app_config;
#endif

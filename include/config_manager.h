#pragma once

#if __has_include("app_config.pb.h")
#include "app_config.pb.h"
#define HAVE_APP_CONFIG_PB 1
#endif
int load_config(void);
int save_network_config(void);
int save_mqtt_config(void);
int save_logging_config(void);

#ifdef HAVE_APP_CONFIG_PB
extern AppConfig app_config;
#endif

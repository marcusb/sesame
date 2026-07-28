#pragma once

#include "app_config.pb.h"

int load_config(void);
int save_network_config(void);
int save_mqtt_config(void);
int save_logging_config(void);

extern AppConfig app_config;

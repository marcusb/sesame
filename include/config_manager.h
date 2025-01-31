#pragma once

#include "app_config.pb.h"

int load_config();
int save_config();

extern AppConfig app_config;

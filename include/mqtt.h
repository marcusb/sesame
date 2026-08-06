#pragma once

#include "controller.h"



void publish_state(const door_state_msg_t *msg);
void mqtt_configure(const MqttConfig *cfg);
void mqtt_stop(void);

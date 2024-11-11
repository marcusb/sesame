#ifndef MQTT_H_
#define MQTT_H_

#include "controller.h"

void mqtt_task(void *params);
void mqtt_subscribe(void);
void publish_state(const door_state_msg_t *msg);

#endif /* #ifndef MQTT_H_ */

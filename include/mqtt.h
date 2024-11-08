#ifndef MQTT_H_
#define MQTT_H_

#include "idcm_msg.h"

void mqtt_task(void *params);

void publish_state(const dcm_door_status_msg_t *msg);

#endif /* #ifndef MQTT_H_ */

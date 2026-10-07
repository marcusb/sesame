#pragma once

#include "controller.h"

#ifdef __cplusplus
extern "C" {
#endif

void matter_update_door_state(const door_state_msg_t* msg);
void matter_update_light_state(bool on);
void InitOTARequestor(void);

#ifdef __cplusplus
}
#endif

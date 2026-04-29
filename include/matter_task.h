#ifndef MATTER_TASK_H
#define MATTER_TASK_H

#include "controller.h"

/**
 * Initialize Matter task and related subsystems.
 */
void matter_init(void);

/**
 * Report door state change to Matter.
 * 
 * @param msg The door state message from the controller.
 */
void matter_report_door_state(const door_state_msg_t* msg);

#endif /* MATTER_TASK_H */

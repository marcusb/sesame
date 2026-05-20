#ifndef MATTER_TASK_H
#define MATTER_TASK_H

#include "controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the CHIP Matter stack (creates the CHIP event-loop task).
 */
void matter_init(void);

/**
 * Notify the Matter task that the network is up (can be called safely from IP task).
 */
void matter_schedule_network_up(void);

/**
 * Report door state change to Matter.
 *
 * @param msg The door state message from the controller.
 */
void matter_report_door_state(const door_state_msg_t* msg);

/**
 * Open the basic commissioning window for `timeout_s` seconds (clamped to a
 * sane range) using the device's root passcode/discriminator. Safe to call
 * even when commissioning is already open — stops the existing window first
 * to ensure SPAKE2+ verifier state is fresh.
 *
 * Returns true on success, false if the Matter stack isn't up yet.
 */
bool matter_commission_open(uint32_t timeout_s);

/**
 * Wipe persisted Matter state: deletes the fabrics file from PSM so that on
 * next boot the device behaves as if never commissioned. Does NOT reboot —
 * the caller should arrange a restart.
 */
void matter_wipe_fabrics(void);

#ifdef __cplusplus
}
#endif

#endif /* MATTER_TASK_H */

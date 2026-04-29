#ifndef MATTER_TASMOTA_SHIM_H
#define MATTER_TASMOTA_SHIM_H

#include "be_vm.h"

/**
 * Drive the Matter/Tasmota state machine tick.
 * Called periodically from the matter task.
 */
void matter_tasmota_tick(bvm* vm);

/**
 * Notify the Tasmota shim that the network is up.
 * Triggers pending network callbacks.
 */
void matter_tasmota_notify_network_up(bvm* vm);

#endif /* MATTER_TASMOTA_SHIM_H */

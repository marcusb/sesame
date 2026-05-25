#pragma once

/* Send IGMPv2 / MLDv2 membership reports for the mDNS multicast groups
 * (224.0.0.251 and ff02::fb) so that IGMP/MLD-snooping APs forward host->device
 * mDNS query traffic. FreeRTOS-Plus-TCP has no native IGMP/MLD support, so the
 * frames are hand-crafted and emitted via the network interface output path. */

void mdns_mcast_join_all(void);

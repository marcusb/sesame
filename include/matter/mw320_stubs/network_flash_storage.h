/* Stub for MW320Config.cpp which uses these functions to persist Wi-Fi
 * credentials.  Sesame replaces the KVS backend via KeyValueStoreManagerImpl
 * (backed by psm_safe.c), so these never execute in our build. */
#pragma once

#include <stdint.h>

static inline uint32_t save_wifi_network(char * filename, uint8_t * network, uint32_t len) { return 1; }
static inline uint32_t get_saved_wifi_network(char * filename, uint8_t * network, uint32_t * len) { return 1; }
static inline uint32_t reset_saved_wifi_network(char * filename) { return 0; }

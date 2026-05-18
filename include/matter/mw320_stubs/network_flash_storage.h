/* KVS backend declarations used by KeyValueStoreManagerImpl (mw320 platform).
 *
 * On hardware: inline stubs that always fail (KVS is replaced by psm_safe.c
 * in a later step). On QEMU: extern declarations backed by kvs_ram.c. */
#pragma once

#include <stdint.h>

#ifdef USE_QEMU
/* Implementations in src/matter/kvs_ram.c */
uint32_t save_wifi_network(char * filename, uint8_t * network, uint32_t len);
uint32_t get_saved_wifi_network(char * filename, uint8_t * network, uint32_t * len);
uint32_t reset_saved_wifi_network(char * filename);
#else
static inline uint32_t save_wifi_network(char * filename, uint8_t * network, uint32_t len) { (void)filename; (void)network; (void)len; return 1; }
static inline uint32_t get_saved_wifi_network(char * filename, uint8_t * network, uint32_t * len) { (void)filename; (void)network; (void)len; return 1; }
static inline uint32_t reset_saved_wifi_network(char * filename) { (void)filename; return 0; }
#endif

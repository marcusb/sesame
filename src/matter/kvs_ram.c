/* RAM-backed KVS backend for QEMU builds.
 *
 * KeyValueStoreManagerImpl (mw320 platform) calls
 * get/save/reset_saved_wifi_network for all KVS operations.  On hardware these
 * hit the NXP flash storage.  On QEMU we keep a fixed-size table in SRAM so
 * Server::Init() can write fabric/session state. Data does not persist across
 * resets, which is acceptable for QEMU testing.
 */

#ifdef USE_QEMU

#include <stdint.h>
#include <string.h>

#define MAX_KV_ENTRIES 64
#define MAX_KEY_LEN 64
#define MAX_VAL_LEN 512

typedef struct {
    char key[MAX_KEY_LEN];
    uint8_t val[MAX_VAL_LEN];
    uint32_t len;
    int used;
} kv_entry_t;

static kv_entry_t s_kvs[MAX_KV_ENTRIES];

static kv_entry_t* find_entry(const char* key) {
    for (int i = 0; i < MAX_KV_ENTRIES; i++) {
        if (s_kvs[i].used && strcmp(s_kvs[i].key, key) == 0) return &s_kvs[i];
    }
    return NULL;
}

uint32_t save_wifi_network(char* filename, uint8_t* network, uint32_t len) {
    if (!filename || len > MAX_VAL_LEN) return 1;
    kv_entry_t* e = find_entry(filename);
    if (!e) {
        for (int i = 0; i < MAX_KV_ENTRIES; i++) {
            if (!s_kvs[i].used) {
                e = &s_kvs[i];
                break;
            }
        }
    }
    if (!e) return 1;
    strncpy(e->key, filename, MAX_KEY_LEN - 1);
    e->key[MAX_KEY_LEN - 1] = '\0';
    memcpy(e->val, network, len);
    e->len = len;
    e->used = 1;
    return 0;
}

uint32_t get_saved_wifi_network(char* filename, uint8_t* network,
                                uint32_t* len) {
    if (!filename || !network || !len) return 1;
    kv_entry_t* e = find_entry(filename);
    if (!e) return 1;
    uint32_t copy = *len < e->len ? *len : e->len;
    memcpy(network, e->val, copy);
    *len = e->len;
    return 0;
}

uint32_t reset_saved_wifi_network(char* filename) {
    if (!filename) return 1;
    kv_entry_t* e = find_entry(filename);
    if (!e) return 0;
    e->used = 0;
    return 0;
}

#endif /* USE_QEMU */

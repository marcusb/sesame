#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "psm.h"

#define PSM_FILE "sesame_psm.bin"

typedef struct {
    char* key;
    void* val;
    uint32_t len;
} psm_entry_t;

static psm_entry_t* entries = NULL;
static uint32_t num_entries = 0;

static void load_psm() {
    FILE* f = fopen(PSM_FILE, "rb");
    if (!f) return;

    while (1) {
        uint32_t key_len;
        if (fread(&key_len, sizeof(key_len), 1, f) != 1) break;
        char* key = malloc(key_len + 1);
        fread(key, 1, key_len, f);
        key[key_len] = 0;

        uint32_t val_len;
        fread(&val_len, sizeof(val_len), 1, f);
        void* val = malloc(val_len);
        fread(val, 1, val_len, f);

        entries = realloc(entries, sizeof(psm_entry_t) * (num_entries + 1));
        entries[num_entries].key = key;
        entries[num_entries].val = val;
        entries[num_entries].len = val_len;
        num_entries++;
    }
    fclose(f);
}

static void save_psm() {
    FILE* f = fopen(PSM_FILE, "wb");
    if (!f) return;

    for (uint32_t i = 0; i < num_entries; i++) {
        uint32_t key_len = strlen(entries[i].key);
        fwrite(&key_len, sizeof(key_len), 1, f);
        fwrite(entries[i].key, 1, key_len, f);
        fwrite(&entries[i].len, sizeof(entries[i].len), 1, f);
        fwrite(entries[i].val, 1, entries[i].len, f);
    }
    fclose(f);
}

int psm_module_init(void* fdesc, psm_hnd_t* phandle, void* psm_cfg) {
    if (entries == NULL) {
        load_psm();
    }
    *phandle = (psm_hnd_t)1;  // Dummy handle
    return 0;
}

int psm_set_variable(psm_hnd_t phandle, const char* variable, const void* value,
                     uint32_t len) {
    for (uint32_t i = 0; i < num_entries; i++) {
        if (strcmp(entries[i].key, variable) == 0) {
            free(entries[i].val);
            entries[i].val = malloc(len);
            memcpy(entries[i].val, value, len);
            entries[i].len = len;
            save_psm();
            return 0;
        }
    }

    entries = realloc(entries, sizeof(psm_entry_t) * (num_entries + 1));
    entries[num_entries].key = strdup(variable);
    entries[num_entries].val = malloc(len);
    memcpy(entries[num_entries].val, value, len);
    entries[num_entries].len = len;
    num_entries++;
    save_psm();
    return 0;
}

int psm_get_variable(psm_hnd_t phandle, const char* variable, void* value,
                     uint32_t max_len) {
    for (uint32_t i = 0; i < num_entries; i++) {
        if (strcmp(entries[i].key, variable) == 0) {
            uint32_t len = entries[i].len;
            if (len > max_len) len = max_len;
            memcpy(value, entries[i].val, len);
            return len;
        }
    }
    return -1;
}

int psm_get_variable_size(psm_hnd_t phandle, const char* variable) {
    for (uint32_t i = 0; i < num_entries; i++) {
        if (strcmp(entries[i].key, variable) == 0) {
            return entries[i].len;
        }
    }
    return -1;
}

int psm_object_delete(psm_hnd_t phandle, const char* variable) {
    for (uint32_t i = 0; i < num_entries; i++) {
        if (strcmp(entries[i].key, variable) == 0) {
            free(entries[i].key);
            free(entries[i].val);
            for (uint32_t j = i; j < num_entries - 1; j++) {
                entries[j] = entries[j + 1];
            }
            num_entries--;
            save_psm();
            return 0;
        }
    }
    return -1;
}

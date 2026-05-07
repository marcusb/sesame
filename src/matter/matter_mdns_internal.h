#pragma once

#include <stddef.h>
#include <stdint.h>

#include "matter_mdns.h"

#define MATTER_MDNS_MAX_SERVICES 8

typedef struct {
    char* service;   // e.g. "_matter"
    char* proto;     // e.g. "_udp"
    char* instance;  // e.g. "ABCDEF1234567890"
    char* hostname;  // e.g. "ABCDEF12345.local"
    uint16_t port;
    uint8_t* txt;  // pre-encoded DNS-SD bytes
    size_t txt_len;
    char** subtypes;  // owned subtype label strings
    size_t subtype_count;
} matter_mdns_service_t;

void matter_mdns_lock(void);
void matter_mdns_unlock(void);

const char* matter_mdns_locked_hostname(void);
const matter_mdns_service_t* matter_mdns_locked_services(size_t* out_count);

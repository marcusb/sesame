#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"
#include "app_logging.h"
#include "be_vm.h"
#include "berry.h"
#include "harness.h"
#include "semphr.h"
#include "task.h"

extern void matter_init(void);
extern bvm* g_matter_vm;
extern SemaphoreHandle_t g_matter_vm_lock;
extern UBaseType_t matter_mdns_snapshot(DNSRecord_t** out);
extern void host_inspector_emit(const char* line);

static void start_matter(void) {
    LogInfo(("Starting Matter stack from harness callback"));
    matter_init();
}

static const char* rec_type_str(uint16_t t) {
    switch (t) {
        case dnsTYPE_A_HOST:
            return "A";
        case dnsTYPE_PTR:
            return "PTR";
        case dnsTYPE_SRV:
            return "SRV";
        case dnsTYPE_TXT:
            return "TXT";
        default:
            return "?";
    }
}

static void emit_mdns_snapshot(void) {
    DNSRecord_t* recs = NULL;
    UBaseType_t n = matter_mdns_snapshot(&recs);
    char buf[512];
    snprintf(buf, sizeof(buf), "MDNS_COUNT=%u", (unsigned)n);
    host_inspector_emit(buf);
    for (UBaseType_t i = 0; i < n; i++) {
        DNSRecord_t* r = &recs[i];
        const char* name = r->pcName ? r->pcName : "";
        const char* type = rec_type_str(r->usRecordType);
        switch (r->usRecordType) {
            case dnsTYPE_PTR:
                snprintf(buf, sizeof(buf), "MDNS %s %s -> %s", type, name,
                         r->xData.pcPtrRecord ? r->xData.pcPtrRecord : "");
                break;
            case dnsTYPE_SRV:
                snprintf(buf, sizeof(buf), "MDNS %s %s -> %s:%u", type, name,
                         r->xData.xSrvRecord.pcTarget
                             ? r->xData.xSrvRecord.pcTarget
                             : "",
                         (unsigned)r->xData.xSrvRecord.usPort);
                break;
            case dnsTYPE_TXT:
                snprintf(buf, sizeof(buf), "MDNS %s %s txt=%s", type, name,
                         r->xData.pcTxtRecord ? r->xData.pcTxtRecord : "");
                break;
            default:
                snprintf(buf, sizeof(buf), "MDNS %s %s", type, name);
                break;
        }
        host_inspector_emit(buf);
    }
    host_inspector_emit("MDNS_END");
}

static void run_berry(const char* code) {
    if (!g_matter_vm || !g_matter_vm_lock) {
        host_inspector_emit("BERRY_ERR vm_not_ready");
        return;
    }
    xSemaphoreTakeRecursive(g_matter_vm_lock, portMAX_DELAY);
    int rc = be_dostring(g_matter_vm, code);
    if (rc != 0) {
        char buf[256];
        const char* err = be_tostring(g_matter_vm, -1);
        snprintf(buf, sizeof(buf), "BERRY_ERR %s", err ? err : "?");
        host_inspector_emit(buf);
        be_pop(g_matter_vm, 1);
    } else {
        host_inspector_emit("BERRY_OK");
    }
    xSemaphoreGiveRecursive(g_matter_vm_lock);
}

static void cmd_handler(const char* line) {
    if (strcmp(line, "mdns_dump") == 0) {
        emit_mdns_snapshot();
    } else if (strncmp(line, "berry ", 6) == 0) {
        run_berry(line + 6);
    } else {
        host_inspector_emit("CMD_UNKNOWN");
    }
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    harness_init();
    harness_on_network_up(start_matter);
    harness_on_cmd(cmd_handler);
    harness_run();

    return 0;
}

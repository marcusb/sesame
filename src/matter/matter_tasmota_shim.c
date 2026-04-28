#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "app_logging.h"
#include "fsl_debug_console.h"
#undef LOG_INFO
#undef LOG_ERROR
#undef LOG_WARN
#undef LOG_DEBUG
#define LOG_INFO(fmt, ...) LogInfo((fmt, ##__VA_ARGS__))
#define LOG_ERROR(fmt, ...) LogError((fmt, ##__VA_ARGS__))
#define LOG_WARN(fmt, ...) LogWarn((fmt, ##__VA_ARGS__))
#define LOG_DEBUG(fmt, ...) LogDebug((fmt, ##__VA_ARGS__))
#include "be_constobj.h"
#include "be_mapping.h"
#include "be_mem.h"
#include "be_module.h"
#include "be_string.h"
#include "be_vm.h"
#include "controller.h"
#include "matter_mdns.h"
#include "psm-v2.h"
#include "queue.h"
#include "task.h"

#define TAG "matter_shim"

extern psm_hnd_t psm_hnd;
extern QueueHandle_t ctrl_queue;

/* tasmota.millis() -> int */
static int tas_millis(bvm* vm) {
    be_pushint(vm, (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    be_return(vm);
}

/* tasmota.rtc_utc() -> int */
static int tas_rtc_utc(bvm* vm) {
    be_pushint(vm, (int)time(NULL));
    be_return(vm);
}

/* tasmota.log(msg:string, level:int) */
static int tas_log(bvm* vm) {
    const char* msg = be_tostring(vm, 1);
    // int level = be_toint(vm, 2);
    LOG_INFO("BERRY: %s", msg);
    be_return_nil(vm);
}

/* tasmota.yield() */
static int tas_yield(bvm* vm) {
    taskYIELD();
    be_return_nil(vm);
}

/* tasmota.time_reached(millis:int) -> bool */
static int tas_time_reached(bvm* vm) {
    int m = be_toint(vm, 1);
    be_pushbool(vm, (int)(xTaskGetTickCount() * portTICK_PERIOD_MS) >= m);
    be_return(vm);
}

/*
 * Registry helpers. berry_matter expects tasmota.{when_network_up, add_*,
 * remove_*, defer, set_timer} to accumulate closures that the matter task
 * later invokes. We store them in Berry-side global lists and call them from
 * matter_tasmota_run_hooks(vm) which the task drives each tick.
 *
 * The tasmota native module is const (its table lives in flash), so we can't
 * attach Berry closures to it directly. Globals are the simplest escape.
 */
static bool network_is_up;

static void list_append_arg(bvm* vm, const char* global_list, int arg_idx) {
    be_getglobal(vm, global_list);
    if (be_isnil(vm, -1)) {
        be_pop(vm, 1);
        be_newobject(vm, "list");
        be_pop(vm, 1); /* discard list_insert helper */
        be_setglobal(vm, global_list);
        be_getglobal(vm, global_list);
    }
    be_getmember(vm, -1, "push");
    be_pushvalue(vm, -2); /* self */
    be_pushvalue(vm, arg_idx);
    be_pcall(vm, 2);
    be_pop(vm, 3);
}

static void list_remove_arg(bvm* vm, const char* global_list, int arg_idx) {
    be_getglobal(vm, global_list); /* [inst] */
    if (be_isnil(vm, -1)) {
        be_pop(vm, 1);
        return;
    }
    int inst_idx = be_absindex(vm, -1);
    be_getmember(vm, inst_idx, ".p"); /* [inst, raw] */
    int raw_idx = be_absindex(vm, -1);
    int n = be_data_size(vm, raw_idx);
    int target = be_absindex(vm, arg_idx);
    for (int i = 0; i < n; i++) {
        be_pushint(vm, i);
        be_getindex(vm, raw_idx); /* [..., raw, i, raw[i]] */
        be_pushvalue(vm, target);
        bbool eq = be_iseq(vm);
        be_pop(vm, 3);
        if (eq) {
            be_getmember(vm, inst_idx, "remove");
            be_pushvalue(vm, inst_idx);
            be_pushint(vm, i);
            be_pcall(vm, 2);
            be_pop(vm, 3);
            break;
        }
    }
    be_pop(vm, 2); /* raw, inst */
}

/* Invoke every closure in list_name with no args. Safe on nil/missing list.
 * The globals we maintain are list instances created via be_newobject("list");
 * their underlying BE_LIST lives in the ".p" member. be_data_size/be_getindex
 * only accept raw BE_LIST, so we peel .p first. */
static void call_each(bvm* vm, const char* list_name) {
    be_getglobal(vm, list_name); /* [inst] */
    if (be_isnil(vm, -1)) {
        be_pop(vm, 1);
        return;
    }
    be_getmember(vm, -1, ".p"); /* [inst, raw_list] */
    int raw_idx = be_absindex(vm, -1);
    int n = be_data_size(vm, raw_idx);
    for (int i = 0; i < n; i++) {
        be_pushint(vm, i);        /* [..., raw, i] */
        be_getindex(vm, raw_idx); /* [..., raw, i, func] */
        int rc = be_pcall(vm, 0); /* [..., raw, i, result|err] */
        if (rc != 0) {
            const char* err = be_tostring(vm, -1);
            LogError(("matter: %s cb #%d: %s", list_name, i, err ? err : "?"));
        }
        be_pop(vm, 2); /* pop result + index */
    }
    be_pop(vm, 2); /* pop raw, inst */
}

static int tas_when_network_up(bvm* vm) {
    list_append_arg(vm, "_matter_net_cbs", 1);
    if (network_is_up) {
        be_pushvalue(vm, 1);
        if (be_pcall(vm, 0) != 0) {
            be_pop(vm, 1);
        }
        be_pop(vm, 1);
    }
    be_return_nil(vm);
}

static int tas_add_fast_loop(bvm* vm) {
    list_append_arg(vm, "_matter_fast_cbs", 1);
    be_return_nil(vm);
}

static int tas_remove_fast_loop(bvm* vm) {
    list_remove_arg(vm, "_matter_fast_cbs", 1);
    be_return_nil(vm);
}

static int tas_add_driver(bvm* vm) {
    list_append_arg(vm, "_matter_drivers", 1);
    be_return_nil(vm);
}

static int tas_remove_driver(bvm* vm) {
    list_remove_arg(vm, "_matter_drivers", 1);
    be_return_nil(vm);
}

static int tas_defer(bvm* vm) {
    list_append_arg(vm, "_matter_fast_cbs_once", 1);
    be_return_nil(vm);
}

static int tas_set_timer(bvm* vm) {
    /* Ignore the delay for now — runs on next fast_loop tick. */
    list_append_arg(vm, "_matter_fast_cbs_once", 2);
    be_return_nil(vm);
}

/* Stubs that berry_matter references but have no sesame analogue. */
static int tas_stub_nil(bvm* vm) {
    (void)vm;
    be_return_nil(vm);
}
static int tas_stub_zero(bvm* vm) {
    be_pushint(vm, 0);
    be_return(vm);
}
/* Matter_Device.init() early-returns unless SetOption151 (Matter enabled) is
 * truthy; the rest of the options berry_matter queries (SO8 Fahrenheit, SO68
 * split RGB/W, SO82 Alexa CT, etc.) are irrelevant to sesame and default 0. */
static int tas_get_option(bvm* vm) {
    int opt = (be_top(vm) >= 1 && be_isint(vm, 1)) ? be_toint(vm, 1) : -1;
    be_pushint(vm, opt == 151 ? 1 : 0);
    be_return(vm);
}
static int tas_stub_false(bvm* vm) {
    be_pushbool(vm, bfalse);
    be_return(vm);
}
static int tas_stub_str(bvm* vm) {
    be_pushstring(vm, "sesame");
    be_return(vm);
}
/* tasmota.version() must return an integer (encoded x.y.z.p) because Matter's
 * EventHandler formats it via >> shifts. Pick 0x00010000 = "0.1.0.0". */
static int tas_version(bvm* vm) {
    be_pushint(vm, 0x00010000);
    be_return(vm);
}

/* Called from matter_task's loop. Ticks fast_loop, drains one-shot defers,
 * fires when_network_up callbacks on transition. */
void matter_tasmota_tick(bvm* vm) {
    call_each(vm, "_matter_fast_cbs");
    be_getglobal(vm, "_matter_fast_cbs_once"); /* [inst] */
    if (be_isnil(vm, -1)) {
        be_pop(vm, 1);
        return;
    }
    int inst_idx = be_absindex(vm, -1);
    be_getmember(vm, inst_idx, ".p"); /* [inst, raw] */
    int raw_idx = be_absindex(vm, -1);
    int n = be_data_size(vm, raw_idx);
    /* Skip the entire drain path when empty. Calling resize(0) on an empty
     * list each tick leaks ~400B/tick inside the Berry VM (bound-method
     * allocations not reclaimed quickly enough), OOMing within ~35s. */
    if (n == 0) {
        be_pop(vm, 2);
        return;
    }
    for (int i = 0; i < n; i++) {
        be_pushint(vm, i);
        be_getindex(vm, raw_idx);
        if (be_pcall(vm, 0) != 0) {
            const char* err = be_tostring(vm, -1);
            LogError(("matter: defer cb #%d: %s", i, err ? err : "?"));
        }
        be_pop(vm, 2);
    }
    be_getmember(vm, inst_idx, "resize"); /* [inst, raw, method] */
    if (!be_isnil(vm, -1)) {
        be_pushvalue(vm, inst_idx);
        be_pushint(vm, 0);
        be_pcall(vm, 2);
    }
    be_pop(vm, 3); /* resize result, raw, inst */
}

void matter_tasmota_notify_network_up(bvm* vm) {
    network_is_up = true;
    call_each(vm, "_matter_net_cbs");
}

/* tasmota.wifi(key?) -> map or value */
static bool get_wifi_info(char* ip_out, char* mac_out, char* ip6_out) {
    NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(NULL);
    while (ep && ep->bits.bIPv6) {
        ep = FreeRTOS_NextEndPoint(NULL, ep);
    }
    if (!ep) return false;
    FreeRTOS_inet_ntoa(ep->ipv4_settings.ulIPAddress, ip_out);
    const uint8_t* mac = ep->xMACAddress.ucBytes;
    snprintf(mac_out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1],
             mac[2], mac[3], mac[4], mac[5]);
    ip6_out[0] = '\0';
    return true;
}

static int tas_wifi(bvm* vm) {
    char ip[16], mac[18], ip6[48];
    bool ok = get_wifi_info(ip, mac, ip6);
    if (be_top(vm) >= 1 && be_isstring(vm, 1)) {
        const char* key = be_tostring(vm, 1);
        if (!strcmp(key, "up")) {
            be_pushbool(vm, ok ? btrue : bfalse);
        } else if (!strcmp(key, "ip")) {
            be_pushstring(vm, ok ? ip : "");
        } else if (!strcmp(key, "mac")) {
            be_pushstring(vm, ok ? mac : "");
        } else {
            be_pushnil(vm);
        }
        be_return(vm);
    }
    be_newobject(vm, "map");
    be_pushstring(vm, "up");
    be_pushbool(vm, ok ? btrue : bfalse);
    be_data_insert(vm, -3);
    be_pop(vm, 2);
    be_pushstring(vm, "ip");
    be_pushstring(vm, ok ? ip : "");
    be_data_insert(vm, -3);
    be_pop(vm, 2);
    be_pushstring(vm, "mac");
    be_pushstring(vm, ok ? mac : "");
    be_data_insert(vm, -3);
    be_pop(vm, 2);
    be_pop(vm, 1);
    be_return(vm);
}

/* tasmota.cmd(name [, silent]) -> map
 *
 * berry_matter's Plugin_Root reads Basic Information / General Diagnostics
 * attributes via tasmota.cmd(...) and indexes the returned JSON-shaped map.
 * Sesame doesn't run Tasmota's command processor, so synthesize the minimal
 * shape Plugin_Root expects for the names it actually queries. Returning nil
 * here causes nil-index exceptions inside read_attribute, which leaves
 * VendorID/ProductID/RegulatoryConfig unanswered during chip-tool's
 * ReadCommissioningInfo and surfaces as "Key not found" on the controller.
 */
extern const char* pcApplicationHostnameHook(void);
static void map_put_str(bvm* vm, const char* key, const char* val) {
    be_pushstring(vm, key);
    be_pushstring(vm, val);
    be_data_insert(vm, -3);
    be_pop(vm, 2);
}
static void map_put_int(bvm* vm, const char* key, int val) {
    be_pushstring(vm, key);
    be_pushint(vm, val);
    be_data_insert(vm, -3);
    be_pop(vm, 2);
}
static int tas_cmd(bvm* vm) {
    const char* name =
        (be_top(vm) >= 1 && be_isstring(vm, 1)) ? be_tostring(vm, 1) : "";
    const char* hostname = pcApplicationHostnameHook();
    if (!hostname) hostname = "sesame";

    be_newobject(vm, "map");

    if (!strcmp(name, "DeviceName")) {
        map_put_str(vm, "DeviceName", hostname);
    } else if (!strcmp(name, "FriendlyName")) {
        map_put_str(vm, "FriendlyName1", hostname);
    } else if (!strncmp(name, "Status 1", 8) && (name[8] == '\0')) {
        be_pushstring(vm, "StatusPRM");
        be_newobject(vm, "map");
        map_put_int(vm, "BootCount", 0);
        be_pop(vm, 1); /* drop inner_data, leaving inner_obj */
        be_data_insert(vm, -3);
        be_pop(vm, 2);
    } else if (!strcmp(name, "Status 2")) {
        be_pushstring(vm, "StatusFWR");
        be_newobject(vm, "map");
        map_put_str(vm, "Hardware", "MW320");
        map_put_str(vm, "Version", "0.1");
        be_pop(vm, 1);
        be_data_insert(vm, -3);
        be_pop(vm, 2);
    } else if (!strcmp(name, "Status 11")) {
        be_pushstring(vm, "StatusSTS");
        be_newobject(vm, "map");
        map_put_int(vm, "UptimeSec",
                    (int)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));
        be_pop(vm, 1);
        be_data_insert(vm, -3);
        be_pop(vm, 2);
    }

    be_pop(vm, 1); /* drop outer_data, leaving outer_obj */
    be_return(vm);
}

/* tasmota.eth() -> map (always down; no ethernet on MW320) */
static int tas_eth(bvm* vm) {
    if (be_top(vm) >= 1 && be_isstring(vm, 1)) {
        const char* key = be_tostring(vm, 1);
        if (!strcmp(key, "up")) {
            be_pushbool(vm, bfalse);
        } else {
            be_pushnil(vm);
        }
        be_return(vm);
    }
    be_newobject(vm, "map");
    be_pushstring(vm, "up");
    be_pushbool(vm, bfalse);
    be_data_insert(vm, -3);
    be_pop(vm, 2);
    be_pop(vm, 1);
    be_return(vm);
}

/* tasmota.loglevel(level) -> bool */
static int tas_loglevel(bvm* vm) {
    int lvl = be_toint(vm, 1);
    be_pushbool(vm, lvl <= 3 ? btrue : bfalse);
    be_return(vm);
}

/* tasmota.rtc(what?) -> int */
static int tas_rtc(bvm* vm) {
    be_pushint(vm, (int)time(NULL));
    be_return(vm);
}

/* tasmota.gc() -> int */
static int tas_gc(bvm* vm) {
    be_gc_collect(vm);
    be_pushint(vm, 0);
    be_return(vm);
}

/* tasmota.delay(ms) */
static int tas_delay(bvm* vm) {
    int ms = be_toint(vm, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    be_return_nil(vm);
}

/* tasmota.scale_uint(v, from_min, from_max, to_min, to_max) -> int */
static int tas_scale_uint(bvm* vm) {
    int v = be_toint(vm, 1);
    int fmn = be_toint(vm, 2);
    int fmx = be_toint(vm, 3);
    int tmn = be_toint(vm, 4);
    int tmx = be_toint(vm, 5);
    int result;
    if (fmx == fmn) {
        result = tmn;
    } else {
        if (v < fmn) v = fmn;
        if (v > fmx) v = fmx;
        result =
            tmn + (int)(((long long)(v - fmn) * (tmx - tmn)) / (fmx - fmn));
    }
    be_pushint(vm, result);
    be_return(vm);
}

/* tasmota.get_config(key:string) -> any */
static int tas_get_config(bvm* vm) {
    const char* key = be_tostring(vm, 1);
    char buf[256];
    int ret = psm_get_variable(psm_hnd, key, buf, sizeof(buf));
    if (ret > 0) {
        be_pushnstring(vm, buf, ret);
    } else {
        be_pushnil(vm);
    }
    be_return(vm);
}

/* tasmota.set_config(key:string, val:string) */
static int tas_set_config(bvm* vm) {
    const char* key = be_tostring(vm, 1);
    const char* val = be_tostring(vm, 2);
    psm_set_variable(psm_hnd, key, val, (int)strlen(val));
    be_return_nil(vm);
}

/* persist module */
static int per_find(bvm* vm) {
    const char* key = be_tostring(vm, 1);
    char psm_key[32];
    snprintf(psm_key, sizeof(psm_key), "p_%s", key);
    char buf[64];
    int ret = psm_get_variable(psm_hnd, psm_key, buf, sizeof(buf));
    if (ret > 0) {
        be_pushnstring(vm, buf, ret);
    } else if (be_top(vm) >= 2) {
        be_pushvalue(vm, 2);
    } else {
        be_pushnil(vm);
    }
    be_return(vm);
}

static int per_setmember(bvm* vm) {
    const char* key = be_tostring(vm, 1);
    const char* val = be_tostring(vm, 2);
    char psm_key[32];
    snprintf(psm_key, sizeof(psm_key), "p_%s", key);
    psm_set_variable(psm_hnd, psm_key, val, (int)strlen(val));
    be_return_nil(vm);
}

static int per_save(bvm* vm) {
    be_return_nil(vm);  // PSM writes immediately for now
}

/* path module */
static int path_remove(bvm* vm) {
    const char* filename = be_tostring(vm, 1);
    psm_object_delete(psm_hnd, filename);
    be_return_nil(vm);
}

static int path_rename(bvm* vm) {
    const char* oldname = be_tostring(vm, 1);
    const char* newname = be_tostring(vm, 2);
    // PSM doesn't have rename, so copy and delete
    char* buf = malloc(4096);
    if (buf) {
        int ret = psm_get_variable(psm_hnd, oldname, buf, 4096);
        if (ret > 0) {
            psm_set_variable(psm_hnd, newname, buf, ret);
            psm_object_delete(psm_hnd, oldname);
        }
        free(buf);
    }
    be_return_nil(vm);
}

/* sesame module */
static int sesame_door_cmd(bvm* vm) {
    int op = be_toint(vm, 1);
    ctrl_msg_t msg;
    msg.type = CTRL_MSG_DOOR_CONTROL;
    switch (op) {
        case 0:
            msg.msg.door_control.command = DOOR_CMD_STOP;
            break;
        case 1:
            msg.msg.door_control.command = DOOR_CMD_OPEN;
            break;
        case 2:
            msg.msg.door_control.command = DOOR_CMD_CLOSE;
            break;
        default:
            be_return_nil(vm);
    }
    xQueueSend(ctrl_queue, &msg, 0);
    be_return_nil(vm);
}

/* mdns module — backed by matter_mdns.c */
static int mdns_start(bvm* vm) { be_return_nil(vm); }

static int mdns_add_hostname(bvm* vm) {
    const char* hostname = be_tostring(vm, 1);
    const char* ipv6 =
        be_top(vm) >= 2 && be_isstring(vm, 2) ? be_tostring(vm, 2) : NULL;
    const char* ipv4 =
        be_top(vm) >= 3 && be_isstring(vm, 3) ? be_tostring(vm, 3) : NULL;
    matter_mdns_add_hostname(hostname, ipv6, ipv4);
    be_return_nil(vm);
}

/*
 * DNS-SD TXT record format: a sequence of <len:byte><chars> chunks, each
 * chunk being "KEY=VALUE". Walks the Berry map at stack idx and emits.
 */
static void serialize_txt_map(bvm* vm, int idx, char* out, size_t outsz) {
    size_t pos = 0;
    out[0] = 0;
    int iter_idx;
    if (be_ismap(vm, idx)) {
        iter_idx = be_absindex(vm, idx);
    } else if (be_isinstance(vm, idx)) {
        /* Map instance: iterate the underlying BE_MAP in .p. */
        be_getmember(vm, idx, ".p");
        iter_idx = be_absindex(vm, -1);
        if (!be_ismap(vm, iter_idx)) {
            LogError(
                ("mdns: .p not a map (type=%s)", be_typename(vm, iter_idx)));
            be_pop(vm, 1);
            return;
        }
    } else {
        LogError(("mdns: serialize_txt_map not map/instance (type=%s)",
                  be_typename(vm, idx)));
        return;
    }
    be_pushiter(vm, iter_idx);
    while (be_iter_hasnext(vm, iter_idx)) {
        be_iter_next(vm, iter_idx);
        /* stack: [..., iter, key, value] */
        const char* key = be_tostring(vm, -2);
        char valbuf[32];
        const char* val;
        if (be_isstring(vm, -1)) {
            val = be_tostring(vm, -1);
        } else if (be_isint(vm, -1)) {
            snprintf(valbuf, sizeof(valbuf), "%ld", (long)be_toint(vm, -1));
            val = valbuf;
        } else {
            val = be_tostring(vm, -1);
        }
        size_t klen = strlen(key);
        size_t vlen = strlen(val);
        size_t entry = klen + 1 + vlen;
        if (entry > 255 || pos + 1 + entry >= outsz) {
            be_pop(vm, 2);
            continue;
        }
        out[pos++] = (char)entry;
        memcpy(out + pos, key, klen);
        pos += klen;
        out[pos++] = '=';
        memcpy(out + pos, val, vlen);
        pos += vlen;
        be_pop(vm, 2);
    }
    be_pop(vm, 1);
    if (be_isinstance(vm, idx)) {
        be_pop(vm, 1); /* pop the .p we pushed */
    }
    out[pos] = 0;
}

static int mdns_add_service(bvm* vm) {
    const char* service = be_tostring(vm, 1);
    const char* proto = be_tostring(vm, 2);
    int port = be_toint(vm, 3);
    char txt[512];
    serialize_txt_map(vm, 4, txt, sizeof(txt));
    const char* instance = be_tostring(vm, 5);
    const char* hostname = be_tostring(vm, 6);
    matter_mdns_add_service(service, proto, port, txt, instance, hostname);
    be_return_nil(vm);
}

static int mdns_add_subtype(bvm* vm) {
    const char* service = be_tostring(vm, 1);
    const char* proto = be_tostring(vm, 2);
    const char* instance = be_tostring(vm, 3);
    const char* hostname = be_tostring(vm, 4);
    const char* subtype = be_tostring(vm, 5);
    matter_mdns_add_subtype(service, proto, instance, hostname, subtype);
    be_return_nil(vm);
}

static int mdns_remove_service(bvm* vm) {
    const char* service = be_tostring(vm, 1);
    const char* proto = be_tostring(vm, 2);
    const char* instance = be_tostring(vm, 3);
    const char* hostname = be_tostring(vm, 4);
    matter_mdns_remove_service(service, proto, instance, hostname);
    be_return_nil(vm);
}

/* UDP class shim. FreeRTOS+TCP rejects binding two UDP sockets to the same
 * port (-EADDRINUSE), so we use a single AF_INET socket. The stack still
 * delivers IPv6 datagrams to it; recvfrom's `from.sin_family` carries the true
 * family and sendto's destination `sin_family` controls how the packet is
 * built — the socket family doesn't constrain either direction. */
typedef struct {
    Socket_t socket;
    char remote_ip[46]; /* INET6_ADDRSTRLEN */
    int remote_port;
} tas_udp_t;

static tas_udp_t* udp_self(bvm* vm) {
    be_getmember(vm, 1, "_p");
    tas_udp_t* u = (tas_udp_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    return u;
}

static int udp_init(bvm* vm) {
    tas_udp_t* u = (tas_udp_t*)be_malloc(vm, sizeof(tas_udp_t));
    u->socket = FREERTOS_INVALID_SOCKET;
    u->remote_ip[0] = '\0';
    u->remote_port = 0;
    be_pushcomptr(vm, u);
    be_setmember(vm, 1, "_p");
    be_pop(vm, 1);
    be_return_nil(vm);
}

static int udp_deinit(bvm* vm) {
    tas_udp_t* u = udp_self(vm);
    if (u) {
        if (u->socket != FREERTOS_INVALID_SOCKET)
            FreeRTOS_closesocket(u->socket);
        be_free(vm, u, sizeof(tas_udp_t));
        be_pushcomptr(vm, NULL);
        be_setmember(vm, 1, "_p");
        be_pop(vm, 1);
    }
    be_return_nil(vm);
}

static int udp_begin(bvm* vm) {
    tas_udp_t* u = udp_self(vm);
    if (!u) {
        be_pushbool(vm, bfalse);
        be_return(vm);
    }
    int port = be_toint(vm, 3);
    Socket_t s = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
                                 FREERTOS_IPPROTO_UDP);
    if (s == FREERTOS_INVALID_SOCKET) {
        be_pushbool(vm, bfalse);
        be_return(vm);
    }
    TickType_t zero = 0;
    FreeRTOS_setsockopt(s, 0, FREERTOS_SO_RCVTIMEO, &zero, sizeof(zero));
    struct freertos_sockaddr addr = {0};
    addr.sin_family = FREERTOS_AF_INET;
    addr.sin_port = FreeRTOS_htons(port);
    if (FreeRTOS_bind(s, &addr, sizeof(addr)) != 0) {
        FreeRTOS_closesocket(s);
        be_pushbool(vm, bfalse);
        be_return(vm);
    }
    u->socket = s;
    be_pushbool(vm, btrue);
    be_return(vm);
}

static int udp_close(bvm* vm) {
    tas_udp_t* u = udp_self(vm);
    if (u && u->socket != FREERTOS_INVALID_SOCKET) {
        FreeRTOS_closesocket(u->socket);
        u->socket = FREERTOS_INVALID_SOCKET;
    }
    be_return_nil(vm);
}

static int udp_read(bvm* vm) {
    tas_udp_t* u = udp_self(vm);
    if (!u || u->socket == FREERTOS_INVALID_SOCKET) be_return_nil(vm);
    struct freertos_sockaddr from;
    uint32_t from_len = sizeof(from);
    uint8_t buf[1024];
    int32_t n =
        FreeRTOS_recvfrom(u->socket, buf, sizeof(buf), 0, &from, &from_len);
    if (n <= 0) be_return_nil(vm);
    if (from.sin_family == FREERTOS_AF_INET6) {
        FreeRTOS_inet_ntop(FREERTOS_AF_INET6, from.sin_address.xIP_IPv6.ucBytes,
                           u->remote_ip, sizeof(u->remote_ip));
    } else {
        FreeRTOS_inet_ntop(FREERTOS_AF_INET, &from.sin_address.ulIP_IPv4,
                           u->remote_ip, sizeof(u->remote_ip));
    }
    u->remote_port = FreeRTOS_ntohs(from.sin_port);
    be_pushbytes(vm, buf, n);
    be_return(vm);
}

static int udp_send(bvm* vm) {
    tas_udp_t* u = udp_self(vm);
    if (!u || u->socket == FREERTOS_INVALID_SOCKET) {
        be_pushbool(vm, bfalse);
        be_return(vm);
    }
    const char* ip_str = be_tostring(vm, 2);
    int port = be_toint(vm, 3);
    size_t len;
    const void* data = be_tobytes(vm, 4, &len);
    bool is_v6 = strchr(ip_str, ':') != NULL;
    struct freertos_sockaddr to = {0};
    to.sin_port = FreeRTOS_htons(port);
    if (is_v6) {
        to.sin_family = FREERTOS_AF_INET6;
        if (FreeRTOS_inet_pton(FREERTOS_AF_INET6, ip_str,
                               to.sin_address.xIP_IPv6.ucBytes) != pdPASS) {
            be_pushbool(vm, bfalse);
            be_return(vm);
        }
    } else {
        to.sin_family = FREERTOS_AF_INET;
        to.sin_address.ulIP_IPv4 = FreeRTOS_inet_addr(ip_str);
    }
    int32_t n = FreeRTOS_sendto(u->socket, data, len, 0, &to, sizeof(to));
    be_pushbool(vm, n == (int32_t)len);
    be_return(vm);
}

static int udp_member(bvm* vm) {
    tas_udp_t* u = udp_self(vm);
    if (!u) {
        be_return_nil(vm);
    }
    const char* attr = be_tostring(vm, 2);
    if (!strcmp(attr, "remote_ip")) {
        be_pushstring(vm, u->remote_ip);
        be_return(vm);
    } else if (!strcmp(attr, "remote_port")) {
        be_pushint(vm, u->remote_port);
        be_return(vm);
    }
    be_return_nil(vm);
}

/* File System Shims */
typedef struct {
    char name[32];
    char* data;
    size_t size;
    size_t pos;
    bool write;
} virtual_file_t;

void* be_fopen(const char* filename, const char* modes) {
    virtual_file_t* f = calloc(1, sizeof(virtual_file_t));
    strncpy(f->name, filename, sizeof(f->name) - 1);
    f->write = (strchr(modes, 'w') != NULL);

    char* buf = malloc(4096);
    int ret = psm_get_variable(psm_hnd, filename, buf, 4096);
    if (ret > 0) {
        f->data = buf;
        f->size = (size_t)ret;
    } else {
        f->data = buf;
        f->size = 0;
    }
    return f;
}

int be_fclose(void* hfile) {
    virtual_file_t* f = (virtual_file_t*)hfile;
    if (f->write) {
        psm_set_variable(psm_hnd, f->name, f->data, (int)f->size);
    }
    free(f->data);
    free(f);
    return 0;
}

size_t be_fwrite(void* hfile, const void* buffer, size_t length) {
    virtual_file_t* f = (virtual_file_t*)hfile;
    f->data = realloc(f->data, f->pos + length);
    memcpy(f->data + f->pos, buffer, length);
    f->pos += length;
    if (f->pos > f->size) f->size = f->pos;
    return length;
}

size_t be_fread(void* hfile, void* buffer, size_t length) {
    virtual_file_t* f = (virtual_file_t*)hfile;
    if (f->pos + length > f->size) length = f->size - f->pos;
    memcpy(buffer, f->data + f->pos, length);
    f->pos += length;
    return length;
}

char* be_fgets(void* hfile, void* buffer, int size) {
    virtual_file_t* f = (virtual_file_t*)hfile;
    int i = 0;
    char* b = (char*)buffer;
    while (i < size - 1 && f->pos < f->size) {
        b[i] = f->data[f->pos++];
        if (b[i++] == '\n') break;
    }
    b[i] = '\0';
    return i > 0 ? (char*)buffer : NULL;
}

int be_fseek(void* hfile, long offset) {
    virtual_file_t* f = (virtual_file_t*)hfile;
    f->pos = (size_t)offset;
    return 0;
}

long int be_ftell(void* hfile) {
    virtual_file_t* f = (virtual_file_t*)hfile;
    return (long int)f->pos;
}

size_t be_fsize(void* hfile) {
    virtual_file_t* f = (virtual_file_t*)hfile;
    return f->size;
}

int be_fflush(void* hfile) { return 0; }

void* berry_malloc32(size_t size) { return malloc(size); }

int matter_publish_command(bvm* vm) {
    LOG_WARN("matter_publish_command: not implemented");
    be_return_nil(vm);
}

/* Missing function from be_matter_module.c */
uint32_t matter_convert_seconds_to_dhm(uint32_t seconds, char* unit,
                                       uint32_t* color, bbool days) {
    *color = 0xFFFFFF;  // Default white
    if (seconds < 60) {
        *unit = 's';
        return seconds;
    }
    if (seconds < 3600) {
        *unit = 'm';
        return seconds / 60;
    }
    if (seconds < 86400 || !days) {
        *unit = 'h';
        return seconds / 3600;
    }
    *unit = 'd';
    return seconds / 86400;
}

/* @const_object_info_begin
class be_class_udp (scope: global, name: udp) {
    _p, var
    init, func(udp_init)
    deinit, func(udp_deinit)
    begin, func(udp_begin)
    close, func(udp_close)
    read, func(udp_read)
    send, func(udp_send)
    member, func(udp_member)
}

module tasmota (scope: global, name: tasmota) {
    millis, func(tas_millis)
    rtc_utc, func(tas_rtc_utc)
    rtc, func(tas_rtc)
    log, func(tas_log)
    yield, func(tas_yield)
    time_reached, func(tas_time_reached)
    get_config, func(tas_get_config)
    set_config, func(tas_set_config)
    wifi, func(tas_wifi)
    eth, func(tas_eth)
    loglevel, func(tas_loglevel)
    gc, func(tas_gc)
    delay, func(tas_delay)
    scale_uint, func(tas_scale_uint)
    when_network_up, func(tas_when_network_up)
    add_fast_loop, func(tas_add_fast_loop)
    remove_fast_loop, func(tas_remove_fast_loop)
    add_driver, func(tas_add_driver)
    remove_driver, func(tas_remove_driver)
    defer, func(tas_defer)
    set_timer, func(tas_set_timer)
    cmd, func(tas_cmd)
    resp_cmnd_str, func(tas_stub_nil)
    resp_cmnd_done, func(tas_stub_nil)
    publish_result, func(tas_stub_nil)
    get_option, func(tas_get_option)
    get_power, func(tas_stub_nil)
    set_power, func(tas_stub_false)
    locale, func(tas_stub_str)
    version, func(tas_version)
    github, func(tas_stub_str)
    add_cmd, func(tas_stub_nil)
    remove_cmd, func(tas_stub_nil)
}

module persist (scope: global, name: persist) {
    find, func(per_find)
    setmember, func(per_setmember)
    save, func(per_save)
}

module path (scope: global, name: path) {
    remove, func(path_remove)
    rename, func(path_rename)
}

module sesame (scope: global, name: sesame) {
    door_cmd, func(sesame_door_cmd)
}

module mdns (scope: global, name: mdns) {
    start, func(mdns_start)
    add_hostname, func(mdns_add_hostname)
    add_service, func(mdns_add_service)
    add_subtype, func(mdns_add_subtype)
    remove_service, func(mdns_remove_service)
}
@const_object_info_end */

#include "be_fixed_be_class_udp.h"
#include "be_fixed_mdns.h"
#include "be_fixed_path.h"
#include "be_fixed_persist.h"
#include "be_fixed_sesame.h"
#include "be_fixed_tasmota.h"

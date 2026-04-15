#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "app_logging.h"
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

/* mdns module stub */
static int mdns_start(bvm* vm) { be_return_nil(vm); }
static int mdns_add_hostname(bvm* vm) { be_return_nil(vm); }
static int mdns_add_service(bvm* vm) { be_return_nil(vm); }
static int mdns_add_subtype(bvm* vm) { be_return_nil(vm); }
static int mdns_remove_service(bvm* vm) { be_return_nil(vm); }

/* UDP class shim */
typedef struct {
    Socket_t socket;
    char remote_ip[16];
    int remote_port;
} tas_udp_t;

static int udp_init(bvm* vm) {
    tas_udp_t* u = (tas_udp_t*)be_malloc(vm, sizeof(tas_udp_t));
    u->socket = FREERTOS_INVALID_SOCKET;
    u->remote_ip[0] = '\0';
    u->remote_port = 0;
    be_pushcomptr(vm, u);
    be_return(vm);
}

static int udp_deinit(bvm* vm) {
    tas_udp_t* u = (tas_udp_t*)be_tocomptr(vm, 1);
    if (u->socket != FREERTOS_INVALID_SOCKET) {
        FreeRTOS_closesocket(u->socket);
    }
    be_free(vm, u, sizeof(tas_udp_t));
    be_return_nil(vm);
}

static int udp_begin(bvm* vm) {
    tas_udp_t* u = (tas_udp_t*)be_tocomptr(vm, 1);
    int port = be_toint(vm, 2);
    u->socket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
                                FREERTOS_IPPROTO_UDP);
    if (u->socket != FREERTOS_INVALID_SOCKET) {
        struct freertos_sockaddr addr;
        addr.sin_port = FreeRTOS_htons(port);
        addr.sin_address.ulIP_IPv4 = 0;
        FreeRTOS_bind(u->socket, &addr, sizeof(addr));
    }
    be_return_nil(vm);
}

static int udp_read(bvm* vm) {
    tas_udp_t* u = (tas_udp_t*)be_tocomptr(vm, 1);
    if (u->socket == FREERTOS_INVALID_SOCKET) {
        be_return_nil(vm);
    }
    struct freertos_sockaddr from;
    uint32_t from_len = sizeof(from);
    uint8_t buf[1024];
    int32_t n =
        FreeRTOS_recvfrom(u->socket, buf, sizeof(buf), 0, &from, &from_len);
    if (n > 0) {
        FreeRTOS_inet_ntoa(from.sin_address.ulIP_IPv4, u->remote_ip);
        u->remote_port = FreeRTOS_ntohs(from.sin_port);
        void* res = be_pushbuffer(vm, n);
        memcpy(res, buf, n);
        be_return(vm);
    }
    be_return_nil(vm);
}

static int udp_send(bvm* vm) {
    tas_udp_t* u = (tas_udp_t*)be_tocomptr(vm, 1);
    const char* ip_str = be_tostring(vm, 2);
    int port = be_toint(vm, 3);
    size_t len;
    const void* data = be_tobytes(vm, 4, &len);
    if (u->socket == FREERTOS_INVALID_SOCKET) {
        be_pushbool(vm, bfalse);
        be_return(vm);
    }
    struct freertos_sockaddr to;
    to.sin_port = FreeRTOS_htons(port);
    to.sin_address.ulIP_IPv4 = FreeRTOS_inet_addr(ip_str);
    int32_t n = FreeRTOS_sendto(u->socket, data, len, 0, &to, sizeof(to));
    be_pushbool(vm, n == (int32_t)len);
    be_return(vm);
}

static int udp_member(bvm* vm) {
    tas_udp_t* u = (tas_udp_t*)be_tocomptr(vm, 1);
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
    read, func(udp_read)
    send, func(udp_send)
    member, func(udp_member)
}

module tasmota (scope: global, name: tasmota) {
    millis, func(tas_millis)
    rtc_utc, func(tas_rtc_utc)
    log, func(tas_log)
    yield, func(tas_yield)
    time_reached, func(tas_time_reached)
    get_config, func(tas_get_config)
    set_config, func(tas_set_config)
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

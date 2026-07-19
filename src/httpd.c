#include "httpd.h"

#include <zephyr/logging/log.h>

#include "app_config.pb.h"
#include "controller.h"
#include "pb_decode.h"
#if SESAME_ENABLE_MATTER
#include "matter_task.h"
#endif

LOG_MODULE_REGISTER(httpd, LOG_LEVEL_DBG);

#include <ctype.h>
#include <string.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>

int strcasecmp(const char* s1, const char* s2) {
    while (*s1 && *s2) {
        int diff = tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
        if (diff != 0) return diff;
        s1++;
        s2++;
    }
    return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

char* strpbrk(const char* s, const char* accept) {
    while (*s) {
        const char* a = accept;
        while (*a) {
            if (*a++ == *s) return (char*)s;
        }
        s++;
    }
    return NULL;
}

static uint16_t http_port = 80;
HTTP_SERVICE_DEFINE(httpd_service, NULL, &http_port, 3, 10, NULL, NULL, NULL);

static int handle_cfg_request(const struct http_request_ctx* req,
                              ctrl_msg_type_t type, const pb_msgdesc_t* desc) {
    ctrl_msg_t msg = {type};
    pb_istream_t stream =
        pb_istream_from_buffer((const pb_byte_t*)req->data, req->data_len);
    bool status = pb_decode(&stream, desc, &msg.msg);
    if (!status) {
        return 400;  // Bad request
    }
    k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
    return 200;  // OK
}

static int cfg_handler(struct http_client_ctx* client,
                       enum http_transaction_status status,
                       const struct http_request_ctx* req,
                       struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        ctrl_msg_type_t type = (ctrl_msg_type_t)(uintptr_t)user_data;
        const pb_msgdesc_t* desc = NULL;
        if (type == CTRL_MSG_WIFI_CONFIG) {
            desc = &NetworkConfig_msg;
        } else if (type == CTRL_MSG_MQTT_CONFIG) {
            desc = &MqttConfig_msg;
        } else if (type == CTRL_MSG_LOGGING_CONFIG) {
            desc = &LoggingConfig_msg;
        }

        if (desc) {
            res->status = handle_cfg_request(req, type, desc);
        } else {
            res->status = 500;
        }
    }
    return 0;
}

static int restart_handler(struct http_client_ctx* client,
                           enum http_transaction_status status,
                           const struct http_request_ctx* request_ctx,
                           struct http_response_ctx* response_ctx,
                           void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        ctrl_msg_t msg = {.type = CTRL_MSG_RESTART};
        k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        response_ctx->status = 200;
    }
    return 0;
}

static int fwupgrade_handler(struct http_client_ctx* client,
                             enum http_transaction_status status,
                             const struct http_request_ctx* req,
                             struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        res->status = handle_cfg_request(req, CTRL_MSG_OTA_UPGRADE,
                                         &FirmwareUpgradeFetchRequest_msg);
    }
    return 0;
}

static int promote_handler(struct http_client_ctx* client,
                           enum http_transaction_status status,
                           const struct http_request_ctx* req,
                           struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        ctrl_msg_t msg = {.type = CTRL_MSG_OTA_PROMOTE};
        k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        res->status = 200;
    }
    return 0;
}

static int open_handler(struct http_client_ctx* client,
                        enum http_transaction_status status,
                        const struct http_request_ctx* req,
                        struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        ctrl_msg_t msg = {.type = CTRL_MSG_DOOR_CONTROL,
                          .msg.door_control = {DOOR_CMD_OPEN}};
        k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        res->status = 200;
    }
    return 0;
}

static int close_handler(struct http_client_ctx* client,
                         enum http_transaction_status status,
                         const struct http_request_ctx* req,
                         struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        ctrl_msg_t msg = {.type = CTRL_MSG_DOOR_CONTROL,
                          .msg.door_control = {DOOR_CMD_CLOSE}};
        k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        res->status = 200;
    }
    return 0;
}

#if SESAME_ENABLE_MATTER
static int matter_commission_handler(struct http_client_ctx* client,
                                     enum http_transaction_status status,
                                     const struct http_request_ctx* req,
                                     struct http_response_ctx* res,
                                     void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        const bool ok = matter_commission_open(900);
        res->status = ok ? 200 : 500;
    }
    return 0;
}

static int matter_reset_handler(struct http_client_ctx* client,
                                enum http_transaction_status status,
                                const struct http_request_ctx* req,
                                struct http_response_ctx* res,
                                void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        matter_wipe_fabrics();
        ctrl_msg_t msg = {.type = CTRL_MSG_RESTART};
        k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        res->status = 200;
    }
    return 0;
}
#endif

/* --- HTTP Resource Details --- */

static struct http_resource_detail_dynamic cfg_network_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = cfg_handler,
    .user_data = (void*)(uintptr_t)CTRL_MSG_WIFI_CONFIG,
};

static struct http_resource_detail_dynamic cfg_mqtt_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = cfg_handler,
    .user_data = (void*)(uintptr_t)CTRL_MSG_MQTT_CONFIG,
};

static struct http_resource_detail_dynamic cfg_logging_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = cfg_handler,
    .user_data = (void*)(uintptr_t)CTRL_MSG_LOGGING_CONFIG,
};

static struct http_resource_detail_dynamic restart_resource_detail = {
    .common =
        {
            .type = HTTP_RESOURCE_TYPE_DYNAMIC,
            .bitmask_of_supported_http_methods = BIT(HTTP_POST) | BIT(HTTP_GET),
        },
    .cb = restart_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic fwupgrade_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = fwupgrade_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic promote_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = promote_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic open_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = open_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic close_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = close_handler,
    .user_data = NULL,
};

#if SESAME_ENABLE_MATTER
static struct http_resource_detail_dynamic matter_commission_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = matter_commission_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic matter_reset_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = matter_reset_handler,
    .user_data = NULL,
};
#endif

/* --- HTTP Resources Definitions --- */

HTTP_RESOURCE_DEFINE(cfg_network_res, httpd_service, "/cfg/network",
                     &cfg_network_detail);
HTTP_RESOURCE_DEFINE(cfg_mqtt_res, httpd_service, "/cfg/mqtt",
                     &cfg_mqtt_detail);
HTTP_RESOURCE_DEFINE(cfg_logging_res, httpd_service, "/cfg/logging",
                     &cfg_logging_detail);
HTTP_RESOURCE_DEFINE(restart_resource, httpd_service, "/restart",
                     &restart_resource_detail);
HTTP_RESOURCE_DEFINE(fwupgrade_resource, httpd_service, "/fwupgrade",
                     &fwupgrade_detail);
HTTP_RESOURCE_DEFINE(promote_resource, httpd_service, "/promote",
                     &promote_detail);
HTTP_RESOURCE_DEFINE(open_resource, httpd_service, "/open", &open_detail);
HTTP_RESOURCE_DEFINE(close_resource, httpd_service, "/close", &close_detail);

#if SESAME_ENABLE_MATTER
HTTP_RESOURCE_DEFINE(matter_commission_resource, httpd_service,
                     "/matter/commission", &matter_commission_detail);
HTTP_RESOURCE_DEFINE(matter_reset_resource, httpd_service, "/matter/reset",
                     &matter_reset_detail);
#endif

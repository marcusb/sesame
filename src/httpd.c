#include "httpd.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/net/http/status.h>
#include <zephyr/sys/clock.h>

#include "app_config.pb.h"
#include "controller.h"
#include "pb_decode.h"
#ifdef CONFIG_CHIP
#include "matter_task.h"
#endif
#include <zephyr/app_version.h>
#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include "ota.h"
#endif
#include "log_ring_buf.h"
#include "mcuboot_info.h"
#include "mcuboot_updater.h"
#include "pic_uart.h"
#include "pic_updater.h"
#include "ws_logs.h"

LOG_MODULE_REGISTER(httpd, LOG_LEVEL_DBG);

#ifdef CONFIG_BOARD_NATIVE_SIM
static uint16_t http_port = 8080;
#else
static uint16_t http_port = 80;
#endif
HTTP_SERVICE_DEFINE(httpd_service, NULL, &http_port, 3, 10, NULL, NULL, NULL);

static int handle_cfg_request(const struct http_request_ctx* req,
                              ctrl_msg_type_t type, const pb_msgdesc_t* desc) {
    LOG_INF("Handling cfg request type %d", type);
    ctrl_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = type;
    pb_istream_t stream =
        pb_istream_from_buffer((const pb_byte_t*)req->data, req->data_len);
    bool status = pb_decode(&stream, desc, &msg.msg);
    if (!status) {
        LOG_ERR("pb_decode failed: %s", PB_GET_ERROR(&stream));
        return HTTP_400_BAD_REQUEST;
    }
    LOG_INF("pb_decode succeeded, enqueuing msg");
    int ret = k_msgq_put(&ctrl_queue, &msg, K_MSEC(1000));
    if (ret != 0) {
        LOG_ERR("Failed to enqueue ctrl message: %d", ret);
        return HTTP_500_INTERNAL_SERVER_ERROR;
    }
    return HTTP_200_OK;
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
            res->status = HTTP_500_INTERNAL_SERVER_ERROR;
        }
        res->body_len = 0;
        res->body = NULL;
        res->final_chunk = true;
    }
    return 0;
}

static int restart_handler(struct http_client_ctx* client,
                           enum http_transaction_status status,
                           const struct http_request_ctx* request_ctx,
                           struct http_response_ctx* response_ctx,
                           void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        LOG_INF("restart_handler called!");
        ctrl_msg_t msg = {.type = CTRL_MSG_RESTART};
        int ret = k_msgq_put(&ctrl_queue, &msg, K_MSEC(1000));
        if (ret != 0) {
            LOG_ERR("Failed to enqueue restart msg: %d", ret);
            response_ctx->status = HTTP_500_INTERNAL_SERVER_ERROR;
        } else {
            response_ctx->status = HTTP_200_OK;
        }
        response_ctx->body_len = 0;
        response_ctx->body = NULL;
        response_ctx->final_chunk = true;
    }
    return 0;
}

static int time_handler(struct http_client_ctx* client,
                        enum http_transaction_status status,
                        const struct http_request_ctx* request_ctx,
                        struct http_response_ctx* response_ctx,
                        void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        static char time_buf[64];
        struct timespec ts;
        sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);
        struct tm tm;
        time_t t = ts.tv_sec;
        gmtime_r(&t, &tm);
        char iso_str[32];
        strftime(iso_str, sizeof(iso_str), "%Y-%m-%dT%H:%M:%SZ", &tm);
        int len = snprintf(time_buf, sizeof(time_buf),
                           "{\"epoch\":%ld,\"time\":\"%s\"}\n", (long)ts.tv_sec,
                           iso_str);
        response_ctx->status = HTTP_200_OK;
        response_ctx->body = (const uint8_t*)time_buf;
        response_ctx->body_len = len;
        response_ctx->final_chunk = true;
    }
    return 0;
}

static int version_handler(struct http_client_ctx* client,
                           enum http_transaction_status status,
                           const struct http_request_ctx* req,
                           struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        static char buf[384];
        const struct mcuboot_info* mb = mcuboot_get_info();
        char mcuboot_buf[128];
        if (mb->magic == MCUBOOT_INFO_MAGIC) {
            snprintf(mcuboot_buf, sizeof(mcuboot_buf),
                     "\"mcuboot\":{\"version\":\"%s\",\"slot\":%u,\"gen_"
                     "level\":%u,\"confirmed\":%s}",
                     mb->version_str, mb->slot, mb->gen_level,
                     mb->confirmed ? "true" : "false");
        } else {
            snprintf(mcuboot_buf, sizeof(mcuboot_buf),
                     "\"mcuboot\":{\"version\":\"unknown\",\"slot\":0,\"gen_"
                     "level\":0,\"confirmed\":false}");
        }

#ifdef CONFIG_BOOTLOADER_MCUBOOT
        int slot = my_boot_fetch_active_slot();
        bool confirmed =
            (slot == 0 || slot == 1) ? my_boot_is_img_confirmed() : false;
        int len;
        if (slot == 0 || slot == 1) {
            len = snprintf(
                buf, sizeof(buf),
                "{\"version\":\"%s\",\"slot\":%d,\"confirmed\":%s,%s}\n",
                SESAME_VERSION_STR, slot, confirmed ? "true" : "false",
                mcuboot_buf);
        } else {
            len = snprintf(buf, sizeof(buf),
                           "{\"version\":\"%s\",\"slot\":\"none\","
                           "\"confirmed\":false,%s}\n",
                           SESAME_VERSION_STR, mcuboot_buf);
        }
#else
        int len = snprintf(
            buf, sizeof(buf),
            "{\"version\":\"%s\",\"slot\":\"none\",\"confirmed\":false,%s}\n",
            SESAME_VERSION_STR, mcuboot_buf);
#endif
        static const struct http_header headers[] = {
            {.name = "Content-Type", .value = "application/json"}};
        res->status = HTTP_200_OK;
        res->headers = headers;
        res->header_count = 1;
        res->body = (const uint8_t*)buf;
        res->body_len = len;
        res->final_chunk = true;
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
        res->body_len = 0;
        res->body = NULL;
        res->final_chunk = true;
    }
    return 0;
}

static int promote_handler(struct http_client_ctx* client,
                           enum http_transaction_status status,
                           const struct http_request_ctx* req,
                           struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        LOG_INF("promote_handler called!");
        ctrl_msg_t msg = {.type = CTRL_MSG_OTA_PROMOTE};
        int ret = k_msgq_put(&ctrl_queue, &msg, K_MSEC(1000));
        if (ret != 0) {
            LOG_ERR("Failed to enqueue promote msg: %d", ret);
            res->status = HTTP_500_INTERNAL_SERVER_ERROR;
        } else {
            res->status = HTTP_200_OK;
        }
        res->body_len = 0;
        res->body = NULL;
        res->final_chunk = true;
    }
    return 0;
}

static const char s_pic_updating_err[] =
    "{\"error\":\"PIC update in progress\"}\n";

static bool check_pic_updating(struct http_response_ctx* res) {
    if (pic_is_updating()) {
        static const struct http_header headers[] = {
            {.name = "Content-Type", .value = "application/json"}};
        res->status = HTTP_503_SERVICE_UNAVAILABLE;
        res->headers = headers;
        res->header_count = 1;
        res->body = (const uint8_t*)s_pic_updating_err;
        res->body_len = sizeof(s_pic_updating_err) - 1;
        res->final_chunk = true;
        return true;
    }
    return false;
}

static int open_handler(struct http_client_ctx* client,
                        enum http_transaction_status status,
                        const struct http_request_ctx* req,
                        struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        LOG_INF("open_handler called!");
        if (check_pic_updating(res)) {
            return 0;
        }
        ctrl_msg_t msg = {.type = CTRL_MSG_DOOR_CONTROL,
                          .msg.door_control = {DOOR_CMD_OPEN}};
        int ret = k_msgq_put(&ctrl_queue, &msg, K_MSEC(1000));
        if (ret != 0) {
            LOG_ERR("Failed to enqueue open msg: %d", ret);
            res->status = HTTP_500_INTERNAL_SERVER_ERROR;
        } else {
            res->status = HTTP_200_OK;
        }
        res->body_len = 0;
        res->body = NULL;
        res->final_chunk = true;
    }
    return 0;
}

static int close_handler(struct http_client_ctx* client,
                         enum http_transaction_status status,
                         const struct http_request_ctx* req,
                         struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        LOG_INF("close_handler called!");
        if (check_pic_updating(res)) {
            return 0;
        }
        ctrl_msg_t msg = {.type = CTRL_MSG_DOOR_CONTROL,
                          .msg.door_control = {DOOR_CMD_CLOSE}};
        int ret = k_msgq_put(&ctrl_queue, &msg, K_MSEC(1000));
        if (ret != 0) {
            LOG_ERR("Failed to enqueue close msg: %d", ret);
            res->status = HTTP_500_INTERNAL_SERVER_ERROR;
        } else {
            res->status = HTTP_200_OK;
        }
        res->body_len = 0;
        res->body = NULL;
        res->final_chunk = true;
    }
    return 0;
}

static int light_handler(struct http_client_ctx* client,
                         enum http_transaction_status status,
                         const struct http_request_ctx* req,
                         struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        if (check_pic_updating(res)) {
            return 0;
        }
        if (client->method == HTTP_GET) {
            bool on = pic_get_light();
            static char buf[32];
            int len = snprintf(buf, sizeof(buf), "{\"light\":%s}\n",
                               on ? "true" : "false");
            static const struct http_header headers[] = {
                {.name = "Content-Type", .value = "application/json"}};
            res->status = HTTP_200_OK;
            res->headers = headers;
            res->header_count = 1;
            res->body = (const uint8_t*)buf;
            res->body_len = len;
            res->final_chunk = true;
        } else if (client->method == HTTP_POST || client->method == HTTP_PUT) {
            bool turn_on = false;
            if (req->data && req->data_len > 0) {
                char body[64] = {0};
                size_t cpy_len = req->data_len < sizeof(body) - 1
                                     ? req->data_len
                                     : sizeof(body) - 1;
                memcpy(body, req->data, cpy_len);
                body[cpy_len] = '\0';
                if (strstr(body, "true") || strstr(body, "1") ||
                    strstr(body, "\"on\"") || strstr(body, "ON") ||
                    strstr(body, ":true")) {
                    turn_on = true;
                }
            }
            LOG_INF("HTTP light control: %s", turn_on ? "ON" : "OFF");
            ctrl_msg_t msg = {.type = CTRL_MSG_LIGHT_CONTROL,
                              .msg.light_control = {.on = turn_on}};
            int ret = k_msgq_put(&ctrl_queue, &msg, K_MSEC(1000));
            static char buf[32];
            int len = snprintf(buf, sizeof(buf), "{\"light\":%s}\n",
                               turn_on ? "true" : "false");
            static const struct http_header headers[] = {
                {.name = "Content-Type", .value = "application/json"}};
            res->status =
                (ret == 0) ? HTTP_200_OK : HTTP_500_INTERNAL_SERVER_ERROR;
            res->headers = headers;
            res->header_count = 1;
            res->body = (const uint8_t*)buf;
            res->body_len = len;
            res->final_chunk = true;
        }
    }
    return 0;
}

static bool s_picupdate_in_progress;
static bool s_picupdate_failed;
static struct http_client_ctx* s_picupdate_client;
static uint32_t s_picupdate_cooldown_until;

static int picupdate_handler(struct http_client_ctx* client,
                             enum http_transaction_status status,
                             const struct http_request_ctx* req,
                             struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_TRANSACTION_ABORTED) {
        if (s_picupdate_in_progress && client == s_picupdate_client) {
            LOG_WRN("PIC update connection aborted by client");
            pic_updater_abort();
            s_picupdate_in_progress = false;
            s_picupdate_client = NULL;
            s_picupdate_cooldown_until = k_uptime_get_32() + 15000;
        }
        return 0;
    }

    if (status == HTTP_SERVER_TRANSACTION_COMPLETE) {
        return 0;
    }

    uint32_t now = k_uptime_get_32();
    if (now < s_picupdate_cooldown_until) {
        if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
            static const struct http_header headers[] = {
                {.name = "Content-Type", .value = "application/json"}};
            static const char busy_body[] =
                "{\"status\":\"error\",\"message\":\"PIC update cooldown in "
                "progress\"}\n";
            res->headers = headers;
            res->header_count = 1;
            res->status = HTTP_503_SERVICE_UNAVAILABLE;
            res->body = (const uint8_t*)busy_body;
            res->body_len = sizeof(busy_body) - 1;
            res->final_chunk = true;
        }
        return 0;
    }

    if (s_picupdate_in_progress && client != s_picupdate_client) {
        if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
            static const struct http_header headers[] = {
                {.name = "Content-Type", .value = "application/json"}};
            static const char busy_body[] =
                "{\"status\":\"error\",\"message\":\"PIC update already in "
                "progress\"}\n";
            res->headers = headers;
            res->header_count = 1;
            res->status = HTTP_503_SERVICE_UNAVAILABLE;
            res->body = (const uint8_t*)busy_body;
            res->body_len = sizeof(busy_body) - 1;
            res->final_chunk = true;
        }
        return 0;
    }

    if (!s_picupdate_in_progress) {
        s_picupdate_in_progress = true;
        s_picupdate_failed = false;
        s_picupdate_client = client;
        pic_updater_reset();
    }

    if (!s_picupdate_failed && req->data && req->data_len > 0) {
        int ret = pic_updater_process_chunk(req->data, req->data_len);
        if (ret != 0) {
            LOG_ERR("pic_updater_process_chunk failed: %d", ret);
            s_picupdate_failed = true;
        }
    }

    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        int ret = 0;
        if (!s_picupdate_failed) {
            ret = pic_updater_complete();
        } else {
            pic_updater_abort();
            ret = -1;
        }
        s_picupdate_in_progress = false;
        s_picupdate_client = NULL;

        static const struct http_header headers[] = {
            {.name = "Content-Type", .value = "application/json"}};
        res->headers = headers;
        res->header_count = 1;

        if (ret == 0 && !s_picupdate_failed) {
            s_picupdate_cooldown_until = k_uptime_get_32() + 5000;
            static const char ok_body[] = "{\"status\":\"success\"}\n";
            res->status = HTTP_200_OK;
            res->body = (const uint8_t*)ok_body;
            res->body_len = sizeof(ok_body) - 1;
        } else {
            s_picupdate_cooldown_until = k_uptime_get_32() + 15000;
            static const char err_body[] = "{\"status\":\"error\"}\n";
            res->status = HTTP_500_INTERNAL_SERVER_ERROR;
            res->body = (const uint8_t*)err_body;
            res->body_len = sizeof(err_body) - 1;
        }
        res->final_chunk = true;
    }
    return 0;
}

static bool s_mcuboot_update_in_progress;
static bool s_mcuboot_update_failed;
static struct http_client_ctx* s_mcuboot_update_client;

static int mcuboot_upgrade_handler(struct http_client_ctx* client,
                                   enum http_transaction_status status,
                                   const struct http_request_ctx* req,
                                   struct http_response_ctx* res,
                                   void* user_data) {
    if (status == HTTP_SERVER_TRANSACTION_ABORTED) {
        if (s_mcuboot_update_in_progress && client == s_mcuboot_update_client) {
            LOG_WRN("MCUboot upgrade connection aborted by client");
            mcuboot_updater_abort();
            s_mcuboot_update_in_progress = false;
            s_mcuboot_update_client = NULL;
        }
        return 0;
    }

    if (status == HTTP_SERVER_TRANSACTION_COMPLETE) {
        return 0;
    }

    if (s_mcuboot_update_in_progress && client != s_mcuboot_update_client) {
        if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
            static const struct http_header headers[] = {
                {.name = "Content-Type", .value = "application/json"}};
            static const char busy_body[] =
                "{\"status\":\"error\",\"message\":\"MCUboot upgrade already "
                "in "
                "progress\"}\n";
            res->headers = headers;
            res->header_count = 1;
            res->status = HTTP_503_SERVICE_UNAVAILABLE;
            res->body = (const uint8_t*)busy_body;
            res->body_len = sizeof(busy_body) - 1;
            res->final_chunk = true;
        }
        return 0;
    }

    if (!s_mcuboot_update_in_progress) {
        s_mcuboot_update_in_progress = true;
        s_mcuboot_update_failed = false;
        s_mcuboot_update_client = client;
        mcuboot_updater_reset();
    }

    if (!s_mcuboot_update_failed && req->data && req->data_len > 0) {
        int ret = mcuboot_updater_process_chunk(req->data, req->data_len);
        if (ret != 0) {
            LOG_ERR("mcuboot_updater_process_chunk failed: %d", ret);
            s_mcuboot_update_failed = true;
        }
    }

    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        int ret = 0;
        if (!s_mcuboot_update_failed) {
            ret = mcuboot_updater_complete();
        } else {
            mcuboot_updater_abort();
            ret = -1;
        }
        s_mcuboot_update_in_progress = false;
        s_mcuboot_update_client = NULL;

        static const struct http_header headers[] = {
            {.name = "Content-Type", .value = "application/json"}};
        res->headers = headers;
        res->header_count = 1;

        if (ret == 0 && !s_mcuboot_update_failed) {
            static const char ok_body[] = "{\"status\":\"success\"}\n";
            res->status = HTTP_200_OK;
            res->body = (const uint8_t*)ok_body;
            res->body_len = sizeof(ok_body) - 1;

            ctrl_msg_t msg = {.type = CTRL_MSG_RESTART};
            k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
        } else {
            static const char err_body[] =
                "{\"status\":\"error\",\"message\":\"MCUboot upgrade "
                "failed\"}\n";
            res->status = HTTP_400_BAD_REQUEST;
            res->body = (const uint8_t*)err_body;
            res->body_len = sizeof(err_body) - 1;
        }
        res->final_chunk = true;
    }
    return 0;
}

static int mcuboot_promote_handler(struct http_client_ctx* client,
                                   enum http_transaction_status status,
                                   const struct http_request_ctx* req,
                                   struct http_response_ctx* res,
                                   void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        LOG_INF("mcuboot_promote_handler called!");
        int ret = mcuboot_updater_promote();
        static const struct http_header headers[] = {
            {.name = "Content-Type", .value = "application/json"}};
        res->headers = headers;
        res->header_count = 1;

        if (ret == 0) {
            static const char ok_body[] = "{\"status\":\"success\"}\n";
            res->status = HTTP_200_OK;
            res->body = (const uint8_t*)ok_body;
            res->body_len = sizeof(ok_body) - 1;
        } else {
            static const char err_body[] =
                "{\"status\":\"error\",\"message\":\"Failed to promote "
                "MCUboot\"}\n";
            res->status = HTTP_500_INTERNAL_SERVER_ERROR;
            res->body = (const uint8_t*)err_body;
            res->body_len = sizeof(err_body) - 1;
        }
        res->final_chunk = true;
    }
    return 0;
}

#ifdef CONFIG_CHIP
static int matter_commission_handler(struct http_client_ctx* client,
                                     enum http_transaction_status status,
                                     const struct http_request_ctx* req,
                                     struct http_response_ctx* res,
                                     void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        const bool ok = matter_commission_open(900);
        res->status = ok ? HTTP_200_OK : HTTP_500_INTERNAL_SERVER_ERROR;
        res->body_len = 0;
        res->body = NULL;
        res->final_chunk = true;
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
        int ret = k_msgq_put(&ctrl_queue, &msg, K_MSEC(1000));
        res->status = (ret == 0) ? HTTP_200_OK : HTTP_500_INTERNAL_SERVER_ERROR;
        res->body_len = 0;
        res->body = NULL;
        res->final_chunk = true;
    }
    return 0;
}

static int matter_info_handler(struct http_client_ctx* client,
                               enum http_transaction_status status,
                               const struct http_request_ctx* req,
                               struct http_response_ctx* res, void* user_data) {
    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        static char buf[2048];
        matter_get_fabric_info_json(buf, sizeof(buf));
        static const struct http_header headers[] = {
            {.name = "Content-Type", .value = "application/json"}};
        res->status = HTTP_200_OK;
        res->headers = headers;
        res->header_count = 1;
        res->body_len = strlen(buf);
        res->body = (const uint8_t*)buf;
        res->final_chunk = true;
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

static struct http_resource_detail_dynamic time_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_GET)},
    .cb = time_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic version_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_GET)},
    .cb = version_handler,
    .user_data = NULL,
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

#include "logs_page.h"

struct logs_tx_state {
    struct log_chunks chunks;
    int step;
};

static struct logs_tx_state s_logs_tx;

static int logs_handler(struct http_client_ctx* client,
                        enum http_transaction_status status,
                        const struct http_request_ctx* req,
                        struct http_response_ctx* res, void* user_data) {
    static const struct http_header headers[] = {
        {.name = "Content-Type", .value = "text/plain; charset=utf-8"}};

    if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
        const char* url = (const char*)client->url_buffer;
        if (strstr(url, "?view") || strstr(url, "?html")) {
            static const struct http_header redir_headers[] = {
                {.name = "Location", .value = "/logs.html"}};
            res->status = HTTP_307_TEMPORARY_REDIRECT;
            res->headers = redir_headers;
            res->header_count = 1;
            res->body_len = 0;
            res->body = NULL;
            res->final_chunk = true;
            return 0;
        }

        if (s_logs_tx.step == 0) {
            log_ring_buf_get_chunks(&s_logs_tx.chunks);
            res->status = HTTP_200_OK;
            res->headers = headers;
            res->header_count = 1;

            if (s_logs_tx.chunks.len1 == 0 && s_logs_tx.chunks.len2 == 0) {
                static const char empty_msg[] = "(No logs in buffer)\n";
                res->body = (const uint8_t*)empty_msg;
                res->body_len = sizeof(empty_msg) - 1;
                res->final_chunk = true;
                s_logs_tx.step = 0;
                return 0;
            }

            res->body = (const uint8_t*)s_logs_tx.chunks.chunk1;
            res->body_len = s_logs_tx.chunks.len1;

            if (s_logs_tx.chunks.len2 > 0) {
                res->final_chunk = false;
                s_logs_tx.step = 1;
            } else {
                res->final_chunk = true;
                s_logs_tx.step = 0;
            }
            return 0;
        } else if (s_logs_tx.step == 1) {
            res->body = (const uint8_t*)s_logs_tx.chunks.chunk2;
            res->body_len = s_logs_tx.chunks.len2;
            res->final_chunk = true;
            s_logs_tx.step = 0;
            return 0;
        }
    } else if (status == HTTP_SERVER_TRANSACTION_COMPLETE ||
               status == HTTP_SERVER_TRANSACTION_ABORTED) {
        s_logs_tx.step = 0;
    }
    return 0;
}

static struct http_resource_detail_dynamic logs_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_GET)},
    .cb = logs_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic light_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods =
                   BIT(HTTP_GET) | BIT(HTTP_POST) | BIT(HTTP_PUT)},
    .cb = light_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic picupdate_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = picupdate_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic mcuboot_upgrade_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = mcuboot_upgrade_handler,
    .user_data = NULL,
};

static struct http_resource_detail_dynamic mcuboot_promote_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_POST)},
    .cb = mcuboot_promote_handler,
    .user_data = NULL,
};

static struct http_resource_detail_static logs_page_detail = {
    .common =
        {
            .type = HTTP_RESOURCE_TYPE_STATIC,
            .bitmask_of_supported_http_methods = BIT(HTTP_GET),
            .content_type = "text/html",
        },
    .static_data = logs_html,
    .static_data_len = sizeof(logs_html) - 1,
};

#ifdef CONFIG_CHIP
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

static struct http_resource_detail_dynamic matter_info_detail = {
    .common = {.type = HTTP_RESOURCE_TYPE_DYNAMIC,
               .bitmask_of_supported_http_methods = BIT(HTTP_GET)},
    .cb = matter_info_handler,
    .user_data = NULL,
};

#include "matter_page.h"

static struct http_resource_detail_static matter_page_detail = {
    .common =
        {
            .type = HTTP_RESOURCE_TYPE_STATIC,
            .bitmask_of_supported_http_methods = BIT(HTTP_GET),
            .content_type = "text/html",
        },
    .static_data = matter_html,
    .static_data_len = sizeof(matter_html) - 1,
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
HTTP_RESOURCE_DEFINE(mcuboot_upgrade_resource, httpd_service,
                     "/fwupgrade/mcuboot", &mcuboot_upgrade_detail);
HTTP_RESOURCE_DEFINE(mcuboot_promote_resource, httpd_service,
                     "/promote/mcuboot", &mcuboot_promote_detail);
HTTP_RESOURCE_DEFINE(promote_resource, httpd_service, "/promote",
                     &promote_detail);

HTTP_RESOURCE_DEFINE(open_resource, httpd_service, "/open", &open_detail);
HTTP_RESOURCE_DEFINE(close_resource, httpd_service, "/close", &close_detail);
HTTP_RESOURCE_DEFINE(light_resource, httpd_service, "/light", &light_detail);
HTTP_RESOURCE_DEFINE(picupdate_resource, httpd_service, "/picupdate",
                     &picupdate_detail);
HTTP_RESOURCE_DEFINE(time_resource, httpd_service, "/time", &time_detail);
HTTP_RESOURCE_DEFINE(version_resource, httpd_service, "/version",
                     &version_detail);
HTTP_RESOURCE_DEFINE(logs_resource, httpd_service, "/logs", &logs_detail);
HTTP_RESOURCE_DEFINE(logs_html_resource, httpd_service, "/logs.html",
                     &logs_page_detail);
HTTP_RESOURCE_DEFINE(logs_view_resource, httpd_service, "/logs/view",
                     &logs_page_detail);
HTTP_RESOURCE_DEFINE(ws_logs_resource, httpd_service, "/ws/logs",
                     &ws_logs_resource_detail);

#ifdef CONFIG_CHIP
HTTP_RESOURCE_DEFINE(matter_commission_resource, httpd_service,
                     "/matter/commission", &matter_commission_detail);
HTTP_RESOURCE_DEFINE(matter_reset_resource, httpd_service, "/matter/reset",
                     &matter_reset_detail);
HTTP_RESOURCE_DEFINE(matter_info_resource, httpd_service, "/matter/info",
                     &matter_info_detail);
HTTP_RESOURCE_DEFINE(matter_page_resource, httpd_service, "/matter",
                     &matter_page_detail);
HTTP_RESOURCE_DEFINE(matter_slash_page_resource, httpd_service, "/matter/",
                     &matter_page_detail);
#endif

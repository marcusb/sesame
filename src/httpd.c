#include "httpd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "core_json.h"
#include "queue.h"
#include "task.h"

// FreeRTOS+TCP
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

// application
#include "app_logging.h"
#include "controller.h"
#include "util.h"

#define BUF_SIZE 1024

typedef enum { HTTP_INIT, HTTP_HEADER, HTTP_BODY } http_state_t;

enum {
    REPLY_OK = 200,
    NO_CONTENT = 204,
    BAD_REQUEST = 400,
    UNAUTHORIZED = 401,
    NOT_FOUND = 404,
    GONE = 410,
    PRECONDITION_FAILED = 412,
    INTERNAL_SERVER_ERROR = 500,
};

typedef enum {
    METHOD_GET,
    METHOD_HEAD,
    METHOD_POST,
    METHOD_PUT,
    METHOD_DELETE,
    METHOD_TRACE,
    METHOD_OPTIONS,
    METHOD_CONNECT,
    METHOD_PATCH,
    METHOD_UNK,
} http_method_t;

typedef struct http_method_desc {
    int method_length;
    const char* name;
    http_method_t type;
} http_method_desc_t;

void reboot(void);

static QueueHandle_t ctrl_queue;

static const http_method_desc_t http_methods[] = {
    {3, "GET", METHOD_GET},         {4, "HEAD", METHOD_HEAD},
    {4, "POST", METHOD_POST},       {3, "PUT", METHOD_PUT},
    {6, "DELETE", METHOD_DELETE},   {5, "TRACE", METHOD_TRACE},
    {7, "OPTIONS", METHOD_OPTIONS}, {7, "CONNECT", METHOD_CONNECT},
    {5, "PATCH", METHOD_PATCH},     {0, "", METHOD_UNK},
};

typedef struct {
    Socket_t socket;
    char* buf;
    http_state_t state;
    http_method_t method;
    const char* url;
    char* header_start;
    char* body_start;
    int content_length;
} http_request_t;

static const char* status_desc(int aCode) {
    switch (aCode) {
        case REPLY_OK: /*  = 200, */
            return "OK";

        case NO_CONTENT: /* 204 */
            return "No content";

        case BAD_REQUEST: /*  = 400, */
            return "Bad request";

        case UNAUTHORIZED: /*  = 401, */
            return "Authorization Required";

        case NOT_FOUND: /*  = 404, */
            return "Not Found";

        case GONE: /*  = 410, */
            return "Done";

        case PRECONDITION_FAILED: /*  = 412, */
            return "Precondition Failed";

        case INTERNAL_SERVER_ERROR: /*  = 500, */
            return "Internal Server Error";
    }

    return "Unknown";
}

static void handle_fwupgrade() {
    ctrl_msg_t msg = {CTRL_MSG_OTA_UPGRADE};
    xQueueSendToBack(ctrl_queue, &msg, 100);
}

static void send_status(const http_request_t* req, int status) {
    int len = snprintf(req->buf, BUF_SIZE,
                       "HTTP/1.1 %d %s\r\n"
                       "Connection: close\r\n",
                       status, status_desc(status));
    FreeRTOS_send(req->socket, req->buf, len, 0);
}

void process_cfg(const http_request_t* req) {
    JSONStatus_t res = JSON_Validate(req->body_start, req->content_length);
    if (res != JSONSuccess) {
        goto bad_req;
    }
    ctrl_msg_t msg = {CTRL_MSG_WIFI_CONFIG};
    const char ssid_key[] = "wifi.ssid";
    const char passwd_key[] = "wifi.passwd";
    const char hostname_key[] = "wifi.hostname";
    char* value;
    size_t len;

    res = JSON_Search(req->body_start, req->content_length, ssid_key,
                      sizeof(ssid_key) - 1, &value, &len);
    if (res == JSONSuccess) {
        len = min(len, wificonfigMAX_SSID_LEN);
        memcpy(msg.msg.wifi_cfg.network_params.ucSSID, value, len);
        msg.msg.wifi_cfg.network_params.ucSSIDLength = len;
    }

    res = JSON_Search(req->body_start, req->content_length, passwd_key,
                      sizeof(passwd_key) - 1, &value, &len);
    if (res == JSONSuccess) {
        len = min(len, wificonfigMAX_PASSPHRASE_LEN);
        memcpy(msg.msg.wifi_cfg.network_params.xPassword.xWPA.cPassphrase,
               value, len);
        msg.msg.wifi_cfg.network_params.xPassword.xWPA.ucLength = len;
    }

    res = JSON_Search(req->body_start, req->content_length, hostname_key,
                      sizeof(hostname_key) - 1, &value, &len);
    if (res == JSONSuccess) {
        len = min(len, sizeof(msg.msg.wifi_cfg.hostname) - 1);
        memcpy(msg.msg.wifi_cfg.hostname, value, len);
        msg.msg.wifi_cfg.hostname[len] = '\0';
    }

    xQueueSendToBack(ctrl_queue, &msg, 100);
    send_status(req, REPLY_OK);
    return;

bad_req:
    send_status(req, BAD_REQUEST);
}

static void do_request(const http_request_t* req) {
    switch (req->method) {
        case METHOD_POST:
            if (strcmp(req->url, "/fwupgrade") == 0) {
                handle_fwupgrade();
                send_status(req, REPLY_OK);
            } else if (strcmp(req->url, "/open") == 0) {
                ctrl_msg_t msg = {CTRL_MSG_DOOR_CONTROL,
                                  {.door_control = {DOOR_CMD_OPEN}}};
                xQueueSendToBack(ctrl_queue, &msg, 100);
                send_status(req, REPLY_OK);
            } else if (strcmp(req->url, "/close") == 0) {
                ctrl_msg_t msg = {CTRL_MSG_DOOR_CONTROL,
                                  {.door_control = {DOOR_CMD_CLOSE}}};
                xQueueSendToBack(ctrl_queue, &msg, 100);
                send_status(req, REPLY_OK);
            } else if (strcmp(req->url, "/cfg/network") == 0) {
                process_cfg(req);
            } else if (strcmp(req->url, "/restart") == 0) {
                send_status(req, REPLY_OK);
                LogDebug(("reboot requested, rebooting..."));
                vTaskDelay(pdMS_TO_TICKS(3000));
                reboot();
            } else {
                send_status(req, NOT_FOUND);
            }
            break;
        default:
            send_status(req, NOT_FOUND);
            break;
    }
}

static int process_request_line(http_request_t* req) {
    char* p = req->buf;
    for (const http_method_desc_t* method = http_methods;
         method->type != METHOD_UNK; method++) {
        int method_len = method->method_length;
        if (strncmp(method->name, p, method->method_length) == 0 &&
            p[method_len] == ' ') {
            req->method = method->type;
            p += method_len + 1;
            req->url = p;
            while (*p != ' ' && *p != '\0') {
                p++;
            }
            if (*p == '\0') {
                return -1;
            }
            *p++ = '\0';
            if (strcmp(p, "HTTP/1.1") != 0) {
                return -1;
            }
            LogDebug(("request %.*s %s", method->method_length, method->name,
                      req->url));
            return 0;
        }
    }
    return -1;
}

static char* get_header(const http_request_t* req, const char* name) {
    const int name_len = strlen(name);
    char* p = req->header_start;
    while (p < req->body_start) {
        char* q = strchrnul(p, ':');
        if (*q == ':') {
            while (*p == ' ') {
                p++;
            }
            if (strncasecmp(p, name, name_len) == 0) {
                p += name_len;
                while (*p == ' ') {
                    p++;
                }
                if (p == q) {
                    p++;
                    return p;
                }
            }
        }
        // skip CRLF, to next header field
        p = q + 2;
    }
    return NULL;
}

static void request_task(void* params) {
    Socket_t socket = (Socket_t)params;
    char* buf = malloc(BUF_SIZE);
    if (!buf) {
        LogError(("malloc failed"));
        goto close_conn;
    }
    const char* buf_end = buf + BUF_SIZE;

    http_request_t req = {socket, buf, HTTP_INIT};
    char* wr_pos = buf;
    char* rd_pos = buf;
    char* mark = buf;
    BaseType_t res;
    for (;;) {
        if (mark == rd_pos) {
            // rd_pos has not moved, no more input was consumed, so need to read
            // more
            res = FreeRTOS_recv(socket, wr_pos, buf_end - wr_pos, 0);
            wr_pos += res;
            if (wr_pos == buf_end) {
                LogDebug(("request too big"));
                goto close_conn;
            }
        }
        mark = rd_pos;
        if (res > 0) {
            switch (req.state) {
                case HTTP_INIT:
                    debug_hexdump('R', buf, wr_pos - buf);
                    for (char* q = buf; q < wr_pos - 1; q++) {
                        if (*q == '\r' && *(q + 1) == '\n') {
                            *q = '\0';
                            process_request_line(&req);
                            req.state = HTTP_HEADER;
                            req.header_start = rd_pos = q + 2;
                            break;
                        }
                    }
                    break;
                case HTTP_HEADER:
                    if (wr_pos >= rd_pos + 2 && *rd_pos == '\r' &&
                        *(rd_pos + 1) == '\n') {
                        rd_pos += 2;
                        req.body_start = rd_pos;
                        char* q = get_header(&req, "content-length");
                        if (q) {
                            req.content_length = atoi(q);
                        } else {
                            req.content_length = 0;
                        }
                        req.state = HTTP_BODY;
                        if (buf_end - wr_pos < req.content_length) {
                            goto close_conn;
                        }
                    } else {
                        for (char* q = rd_pos; q < wr_pos - 1; q++) {
                            if (*q == '\r' && *(q + 1) == '\n') {
                                *q = '\0';
                                rd_pos = q + 2;
                                break;
                            }
                        }
                    }
                    break;

                case HTTP_BODY:
                    if (wr_pos - req.body_start >= req.content_length) {
                        do_request(&req);
                        goto close_conn;
                    }
            }
        } else if (res < 0) {
            goto err;
        }
    }

err:
    FreeRTOS_printf(("recv() returned %d\n", res));

close_conn:
    FreeRTOS_shutdown(socket, FREERTOS_SHUT_RDWR);
    for (int i = 0; i < 20 && FreeRTOS_recv(socket, buf, BUF_SIZE, 0) >= 0;
         i++) {
        vTaskDelay(pdTICKS_TO_MS(250));
    }
    FreeRTOS_closesocket(socket);
    if (buf) {
        free(buf);
    }
    vTaskDelete(NULL);
}

void httpd_task(void* params) {
    ctrl_queue = (QueueHandle_t)params;
    const BaseType_t backlog = 20;
    Socket_t socket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM,
                                      FREERTOS_IPPROTO_TCP);
    configASSERT(socket != FREERTOS_INVALID_SOCKET);

    const TickType_t recv_tmout = portMAX_DELAY;
    FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_RCVTIMEO, &recv_tmout, 0);

    uint16_t port = 80;
    struct freertos_sockaddr bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_port = FreeRTOS_htons(port);
    bind_addr.sin_family = FREERTOS_AF_INET;
    BaseType_t res = FreeRTOS_bind(socket, &bind_addr, sizeof(bind_addr));
    if (res) {
        LogError(("bind failed %d", res));
        goto ret;
    }
    FreeRTOS_listen(socket, backlog);
    LogInfo(("HTTP server listening on port %d", port));

    for (;;) {
        struct freertos_sockaddr client;
        socklen_t sz = sizeof(client);
        Socket_t conn = FreeRTOS_accept(socket, &client, &sz);
        configASSERT(conn != FREERTOS_INVALID_SOCKET);

        xTaskCreate(request_task, "HttpWorker", 512, (void*)conn,
                    tskIDLE_PRIORITY, NULL);
    }

ret:
    vTaskDelete(NULL);
}

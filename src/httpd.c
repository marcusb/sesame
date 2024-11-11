#include "httpd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "app_logging.h"
#include "controller.h"
#include "queue.h"
#include "task.h"

#define BUF_SIZE 1024

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

enum HttpMethod {
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
};
#define METHODS_COUNT (METHOD_UNK + 1)

typedef struct http_method_desc {
    BaseType_t method_length;
    const char* name;
    const unsigned char type;
} http_method_desc_t;

static QueueHandle_t ctrl_queue;

static const http_method_desc_t http_methods[METHODS_COUNT] = {
    {3, "GET", METHOD_GET},         {4, "HEAD", METHOD_HEAD},
    {4, "POST", METHOD_POST},       {3, "PUT", METHOD_PUT},
    {6, "DELETE", METHOD_DELETE},   {5, "TRACE", METHOD_TRACE},
    {7, "OPTIONS", METHOD_OPTIONS}, {7, "CONNECT", METHOD_CONNECT},
    {5, "PATCH", METHOD_PATCH},     {4, "UNKN", METHOD_UNK},
};

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

// static char empty[1] = {'\0'};

static void handle_fwupgrade() {
    ctrl_msg_t msg = {CTRL_MSG_OTA_UPGRADE};
    xQueueSendToBack(ctrl_queue, &msg, 100);
}

static void send_status(Socket_t socket, char* buf, int status) {
    int len = snprintf(buf, BUF_SIZE,
                       "HTTP/1.1 %d %s\r\n"
                       "Connection: close\r\n",
                       status, status_desc(status));
    FreeRTOS_send(socket, buf, len, 0);
}

static void do_request(Socket_t socket, char* buf,
                       const http_method_desc_t* method, const char* url) {
    LogDebug(("request %s %s", method->name, url));
    switch (method->type) {
        case METHOD_POST:
            if (strcmp(url, "/fwupgrade") == 0) {
                handle_fwupgrade();
                send_status(socket, buf, REPLY_OK);
                return;
            } else if (strcmp(url, "/open") == 0) {
                ctrl_msg_t msg = {CTRL_MSG_DOOR_CONTROL,
                                  {.door_control = {DOOR_CMD_OPEN}}};
                xQueueSendToBack(ctrl_queue, &msg, 100);
            } else if (strcmp(url, "/close") == 0) {
                ctrl_msg_t msg = {CTRL_MSG_DOOR_CONTROL,
                                  {.door_control = {DOOR_CMD_CLOSE}}};
                xQueueSendToBack(ctrl_queue, &msg, 100);
            } else {
                send_status(socket, buf, NOT_FOUND);
                break;
            }
            send_status(socket, buf, REPLY_OK);
            break;
        default:
            send_status(socket, buf, NOT_FOUND);
            break;
    }
}

static void request_task(void* params) {
    Socket_t socket = (Socket_t)params;
    char* buf = malloc(BUF_SIZE);
    if (!buf) {
        LogError(("malloc failed"));
        goto ret;
    }

    BaseType_t res = FreeRTOS_recv(socket, buf, BUF_SIZE, 0);
    if (res > 0) {
        char* p = buf;

        if (res < BUF_SIZE) {
            buf[res] = '\0';
        }
        while (res && (buf[res - 1] == 13 || buf[res - 1] == 10)) {
            p[--res] = '\0';
        }

        const char* end = p + res;
        const http_method_desc_t* method = http_methods;
        char* url = p;
        // char* proto = empty;

        int i;
        for (i = 0; i < METHODS_COUNT - 1; i++, method++) {
            BaseType_t method_len = method->method_length;
            if (res >= method_len && memcmp(method->name, p, method_len) == 0) {
                url += method_len + 1;

                for (char* last = url; last < end; last++) {
                    char ch = *last;
                    if (ch == '\0' || strchr("\n\r \t", ch) != NULL) {
                        *last = '\0';
                        // proto = last + 1;
                        break;
                    }
                }
                break;
            }
        }

        if (i < (METHODS_COUNT - 1)) {
            do_request(socket, buf, method, url);
        }
    } else if (res < 0) {
        /* The connection will be closed and the client will be deleted. */
        FreeRTOS_printf(("recv() returned %d\n", res));
    }

ret:
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
    static const TickType_t recv_tmout = portMAX_DELAY;
    const BaseType_t backlog = 20;
    Socket_t socket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM,
                                      FREERTOS_IPPROTO_TCP);
    configASSERT(socket != FREERTOS_INVALID_SOCKET);

    FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_RCVTIMEO, &recv_tmout,
                        sizeof(recv_tmout));

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

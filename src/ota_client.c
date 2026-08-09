#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/http/client.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/netdb.h>

#include "ota.h"

extern void feed_watchdog(void);

LOG_MODULE_REGISTER(ota_client, LOG_LEVEL_INF);

#define READ_SIZE 1024

static const char http_prefix[] = "http://";

int parse_url(char* url, char* hostname, size_t max_hostname_len, char** path,
              uint16_t* port) {
    static const int prefix_len = sizeof(http_prefix) - 1;
    if (strncmp(url, http_prefix, prefix_len)) {
        return -1;
    }
    char* p = url + prefix_len;
    if (*p == '\0') {
        return -1;
    }
    char* q = strchr(p, '/');
    if (q == NULL) {
        *path = "/";
    } else {
        *q = '\0';
        *path = q + 1;  // leave without slash, we'll add it in format
    }

    if ((q = strchr(p, ':'))) {
        *q = '\0';
        *port = atoi(q + 1);
    } else {
        *port = 80;
    }

    strncpy(hostname, p, max_hostname_len - 1);
    hostname[max_hostname_len - 1] = '\0';
    return 0;
}

static int response_cb(struct http_response* rsp,
                       enum http_final_call final_data, void* user_data) {
    ota_upd_state_t* ota_state = user_data;

    if (rsp->body_frag_len > 0) {
        feed_watchdog();
        if (ota_write_chunk(ota_state, rsp->body_frag_start,
                            rsp->body_frag_len) < 0) {
            LOG_ERR("Failed to write OTA chunk");
            return -1;
        }
    }

    if (final_data == HTTP_DATA_FINAL) {
        LOG_INF("All data received");
    }

    return 0;
}

void ota_client_start(const FirmwareUpgradeFetchRequest* msg) {
    char url[128];
    strncpy(url, msg->url, sizeof(url) - 1);
    url[sizeof(url) - 1] = '\0';

    char hostname[64];
    char* path = NULL;
    uint16_t port = 0;

    if (parse_url(url, hostname, sizeof(hostname), &path, &port) != 0) {
        LOG_ERR("invalid URL: %s", msg->url);
        return;
    }

    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo* res;

    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", port);

    int ret = zsock_getaddrinfo(hostname, port_str, &hints, &res);
    if (ret != 0) {
        LOG_ERR("getaddrinfo failed: %d", ret);
        return;
    }

    int sock = zsock_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        LOG_ERR("socket failed");
        zsock_freeaddrinfo(res);
        return;
    }

    if (zsock_connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
        LOG_ERR("connect failed");
        zsock_close(sock);
        zsock_freeaddrinfo(res);
        return;
    }
    zsock_freeaddrinfo(res);

    ota_upd_state_t ota_state;
    if (ota_init(&ota_state) < 0) {
        zsock_close(sock);
        return;
    }

    struct http_request req;
    memset(&req, 0, sizeof(req));

    req.method = HTTP_GET;
    req.url = path;
    req.host = hostname;
    req.protocol = "HTTP/1.1";
    req.response = response_cb;
    req.recv_buf = malloc(READ_SIZE);
    if (!req.recv_buf) {
        LOG_ERR("Failed to allocate recv buf");
        ota_finish(&ota_state);
        zsock_close(sock);
        return;
    }
    req.recv_buf_len = READ_SIZE;

    LOG_INF("Starting HTTP GET /%s from %s:%d", path, hostname, port);

    ret = http_client_req(sock, &req, 10000, &ota_state);
    if (ret < 0) {
        LOG_ERR("http_client_req failed: %d", ret);
    }

    free(req.recv_buf);
    zsock_close(sock);

    ota_finish(&ota_state);
}

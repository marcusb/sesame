#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>
LOG_MODULE_REGISTER(mqtt, LOG_LEVEL_DBG);

#include <zephyr/sys/sys_heap.h>

extern struct k_heap _system_heap;

#include "app_config.pb.h"
#include "config_manager.h"
#include "controller.h"
#include "mqtt.h"
#include "time_util.h"

#define MQTT_CLIENTID "sesame_client"
#define MQTT_RX_BUF_SIZE 512
#define MQTT_TX_BUF_SIZE 512
#define SECONDS_PER_DAY (86400)

static uint8_t rx_buffer[MQTT_RX_BUF_SIZE];
static uint8_t tx_buffer[MQTT_TX_BUF_SIZE];
static struct mqtt_client client_ctx;
static struct sockaddr_in broker;
static struct zsock_pollfd fds[1];
static int nfds;
static bool connected = false;

static char state_topic[64];
static char lwt_topic[64];
static char cmd_topic[64];

#define LWT_ONLINE "Online"
#define LWT_OFFLINE "Offline"

static void prepare_fds(struct mqtt_client* client) {
    if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
        fds[0].fd = client->transport.tcp.sock;
    }
    fds[0].events = ZSOCK_POLLIN;
    nfds = 1;
}

static void clear_fds(void) { nfds = 0; }

static int wait(int timeout) {
    int ret = 0;
    if (nfds > 0) {
        ret = zsock_poll(fds, nfds, timeout);
        if (ret < 0) {
            LOG_ERR("poll error: %d", errno);
        }
    }
    return ret;
}

static void publish(const char* topic, const char* payload, bool retain) {
    if (!connected) {
        return;
    }

    struct mqtt_publish_param param;
    param.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
    param.message.topic.topic.utf8 = (uint8_t*)topic;
    param.message.topic.topic.size = strlen(topic);
    param.message.payload.data = (uint8_t*)payload;
    param.message.payload.len = strlen(payload);
    param.message_id = sys_rand32_get();
    param.dup_flag = 0U;
    param.retain_flag = retain ? 1U : 0U;

    int ret = mqtt_publish(&client_ctx, &param);
    if (ret) {
        LOG_ERR("MQTT publish failed: %d", ret);
    }
}

static unsigned long strntoul(const char* p, size_t len, const char** endp) {
    unsigned long x = 0;
    const char* end = p + len;
    while (p < end && *p == ' ') p++;
    while (p < end) {
        char c = *p++;
        if (!isdigit(c)) break;
        x *= 10;
        x += c - '0';
    }
    *endp = p;
    return x;
}

static void mqtt_evt_handler(struct mqtt_client* const client,
                             const struct mqtt_evt* evt) {
    switch (evt->type) {
        case MQTT_EVT_CONNACK:
            if (evt->result != 0) {
                LOG_ERR("MQTT connect failed %d", evt->result);
                break;
            }
            connected = true;
            LOG_INF("MQTT client connected!");
            break;

        case MQTT_EVT_DISCONNECT:
            LOG_INF("MQTT client disconnected %d", evt->result);
            connected = false;
            clear_fds();
            break;

        case MQTT_EVT_PUBLISH: {
            const struct mqtt_publish_param* pub = &evt->param.publish;
            if (pub->message.topic.topic.size == strlen(cmd_topic) &&
                strncmp(pub->message.topic.topic.utf8, cmd_topic,
                        strlen(cmd_topic)) == 0) {
                char payload[16] = {0};
                size_t len = pub->message.payload.len;
                if (len >= sizeof(payload)) {
                    len = sizeof(payload) - 1;
                }
                // Read payload from stream
                if (mqtt_read_publish_payload(client, payload, len) >= 0) {
                    const char* endptr;
                    long val = strntoul(payload, len, &endptr);
                    if (endptr != payload) {
                        door_cmd_t cmd;
                        if (val == 0)
                            cmd = DOOR_CMD_CLOSE;
                        else if (val == 1)
                            cmd = DOOR_CMD_OPEN;
                        else
                            break;
                        ctrl_msg_t msg = {CTRL_MSG_DOOR_CONTROL,
                                          {.door_control = {cmd}}};
                        k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
                    }
                }
            } else {
                // Discard payload
                mqtt_read_publish_payload(client, NULL,
                                          pub->message.payload.len);
            }
            break;
        }

        default:
            break;
    }
}

static int subscribe(void) {
    struct mqtt_topic topic = {
        .topic = {.utf8 = (uint8_t*)cmd_topic, .size = strlen(cmd_topic)},
        .qos = MQTT_QOS_0_AT_MOST_ONCE};
    const struct mqtt_subscription_list sub = {
        .list = &topic, .list_count = 1, .message_id = sys_rand32_get()};
    return mqtt_subscribe(&client_ctx, &sub);
}

static int broker_init(const MqttConfig* cfg) {
    struct zsock_addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct zsock_addrinfo* res;

    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%u", cfg->broker_port);

    int ret = zsock_getaddrinfo(cfg->broker_host, port_str, &hints, &res);
    if (ret != 0) {
        LOG_ERR("Failed to resolve broker %s: %d", cfg->broker_host, ret);
        return -1;
    }

    memcpy(&broker, res->ai_addr, res->ai_addrlen);
    zsock_freeaddrinfo(res);
    return 0;
}

static int client_init(const MqttConfig* cfg) {
    mqtt_client_init(&client_ctx);
    if (broker_init(cfg) != 0) {
        return -1;
    }

    client_ctx.broker = &broker;
    client_ctx.evt_cb = mqtt_evt_handler;
    client_ctx.client_id.utf8 =
        (uint8_t*)(cfg->client_id[0] ? cfg->client_id : MQTT_CLIENTID);
    client_ctx.client_id.size = strlen(client_ctx.client_id.utf8);
    static struct mqtt_utf8 mqtt_username;
    static struct mqtt_utf8 mqtt_password;

    if (cfg->password[0]) {
        mqtt_password.utf8 = (uint8_t*)cfg->password;
        mqtt_password.size = strlen(cfg->password);
        client_ctx.password = &mqtt_password;
    } else {
        client_ctx.password = NULL;
    }

    if (cfg->username[0]) {
        mqtt_username.utf8 = (uint8_t*)cfg->username;
        mqtt_username.size = strlen(cfg->username);
        client_ctx.user_name = &mqtt_username;
    } else {
        client_ctx.user_name = NULL;
    }

    client_ctx.protocol_version = MQTT_VERSION_3_1_1;

    client_ctx.rx_buf = rx_buffer;
    client_ctx.rx_buf_size = sizeof(rx_buffer);
    client_ctx.tx_buf = tx_buffer;
    client_ctx.tx_buf_size = sizeof(tx_buffer);
    client_ctx.transport.type = MQTT_TRANSPORT_NON_SECURE;

    // Will config
    static struct mqtt_topic will_topic = {
        .topic = {.utf8 = (uint8_t*)"dummy",
                  .size = 0},  // Set dynamically below
        .qos = MQTT_QOS_0_AT_MOST_ONCE};
    will_topic.topic.utf8 = (uint8_t*)lwt_topic;
    will_topic.topic.size = strlen(lwt_topic);

    static struct mqtt_utf8 will_msg = {.utf8 = (uint8_t*)LWT_OFFLINE,
                                        .size = sizeof(LWT_OFFLINE) - 1};

    client_ctx.will_topic = &will_topic;
    client_ctx.will_message = &will_msg;
    client_ctx.will_retain = 1;
    return 0;
}

void mqtt_task(void* p1, void* p2, void* p3) {
    LOG_INF("MQTT task started, has_config=%d", app_config.has_mqtt_config);
    if (app_config.has_mqtt_config) {
        LOG_INF("enabled=%d, broker=%s", app_config.mqtt_config.enabled,
                app_config.mqtt_config.broker_host);
    }
    const MqttConfig* cfg = &app_config.mqtt_config;

    while (1) {
        if (!app_config.has_mqtt_config || !app_config.mqtt_config.enabled ||
            !app_config.mqtt_config.broker_host[0]) {
            k_msleep(5000);
            continue;
        }

        const char* prefix = cfg->prefix[0] ? cfg->prefix : "sesame";
        snprintf(state_topic, sizeof(state_topic), "%s/state", prefix);
        snprintf(lwt_topic, sizeof(lwt_topic), "%s/availability", prefix);
        snprintf(cmd_topic, sizeof(cmd_topic), "%s/cmd", prefix);

        if (client_init(cfg) != 0) {
            k_msleep(5000);
            continue;
        }

        LOG_INF("Connecting to MQTT broker %s:%d", cfg->broker_host,
                cfg->broker_port);
        int rc = mqtt_connect(&client_ctx);
        if (rc != 0) {
            LOG_ERR("mqtt_connect failed: %d", rc);
            k_msleep(5000);
            continue;
        }

        prepare_fds(&client_ctx);

        while (!connected) {
            if (wait(5000) > 0) {
                mqtt_input(&client_ctx);
            } else {
                break;
            }
        }

        if (!connected) {
            mqtt_abort(&client_ctx);
            k_msleep(5000);
            continue;
        }

        subscribe();
        publish(lwt_topic, LWT_ONLINE, true);

        while (connected) {
            if (wait(5000) > 0) {
                rc = mqtt_input(&client_ctx);
                if (rc != 0) {
                    LOG_ERR("mqtt_input error: %d", rc);
                    break;
                }
            }

            rc = mqtt_live(&client_ctx);
            if (rc != 0 && rc != -EAGAIN) {
                LOG_ERR("mqtt_live error: %d", rc);
                break;
            }
        }

        LOG_WRN("MQTT disconnected, reconnecting...");
        mqtt_abort(&client_ctx);
        connected = false;
        k_msleep(5000);
    }
}

void publish_state(const door_state_msg_t* msg) {
    char* state;
    switch (msg->state) {
        case DCM_DOOR_STATE_CLOSED:
            state = "CLOSED";
            break;
        case DCM_DOOR_STATE_OPEN:
            state = "OPEN";
            break;
        default:
            state = "UNDEF";
    }
    char* dir;
    switch (msg->direction) {
        case DCM_DOOR_DIR_UP:
            dir = "up";
            break;
        case DCM_DOOR_DIR_DOWN:
            dir = "down";
            break;
        case DCM_DOOR_DIR_STOPPED:
            dir = "stopped";
            break;
        default:
            dir = "UNDEF";
    }

    static const char fmt[] =
        "{\"contact\":\"%s\",\"dir\":\"%s\",\"pos\":%d,\"uptime\":\"%uT%s\","
        "\"uptime_sec\":%u,\"heap_free_bytes\":%u}";
    unsigned uptime_s = k_uptime_get_32() / 1000;
    unsigned days = uptime_s / SECONDS_PER_DAY;
    time_t time_ms = uptime_s % SECONDS_PER_DAY;
    char tm_hms[9] = {0};
    struct tm tm;
    if (gmtime_r(&time_ms, &tm)) {
        snprintf(tm_hms, sizeof(tm_hms), "%02d:%02d:%02d", tm.tm_hour,
                 tm.tm_min, tm.tm_sec);
    }
    static char payload[128];
    struct sys_memory_stats stats;
    size_t free_heap = 0;
    if (sys_heap_runtime_stats_get(&_system_heap.heap, &stats) == 0) {
        free_heap = stats.free_bytes;
    }

    snprintf(payload, sizeof(payload), fmt, state, dir, msg->pos, days, tm_hms,
             uptime_s, (unsigned int)free_heap);
    LOG_INF("Publishing to %s: %s", state_topic, payload);
    publish(state_topic, payload, false);
}

K_THREAD_DEFINE(mqtt_tid, 2048, mqtt_task, NULL, NULL, NULL, 5, 0, 0);

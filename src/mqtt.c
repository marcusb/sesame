/*
 * Lab-Project-coreMQTT-Agent 201215
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 */

#include "mqtt.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "core_mqtt.h"
#include "core_mqtt_agent.h"
#include "core_mqtt_agent_message_interface.h"
#include "freertos_agent_message.h"
#include "queue.h"
#include "task.h"
#include "transport_plaintext.h"

// Application
#include "app_config.pb.h"
#include "app_logging.h"
#include "backoff_algorithm.h"
#include "controller.h"

static char state_topic[64];
static char lwt_topic[64];
static char cmd_topic[64];

static const char* LWT_ONLINE = "Online";
static const char* LWT_OFFLINE = "Offline";

struct NetworkContext {
    PlaintextTransportParams_t* pParams;
};

struct MQTTAgentCommandContext {
    MQTTStatus_t status;
    TaskHandle_t notify_task;
    uint32_t notify_val;
};

static PlaintextTransportParams_t transport_params;
static NetworkContext_t network_context = {&transport_params};
static TransportInterface_t transport = {
    Plaintext_FreeRTOS_recv, Plaintext_FreeRTOS_send, NULL, &network_context};

static uint32_t next_msg_id;
static bool mqtt_initialized;

/**
 * @brief Dimensions the buffer used to serialize and deserialize MQTT packets.
 * @note Specified in bytes.  Must be large enough to hold the maximum
 * anticipated MQTT payload.
 */
#define MQTT_AGENT_NETWORK_BUFFER_SIZE (512)

/**
 * @brief Size of statically allocated buffers for holding topic names and
 * payloads.
 */
#define MSG_BUF_LEN (64)

/**
 * @brief Timeout for receiving CONNACK after sending an MQTT CONNECT packet.
 * Defined in milliseconds.
 */
#define CONNACK_RECV_TIMEOUT_MS (2000U)

/**
 * @brief The maximum amount of time in milliseconds to wait for the commands
 * to be posted to the MQTT agent should the MQTT agent's command queue be full.
 * Tasks wait in the Blocked state, so don't use any CPU time.
 */
#define MAX_COMMAND_SEND_BLOCK_TIME_MS (500)

/*
 * @brief The time to wait for a notification callback
 */
#define NOTIFICATION_WAIT_MS (20000)

/**
 * @brief The maximum number of retries for network operation with server.
 */
#define RETRY_MAX_ATTEMPTS (5U)

/**
 * @brief The maximum back-off delay (in milliseconds) for retrying failed
 * operation with server.
 */
#define RETRY_MAX_BACKOFF_DELAY_MS (5000U)

/**
 * @brief The base back-off delay (in milliseconds) to use for network
 * operation retry attempts.
 */
#define RETRY_BACKOFF_BASE_MS (500U)

/**
 * @brief The maximum time interval in seconds which is allowed to elapse
 *  between two Control Packets.
 *
 *  It is the responsibility of the Client to ensure that the interval between
 *  Control Packets being sent does not exceed the this Keep Alive value. In
 * the absence of sending any other Control Packets, the Client MUST send a
 *  PINGREQ Packet.
 */
#define KEEP_ALIVE_INTERVAL_SECONDS (60U)

/**
 * @brief Socket send and receive timeouts to use.  Specified in milliseconds.
 */
#define TRANSPORT_SEND_RECV_TIMEOUT_MS (750)

#define SECONDS_PER_DAY (86400)

static MQTTAgentContext_t mqtt_agent_context;

static uint8_t xNetworkBuffer[MQTT_AGENT_NETWORK_BUFFER_SIZE];

static MQTTAgentMessageContext_t xCommandQueue;

/**
 * @brief The timer query function provided to the MQTT context.
 *
 * @return Time in milliseconds.
 */
static uint32_t get_time_millis(void) {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

static unsigned long strntoul(const char* p, size_t len, const char** endp) {
    unsigned long x = 0;
    const char* end = p + len;
    while (p < end && *p == ' ') {
        p++;
    }
    while (p < end) {
        char c = *p++;
        if (!isdigit(c)) {
            break;
        }
        x += c - '0';
        x *= 10;
    }
    *endp = end;
    return x;
}

static void incoming_pub_cb(MQTTAgentContext_t* ctx, uint16_t packet_id,
                            MQTTPublishInfo_t* pub_info) {
    bool matched;
    MQTT_MatchTopic(pub_info->pTopicName, pub_info->topicNameLength, cmd_topic,
                    strlen(cmd_topic), &matched);
    if (matched) {
        LogDebug(("Received incoming publish message %.*s",
                  pub_info->topicNameLength, pub_info->pTopicName));
        const char* endptr;
        long val =
            strntoul(pub_info->pPayload, pub_info->payloadLength, &endptr);
        if (endptr != pub_info->pPayload) {
            door_cmd_t cmd;
            if (val == 0) {
                cmd = DOOR_CMD_CLOSE;
            } else if (val == 1) {
                cmd = DOOR_CMD_OPEN;
            } else {
                return;
            }
            ctrl_msg_t msg = {CTRL_MSG_DOOR_CONTROL, {.door_control = {cmd}}};
            xQueueSendToBack(ctrl_queue, &msg, 100);
        }
    } else {
        /* Ensure the topic string is terminated for printing.  This will
         * over- write the message ID, which is restored afterwards. */
        char* p = (char*)&(pub_info->pTopicName[pub_info->topicNameLength]);
        char c = *p;
        *p = 0x00;
        LogWarn(("Received an unsolicited publish from topic %s",
                 pub_info->pTopicName));
        *p = c;
    }
}

static MQTTAgentCommand_t* agent_get_cmd(uint32_t block_time_ms) {
    return pvPortMalloc(sizeof(MQTTAgentCommand_t));
}

static bool agent_release_cmd(MQTTAgentCommand_t* cmd) {
    vPortFree(cmd);
    return true;
}

/**
 * @brief Initializes an MQTT context, including transport interface and
 * network buffer.
 *
 * @return `MQTTSuccess` if the initialization succeeds, else
 * `MQTTBadParameter`.
 */
static MQTTStatus_t mqtt_init(void) {
    MQTTStatus_t xReturn;
    MQTTFixedBuffer_t xFixedBuffer = {.pBuffer = xNetworkBuffer,
                                      .size = MQTT_AGENT_NETWORK_BUFFER_SIZE};
    MQTTAgentMessageInterface_t messageInterface = {
        .pMsgCtx = NULL,
        .send = (MQTTAgentMessageSend_t)Agent_MessageSend,
        .recv = (MQTTAgentMessageRecv_t)Agent_MessageReceive,
        .getCommand = agent_get_cmd,
        .releaseCommand = agent_release_cmd};

    xCommandQueue.queue = xQueueCreate(MQTT_AGENT_COMMAND_QUEUE_LENGTH,
                                       sizeof(MQTTAgentCommand_t*));
    configASSERT(xCommandQueue.queue);
    messageInterface.pMsgCtx = &xCommandQueue;

    /* Initialize MQTT library. */
    xReturn =
        MQTTAgent_Init(&mqtt_agent_context, &messageInterface, &xFixedBuffer,
                       &transport, get_time_millis, incoming_pub_cb, NULL);

    return xReturn;
}

static void subscribe_cb(MQTTAgentCommandContext_t* ctx,
                         MQTTAgentReturnInfo_t* return_info) {
    ctx->status = return_info->returnCode;
    xTaskNotify(ctx->notify_task, (uint32_t)(return_info->returnCode),
                eSetValueWithOverwrite);
}

static bool subscribe(MQTTQoS_t qos, char* topic_filter) {
    uint32_t msg_id;

    xTaskNotifyStateClear(NULL);
    taskENTER_CRITICAL();
    {
        msg_id = ++next_msg_id;
    }
    taskEXIT_CRITICAL();

    /* Complete the subscribe information.  The topic string must persist
     * for duration of subscription! */
    MQTTSubscribeInfo_t subscribe_info = {qos, topic_filter,
                                          strlen(topic_filter)};
    MQTTAgentSubscribeArgs_t args = {&subscribe_info, 1};

    /* Complete an application defined context associated with this
     * subscribe message. This gets updated in the callback function so the
     * variable must persist until the callback executes. */
    MQTTAgentCommandContext_t ctx = {0, xTaskGetCurrentTaskHandle(), msg_id};
    MQTTAgentCommandInfo_t params = {subscribe_cb, (void*)&ctx,
                                     MAX_COMMAND_SEND_BLOCK_TIME_MS};

    /* Loop in case the queue used to communicate with the MQTT agent is
     * full and attempts to post to it time out.  The queue will not
     * become full if the priority of the MQTT agent task is higher than
     * the priority of the task calling this function. */
    LogInfo(
        ("Sending subscribe request to agent for topic filter: "
         "%s with id %d",
         topic_filter, msg_id));

    MQTTStatus_t res;
    do {
        res = MQTTAgent_Subscribe(&mqtt_agent_context, &args, &params);
    } while (res != MQTTSuccess);

    /* Wait for acks to the subscribe message - this is optional but done
     * here so the code below can check the notification sent by the
     * callback matches the msg_id value set in the context above. */
    BaseType_t acked =
        xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(NOTIFICATION_WAIT_MS));

    /* Check both ways the status was passed back just for demonstration
     * purposes. */
    if ((acked != pdTRUE) || (ctx.status != MQTTSuccess)) {
        LogInfo(
            ("Error or timed out waiting for ack to subscribe message topic "
             "%s: %d %d",
             topic_filter, acked, ctx.status));
    } else {
        LogInfo(("Received subscribe ack for topic %s containing ID %d",
                 topic_filter, (int)ctx.notify_val));
    }

    return acked;
}

static void subscribe_topics() { subscribe(MQTTQoS0, cmd_topic); }

/**
 * @brief Sends an MQTT Connect packet over the already connected TCP
 * socket.
 *
 * @param[in] clean_session If a clean session should be established.
 *
 * @return `MQTTSuccess` if connection succeeds, else appropriate error code
 * from MQTT_Connect.
 */
static MQTTStatus_t mqtt_connect(const MqttConfig* cfg, bool clean_session) {
    MQTTStatus_t res;
    MQTTConnectInfo_t connect_info = {
        clean_session,  KEEP_ALIVE_INTERVAL_SECONDS,
        cfg->client_id, strlen(cfg->client_id),
        cfg->username,  strlen(cfg->username),
        cfg->password,  strlen(cfg->password)};
    MQTTPublishInfo_t last_will = {MQTTQoS0,
                                   false,
                                   false,
                                   lwt_topic,
                                   strlen(lwt_topic),
                                   LWT_OFFLINE,
                                   strlen(LWT_OFFLINE)};

    bool session_present = false;
    res = MQTT_Connect(&(mqtt_agent_context.mqttContext), &connect_info,
                       &last_will, CONNACK_RECV_TIMEOUT_MS, &session_present);

    LogInfo(("Session present: %d", session_present));

    /* Resume a session if desired. */
    if (res == MQTTSuccess && !clean_session) {
        res = MQTTAgent_ResumeSession(&mqtt_agent_context, session_present);

        if ((res == MQTTSuccess) && (session_present == false)) {
            LogInfo(("resubscribing to topics"));
            subscribe_topics();
        }
    }

    return res;
}

/**
 * @brief Callback executed when there is activity on the TCP socket that is
 * connected to the MQTT broker.  If there are no messages in the MQTT
 * agent's command queue then the callback send a message to ensure the MQTT
 * agent task unblocks and can therefore process whatever is necessary on
 * the socket (if anything) as quickly as possible.
 *
 * @param[in] pxSocket Socket with data, unused.
 */
static void prvMQTTClientSocketWakeupCallback(Socket_t pxSocket) {
    MQTTAgentCommandInfo_t xCommandParams = {0};

    /* Just to avoid compiler warnings.  The socket is not used but the
     * function prototype cannot be changed because this is a callback
     * function. */
    (void)pxSocket;

    /* A socket used by the MQTT task may need attention.  Send an event
     * to the MQTT task to make sure the task is not blocked on
     * xCommandQueue.
     */
    if ((uxQueueMessagesWaiting(xCommandQueue.queue) == 0U) &&
        (FreeRTOS_recvcount(pxSocket) > 0)) {
        /* Don't block as this is called from the context of the IP task. */
        xCommandParams.blockTimeMs = 0U;
        MQTTAgent_ProcessLoop(&mqtt_agent_context, &xCommandParams);
    }
}

/**
 * @brief Connect a TCP socket to the MQTT broker.
 *
 * @param[in] pxNetworkContext Network context.
 *
 * @return `pdPASS` if connection succeeds, else `pdFAIL`.
 */
static BaseType_t socket_connect(const char* host, uint16_t port,
                                 NetworkContext_t* pxNetworkContext) {
    BaseType_t xConnected = pdFAIL;
    BackoffAlgorithmStatus_t backoff_status = BackoffAlgorithmSuccess;
    BackoffAlgorithmContext_t reconnect_params = {0};
    const TickType_t xTransportTimeout = 0UL;

    PlaintextTransportStatus_t xNetworkStatus =
        PLAINTEXT_TRANSPORT_CONNECT_FAILURE;

    BackoffAlgorithm_InitializeParams(&reconnect_params, RETRY_BACKOFF_BASE_MS,
                                      RETRY_MAX_BACKOFF_DELAY_MS,
                                      RETRY_MAX_ATTEMPTS);

    do {
        LogInfo(("connecting to %s:%d", host, port));
        xNetworkStatus = Plaintext_FreeRTOS_Connect(
            pxNetworkContext, host, port, TRANSPORT_SEND_RECV_TIMEOUT_MS,
            TRANSPORT_SEND_RECV_TIMEOUT_MS);
        xConnected =
            (xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS) ? pdPASS : pdFAIL;

        if (!xConnected) {
            uint16_t backoff;
            backoff_status = BackoffAlgorithm_GetNextBackoff(&reconnect_params,
                                                             rand(), &backoff);

            if (backoff_status == BackoffAlgorithmSuccess) {
                LogWarn(
                    ("Connection to the broker failed. "
                     "Retrying connection in %hu ms.",
                     backoff));
                vTaskDelay(pdMS_TO_TICKS(backoff));
            }
        }

        if (backoff_status == BackoffAlgorithmRetriesExhausted) {
            LogError(
                ("Connection to the broker failed, all attempts exhausted."));
        }
    } while ((xConnected != pdPASS) &&
             (backoff_status == BackoffAlgorithmSuccess));

    /* Set the socket wakeup callback and ensure the read block time. */
    if (xConnected) {
        (void)FreeRTOS_setsockopt(pxNetworkContext->pParams->tcpSocket,
                                  0, /* Level - Unused. */
                                  FREERTOS_SO_WAKEUP_CALLBACK,
                                  (void*)prvMQTTClientSocketWakeupCallback,
                                  sizeof(&(prvMQTTClientSocketWakeupCallback)));

        (void)FreeRTOS_setsockopt(pxNetworkContext->pParams->tcpSocket, 0,
                                  FREERTOS_SO_RCVTIMEO, &xTransportTimeout,
                                  sizeof(TickType_t));
    }

    return xConnected;
}

/**
 * @brief Disconnect a TCP connection.
 *
 * @param[in] pxNetworkContext Network context.
 *
 * @return `pdPASS` if disconnect succeeds, else `pdFAIL`.
 */
static BaseType_t socket_disconnect(NetworkContext_t* pxNetworkContext) {
    BaseType_t xDisconnected = pdFAIL;

    /* Set the wakeup callback to NULL since the socket will disconnect. */
    (void)FreeRTOS_setsockopt(
        pxNetworkContext->pParams->tcpSocket, 0, /* Level - Unused. */
        FREERTOS_SO_WAKEUP_CALLBACK, (void*)NULL, sizeof(void*));

    LogInfo(("Disconnecting TCP connection"));
    PlaintextTransportStatus_t xNetworkStatus =
        PLAINTEXT_TRANSPORT_CONNECT_FAILURE;
    xNetworkStatus = Plaintext_FreeRTOS_Disconnect(pxNetworkContext);
    xDisconnected =
        (xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS) ? pdPASS : pdFAIL;
    return xDisconnected;
}

/**
 * @brief Connects a TCP socket to the MQTT broker, then creates and MQTT
 * connection to the same.
 */
static void connect_broker(const MqttConfig* cfg) {
    BaseType_t xNetworkStatus = pdFAIL;
    MQTTStatus_t xMQTTStatus;

    /* Connect a TCP socket to the broker. */
    xNetworkStatus =
        socket_connect(cfg->broker_host, cfg->broker_port, &network_context);
    configASSERT(xNetworkStatus == pdPASS);

    /* Initialize the MQTT context with the buffer and transport interface.
     */
    xMQTTStatus = mqtt_init();
    configASSERT(xMQTTStatus == MQTTSuccess);

    /* Form an MQTT connection without a persistent session. */
    xMQTTStatus = mqtt_connect(cfg, true);
    configASSERT(xMQTTStatus == MQTTSuccess);
}

static void agent_task(void* params) {
    const MqttConfig* cfg = (const MqttConfig*)params;
    MQTTContext_t* pMqttContext = &(mqtt_agent_context.mqttContext);

    MQTTStatus_t status;
    do {
        /* MQTTAgent_CommandLoop() is effectively the agent implementation.
         * It will manage the MQTT protocol until such time that an error
         * occurs, which could be a disconnect.  If an error occurs the MQTT
         * context on which the error happened is returned so there can be
         * an attempt to clean up and reconnect however the application
         * writer prefers. */
        status = MQTTAgent_CommandLoop(&mqtt_agent_context);
        BaseType_t res = pdFAIL;

        /* Success is returned for disconnect or termination. The socket
         * should be disconnected. */
        if (status == MQTTSuccess) {
            /* MQTT Disconnect. Disconnect the socket. */
            res = socket_disconnect(&network_context);
        }
        /* Error. */
        else {
            /* Reconnect TCP. */
            res = socket_disconnect(&network_context);
            configASSERT(res == pdPASS);
            res = socket_connect(cfg->broker_host, cfg->broker_port,
                                 &network_context);
            configASSERT(res == pdPASS);
            pMqttContext->connectStatus = MQTTNotConnected;
            /* MQTT Connect with a persistent session. */
            MQTTStatus_t xConnectStatus = mqtt_connect(cfg, false);
            configASSERT(xConnectStatus == MQTTSuccess);
        }
    } while (status != MQTTSuccess);
}

static void publish(const char* topic, const char* payload, bool retain) {
    if (!mqtt_initialized) {
        LogInfo(("MQTT not initialized, skipping pub"));
        return;
    }
    MQTTAgentCommandInfo_t command_params = {NULL, NULL,
                                             MAX_COMMAND_SEND_BLOCK_TIME_MS};
    MQTTPublishInfo_t publish_info = {MQTTQoS0,       retain,        false,
                                      topic,          strlen(topic), payload,
                                      strlen(payload)};

    MQTTAgent_Publish(&mqtt_agent_context, &publish_info, &command_params);
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
        "{\"contact\":\"%s\",\"dir\":\"%s\",\"pos\":%d,\"uptime\":\"%ldT%s\","
        "\"uptime_sec\":%ld,\"heap_free_bytes\":%u}";
    long uptime_s = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    long days = uptime_s / SECONDS_PER_DAY;
    time_t time_ms = uptime_s % SECONDS_PER_DAY;
    char tm_hms[9] = {0};
    struct tm tm;
    if (gmtime_r(&time_ms, &tm) &&
        strftime(tm_hms, sizeof(tm_hms), "%H:%M:%S", &tm) == 0) {
        *tm_hms = '\0';
    }
    static char payload[128];
    snprintf(payload, sizeof(payload), fmt, state, dir, msg->pos, days, tm_hms,
             uptime_s, xPortGetFreeHeapSize());
    publish(state_topic, payload, false);
}

void mqtt_task(void* params) {
    const MqttConfig* cfg = (const MqttConfig*)params;
    if (!cfg->enabled) {
        goto ret;
    }

    const char* prefix = *cfg->prefix ? cfg->prefix : "sesame";
    snprintf(state_topic, sizeof(state_topic), "%s/state", prefix);
    snprintf(lwt_topic, sizeof(lwt_topic), "%s/availability", prefix);
    snprintf(cmd_topic, sizeof(cmd_topic), "%s/cmd", prefix);

    connect_broker(cfg);
    xTaskCreate(agent_task, "MQTT-Agent", 512, NULL, tskIDLE_PRIORITY + 4,
                NULL);
    mqtt_initialized = true;
    publish(lwt_topic, LWT_ONLINE, true);
    subscribe_topics();

ret:
    vTaskDelete(NULL);
}

#include "mqtt.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "app_logging.h"
#include "backoff_algorithm.h"
#include "core_mqtt.h"
#include "core_mqtt_agent.h"
#include "core_mqtt_agent_message_interface.h"
#include "freertos_agent_message.h"
#include "freertos_command_pool.h"
#include "queue.h"
#include "subscription_manager.h"
#include "task.h"
#include "transport_plaintext.h"

#define democonfigMQTT_BROKER_ENDPOINT "172.16.1.25"
#define democonfigMQTT_BROKER_PORT 1883
#define democonfigCLIENT_IDENTIFIER "sesame"
#define democonfigCLIENT_USERNAME "sesame"
#define democonfigCLIENT_PASSWORD "***REMOVED***"

struct NetworkContext {
    PlaintextTransportParams_t* pParams;
};

struct MQTTAgentCommandContext {
    MQTTStatus_t status;
    TaskHandle_t notify_task;
    uint32_t notify_val;
    void* pArgs;
};

static PlaintextTransportParams_t transport_params;
static NetworkContext_t network_context = {&transport_params};
static TransportInterface_t transport = {
    Plaintext_FreeRTOS_recv, Plaintext_FreeRTOS_send, NULL, &network_context};

/**
 * @brief Dimensions the buffer used to serialize and deserialize MQTT packets.
 * @note Specified in bytes.  Must be large enough to hold the maximum
 * anticipated MQTT payload.
 */
#define MQTT_AGENT_NETWORK_BUFFER_SIZE (2048)

/**
 * @brief Timeout for receiving CONNACK after sending an MQTT CONNECT packet.
 * Defined in milliseconds.
 */
#define CONNACK_RECV_TIMEOUT_MS (2000U)

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
 * @brief The base back-off delay (in milliseconds) to use for network operation
 * retry attempts.
 */
#define RETRY_BACKOFF_BASE_MS (500U)

/**
 * @brief The maximum time interval in seconds which is allowed to elapse
 *  between two Control Packets.
 *
 *  It is the responsibility of the Client to ensure that the interval between
 *  Control Packets being sent does not exceed the this Keep Alive value. In the
 *  absence of sending any other Control Packets, the Client MUST send a
 *  PINGREQ Packet.
 */
#define KEEP_ALIVE_INTERVAL_SECONDS (60U)

/**
 * @brief Socket send and receive timeouts to use.  Specified in milliseconds.
 */
#define TRANSPORT_SEND_RECV_TIMEOUT_MS (750)

/**
 * @brief Global entry time into the application to use as a reference timestamp
 * in the #prvGetTimeMs function. #prvGetTimeMs will always return the
 * difference between the current time and the global entry time. This will
 * reduce the chances of overflow for the 32 bit unsigned integer used for
 * holding the timestamp.
 */
static uint32_t elapsed;

MQTTAgentContext_t mqtt_agent_context;

static uint8_t xNetworkBuffer[MQTT_AGENT_NETWORK_BUFFER_SIZE];

static MQTTAgentMessageContext_t xCommandQueue;

/**
 * @brief The global array of subscription elements.
 *
 * @note No thread safety is required to this array, since the updates the array
 * elements are done only from one task at a time. The subscription manager
 * implementation expects that the array of the subscription elements used for
 * storing subscriptions to be initialized to 0. As this is a global array, it
 * will be initialized to 0 by default.
 */
SubscriptionElement_t
    xGlobalSubscriptionList[SUBSCRIPTION_MANAGER_MAX_SUBSCRIPTIONS];

/**
 * @brief The timer query function provided to the MQTT context.
 *
 * @return Time in milliseconds.
 */
static uint32_t prvGetTimeMs(void) {
    TickType_t xTickCount = 0;
    uint32_t ulTimeMs = 0UL;

    /* Get the current tick count. */
    xTickCount = xTaskGetTickCount();

    /* Convert the ticks to milliseconds. */
    ulTimeMs = pdTICKS_TO_MS(xTickCount);

    /* Reduce ulGlobalEntryTimeMs from obtained time so as to always return the
     * elapsed time in the application. */
    ulTimeMs = (uint32_t)(ulTimeMs - elapsed);

    return ulTimeMs;
}

/**
 * @brief Fan out the incoming publishes to the callbacks registered by
 * different tasks. If there are no callbacks registered for the incoming
 * publish, it will be passed to the unsolicited publish handler.
 *
 * @param[in] pMqttAgentContext Agent context.
 * @param[in] packetId Packet ID of publish.
 * @param[in] pxPublishInfo Info of incoming publish.
 */
static void prvIncomingPublishCallback(MQTTAgentContext_t* pMqttAgentContext,
                                       uint16_t packetId,
                                       MQTTPublishInfo_t* pxPublishInfo) {
    bool xPublishHandled = false;
    char cOriginalChar, *pcLocation;

    (void)packetId;

    /* Fan out the incoming publishes to the callbacks registered using
     * subscription manager. */
    xPublishHandled = handleIncomingPublishes(
        (SubscriptionElement_t*)pMqttAgentContext->pIncomingCallbackContext,
        pxPublishInfo);

    /* If there are no callbacks to handle the incoming publishes,
     * handle it as an unsolicited publish. */
    if (xPublishHandled != true) {
        /* Ensure the topic string is terminated for printing.  This will over-
         * write the message ID, which is restored afterwards. */
        pcLocation =
            (char*)&(pxPublishInfo->pTopicName[pxPublishInfo->topicNameLength]);
        cOriginalChar = *pcLocation;
        *pcLocation = 0x00;
        LogWarn(("WARN:  Received an unsolicited publish from topic %s",
                 pxPublishInfo->pTopicName));
        *pcLocation = cOriginalChar;
    }
}

/**
 * @brief Initializes an MQTT context, including transport interface and
 * network buffer.
 *
 * @return `MQTTSuccess` if the initialization succeeds, else
 * `MQTTBadParameter`.
 */
static MQTTStatus_t prvMQTTInit(void) {
    MQTTStatus_t xReturn;
    MQTTFixedBuffer_t xFixedBuffer = {.pBuffer = xNetworkBuffer,
                                      .size = MQTT_AGENT_NETWORK_BUFFER_SIZE};
    static uint8_t staticQueueStorageArea[MQTT_AGENT_COMMAND_QUEUE_LENGTH *
                                          sizeof(MQTTAgentCommand_t*)];
    static StaticQueue_t staticQueueStructure;
    MQTTAgentMessageInterface_t messageInterface = {
        .pMsgCtx = NULL,
        .send = Agent_MessageSend,
        .recv = Agent_MessageReceive,
        .getCommand = Agent_GetCommand,
        .releaseCommand = Agent_ReleaseCommand};

    xCommandQueue.queue = xQueueCreateStatic(
        MQTT_AGENT_COMMAND_QUEUE_LENGTH, sizeof(MQTTAgentCommand_t*),
        staticQueueStorageArea, &staticQueueStructure);
    configASSERT(xCommandQueue.queue);
    messageInterface.pMsgCtx = &xCommandQueue;

    /* Initialize the task pool. */
    Agent_InitializePool();

    /* Initialize MQTT library. */
    xReturn =
        MQTTAgent_Init(&mqtt_agent_context, &messageInterface, &xFixedBuffer,
                       &transport, prvGetTimeMs, prvIncomingPublishCallback,
                       /* Context to pass into the callback. Passing the
                          pointer to subscription array. */
                       xGlobalSubscriptionList);

    return xReturn;
}

/**
 * @brief Passed into MQTTAgent_Subscribe() as the callback to execute when the
 * broker ACKs the SUBSCRIBE message. This callback implementation is used for
 * handling the completion of resubscribes. Any topic filter failed to
 * resubscribe will be removed from the subscription list.
 *
 * See https://freertos.org/mqtt/mqtt-agent-demo.html#example_mqtt_api_call
 *
 * @param[in] pxCommandContext Context of the initial command.
 * @param[in] pxReturnInfo The result of the command.
 */
static void prvSubscriptionCommandCallback(
    MQTTAgentCommandContext_t* pxCommandContext,
    MQTTAgentReturnInfo_t* pxReturnInfo) {
    size_t lIndex = 0;
    MQTTAgentSubscribeArgs_t* pxSubscribeArgs =
        (MQTTAgentSubscribeArgs_t*)pxCommandContext;

    /* If the return code is success, no further action is required as all the
     * topic filters are already part of the subscription list. */
    if (pxReturnInfo->returnCode != MQTTSuccess) {
        /* Check through each of the suback codes and determine if there are any
         * failures. */
        for (lIndex = 0; lIndex < pxSubscribeArgs->numSubscriptions; lIndex++) {
            /* This demo doesn't attempt to resubscribe in the event that a
             * SUBACK failed. */
            if (pxReturnInfo->pSubackCodes[lIndex] == MQTTSubAckFailure) {
                LogError(
                    ("Failed to resubscribe to topic %.*s.",
                     pxSubscribeArgs->pSubscribeInfo[lIndex].topicFilterLength,
                     pxSubscribeArgs->pSubscribeInfo[lIndex].pTopicFilter));
                /* Remove subscription callback for unsubscribe. */
                removeSubscription(
                    xGlobalSubscriptionList,
                    pxSubscribeArgs->pSubscribeInfo[lIndex].pTopicFilter,
                    pxSubscribeArgs->pSubscribeInfo[lIndex].topicFilterLength);
            }
        }

        /* Hit an assert as some of the tasks won't be able to proceed correctly
         * without the subscriptions. This logic will be updated with
         * exponential backoff and retry.  */
        configASSERT(pdTRUE);
    }
}

/**
 * @brief Function to attempt to resubscribe to the topics already present in
 * the subscription list.
 *
 * This function will be invoked when this demo requests the broker to
 * reestablish the session and the broker cannot do so. This function will
 * enqueue commands to the MQTT Agent queue and will be processed once the
 * command loop starts.
 *
 * @return `MQTTSuccess` if adding subscribes to the command queue succeeds,
 * else appropriate error code from MQTTAgent_Subscribe.
 * */
static MQTTStatus_t prvHandleResubscribe(void) {
    MQTTStatus_t xResult = MQTTBadParameter;
    uint32_t ulIndex = 0U;
    uint16_t usNumSubscriptions = 0U;

    /* These variables need to stay in scope until command completes. */
    static MQTTAgentSubscribeArgs_t xSubArgs = {0};
    static MQTTSubscribeInfo_t
        xSubInfo[SUBSCRIPTION_MANAGER_MAX_SUBSCRIPTIONS] = {0};
    static MQTTAgentCommandInfo_t xCommandParams = {0};

    /* Loop through each subscription in the subscription list and add a
     * subscribe command to the command queue. */
    for (ulIndex = 0U; ulIndex < SUBSCRIPTION_MANAGER_MAX_SUBSCRIPTIONS;
         ulIndex++) {
        /* Check if there is a subscription in the subscription list. This demo
         * doesn't check for duplicate subscriptions. */
        if (xGlobalSubscriptionList[ulIndex].usFilterStringLength != 0) {
            xSubInfo[usNumSubscriptions].pTopicFilter =
                xGlobalSubscriptionList[ulIndex].pcSubscriptionFilterString;
            xSubInfo[usNumSubscriptions].topicFilterLength =
                xGlobalSubscriptionList[ulIndex].usFilterStringLength;

            /* QoS1 is used for all the subscriptions in this demo. */
            xSubInfo[usNumSubscriptions].qos = MQTTQoS1;

            LogInfo(("Resubscribe to the topic %.*s will be attempted.",
                     xSubInfo[usNumSubscriptions].topicFilterLength,
                     xSubInfo[usNumSubscriptions].pTopicFilter));

            usNumSubscriptions++;
        }
    }

    if (usNumSubscriptions > 0U) {
        xSubArgs.pSubscribeInfo = xSubInfo;
        xSubArgs.numSubscriptions = usNumSubscriptions;

        /* The block time can be 0 as the command loop is not running at this
         * point. */
        xCommandParams.blockTimeMs = 0U;
        xCommandParams.cmdCompleteCallback = prvSubscriptionCommandCallback;
        xCommandParams.pCmdCompleteCallbackContext = (void*)&xSubArgs;

        /* Enqueue subscribe to the command queue. These commands will be
         * processed only when command loop starts. */
        xResult = MQTTAgent_Subscribe(&mqtt_agent_context, &xSubArgs,
                                      &xCommandParams);
    } else {
        /* Mark the resubscribe as success if there is nothing to be subscribed.
         */
        xResult = MQTTSuccess;
    }

    if (xResult != MQTTSuccess) {
        LogError(("Failed to enqueue the MQTT subscribe command. xResult=%s.",
                  MQTT_Status_strerror(xResult)));
    }

    return xResult;
}

/**
 * @brief Sends an MQTT Connect packet over the already connected TCP socket.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 * @param[in] xCleanSession If a clean session should be established.
 *
 * @return `MQTTSuccess` if connection succeeds, else appropriate error code
 * from MQTT_Connect.
 */
static MQTTStatus_t prvMQTTConnect(bool xCleanSession) {
    MQTTStatus_t xResult;
    MQTTConnectInfo_t xConnectInfo;
    bool xSessionPresent = false;

    /* Many fields are not used in this demo so start with everything at 0. */
    memset(&xConnectInfo, 0x00, sizeof(xConnectInfo));

    /* Start with a clean session i.e. direct the MQTT broker to discard any
     * previous session data. Also, establishing a connection with clean session
     * will ensure that the broker does not store any data when this client
     * gets disconnected. */
    xConnectInfo.cleanSession = xCleanSession;

    /* The client identifier is used to uniquely identify this MQTT client to
     * the MQTT broker. In a production device the identifier can be something
     * unique, such as a device serial number. */
    xConnectInfo.pClientIdentifier = democonfigCLIENT_IDENTIFIER;
    xConnectInfo.clientIdentifierLength =
        (uint16_t)strlen(democonfigCLIENT_IDENTIFIER);

    /* Set MQTT keep-alive period. It is the responsibility of the application
     * to ensure that the interval between Control Packets being sent does not
     * exceed the Keep Alive value. In the absence of sending any other Control
     * Packets, the Client MUST send a PINGREQ Packet.  This responsibility will
     * be moved inside the agent. */
    xConnectInfo.keepAliveSeconds = KEEP_ALIVE_INTERVAL_SECONDS;

    xConnectInfo.pUserName = democonfigCLIENT_USERNAME;
    xConnectInfo.userNameLength = (uint16_t)strlen(democonfigCLIENT_USERNAME);
    xConnectInfo.pPassword = democonfigCLIENT_PASSWORD;
    xConnectInfo.passwordLength = (uint16_t)strlen(democonfigCLIENT_PASSWORD);

    /* Send MQTT CONNECT packet to broker. MQTT's Last Will and Testament
     * feature is not used in this demo, so it is passed as NULL. */
    xResult = MQTT_Connect(&(mqtt_agent_context.mqttContext), &xConnectInfo,
                           NULL, CONNACK_RECV_TIMEOUT_MS, &xSessionPresent);

    LogInfo(("Session present: %d\n", xSessionPresent));

    /* Resume a session if desired. */
    if ((xResult == MQTTSuccess) && (xCleanSession == false)) {
        xResult = MQTTAgent_ResumeSession(&mqtt_agent_context, xSessionPresent);

        /* Resubscribe to all the subscribed topics. */
        if ((xResult == MQTTSuccess) && (xSessionPresent == false)) {
            xResult = prvHandleResubscribe();
        }
    }

    return xResult;
}

/**
 * @brief Callback executed when there is activity on the TCP socket that is
 * connected to the MQTT broker.  If there are no messages in the MQTT agent's
 * command queue then the callback send a message to ensure the MQTT agent
 * task unblocks and can therefore process whatever is necessary on the socket
 * (if anything) as quickly as possible.
 *
 * @param[in] pxSocket Socket with data, unused.
 */
static void prvMQTTClientSocketWakeupCallback(Socket_t pxSocket) {
    MQTTAgentCommandInfo_t xCommandParams = {0};

    /* Just to avoid compiler warnings.  The socket is not used but the function
     * prototype cannot be changed because this is a callback function. */
    (void)pxSocket;

    /* A socket used by the MQTT task may need attention.  Send an event
     * to the MQTT task to make sure the task is not blocked on xCommandQueue.
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
static BaseType_t prvSocketConnect(NetworkContext_t* pxNetworkContext) {
    BaseType_t xConnected = pdFAIL;
    BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;
    BackoffAlgorithmContext_t xReconnectParams = {0};
    uint16_t usNextRetryBackOff = 0U;
    const TickType_t xTransportTimeout = 0UL;

    PlaintextTransportStatus_t xNetworkStatus =
        PLAINTEXT_TRANSPORT_CONNECT_FAILURE;

    // BackoffAlgorithm_InitializeParams(&xReconnectParams,
    // RETRY_BACKOFF_BASE_MS,
    //                                   RETRY_MAX_BACKOFF_DELAY_MS,
    //                                   RETRY_MAX_ATTEMPTS);

    do {
        LogInfo(("Creating a TCP connection to %s:%d.",
                 democonfigMQTT_BROKER_ENDPOINT, democonfigMQTT_BROKER_PORT));
        xNetworkStatus = Plaintext_FreeRTOS_Connect(
            pxNetworkContext, democonfigMQTT_BROKER_ENDPOINT,
            democonfigMQTT_BROKER_PORT,
            TRANSPORT_SEND_RECV_TIMEOUT_MS,
            TRANSPORT_SEND_RECV_TIMEOUT_MS);
        xConnected =
            (xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS) ? pdPASS : pdFAIL;

        if (!xConnected) {
            /* Get back-off value (in milliseconds) for the next connection
             * retry. */
            // xBackoffAlgStatus = BackoffAlgorithm_GetNextBackoff(
            //     &xReconnectParams, rand(), &usNextRetryBackOff);

            // if (xBackoffAlgStatus == BackoffAlgorithmSuccess) {
            //     LogWarn(
            //         ("Connection to the broker failed. "
            //          "Retrying connection in %hu ms.",
            //          usNextRetryBackOff));
            //     vTaskDelay(pdMS_TO_TICKS(usNextRetryBackOff));
            // }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // if (xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted) {
        //     LogError(
        //         ("Connection to the broker failed, all attempts
        //         exhausted."));
        // }
    } while ((xConnected != pdPASS) &&
             (xBackoffAlgStatus == BackoffAlgorithmSuccess));

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
static BaseType_t prvSocketDisconnect(NetworkContext_t* pxNetworkContext) {
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
static void connect_broker(void) {
    BaseType_t xNetworkStatus = pdFAIL;
    MQTTStatus_t xMQTTStatus;

    /* Connect a TCP socket to the broker. */
    xNetworkStatus = prvSocketConnect(&network_context);
    configASSERT(xNetworkStatus == pdPASS);

    /* Initialize the MQTT context with the buffer and transport interface. */
    xMQTTStatus = prvMQTTInit();
    configASSERT(xMQTTStatus == MQTTSuccess);

    /* Form an MQTT connection without a persistent session. */
    xMQTTStatus = prvMQTTConnect(true);
    configASSERT(xMQTTStatus == MQTTSuccess);
}

/**
 * @brief Task used to run the MQTT agent.  In this example the first task that
 * is created is responsible for creating all the other demo tasks.  Then,
 * rather than create prvMQTTAgentTask() as a separate task, it simply calls
 * prvMQTTAgentTask() to become the agent task itself.
 *
 * This task calls MQTTAgent_CommandLoop() in a loop, until
 * MQTTAgent_Terminate() is called. If an error occurs in the command loop, then
 * it will reconnect the TCP and MQTT connections.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation.
 * Not used in this example.
 */
static void run_agent(void* pvParameters) {
    BaseType_t xNetworkResult = pdFAIL;
    MQTTStatus_t xMQTTStatus = MQTTSuccess, xConnectStatus = MQTTSuccess;
    MQTTContext_t* pMqttContext = &(mqtt_agent_context.mqttContext);

    (void)pvParameters;

    do {
        /* MQTTAgent_CommandLoop() is effectively the agent implementation.  It
         * will manage the MQTT protocol until such time that an error occurs,
         * which could be a disconnect.  If an error occurs the MQTT context on
         * which the error happened is returned so there can be an attempt to
         * clean up and reconnect however the application writer prefers. */
        xMQTTStatus = MQTTAgent_CommandLoop(&mqtt_agent_context);

        /* Success is returned for disconnect or termination. The socket should
         * be disconnected. */
        if (xMQTTStatus == MQTTSuccess) {
            /* MQTT Disconnect. Disconnect the socket. */
            xNetworkResult = prvSocketDisconnect(&network_context);
        }
        /* Error. */
        else {
            /* Reconnect TCP. */
            xNetworkResult = prvSocketDisconnect(&network_context);
            configASSERT(xNetworkResult == pdPASS);
            xNetworkResult = prvSocketConnect(&network_context);
            configASSERT(xNetworkResult == pdPASS);
            pMqttContext->connectStatus = MQTTNotConnected;
            /* MQTT Connect with a persistent session. */
            xConnectStatus = prvMQTTConnect(false);
            configASSERT(xConnectStatus == MQTTSuccess);
        }
    } while (xMQTTStatus != MQTTSuccess);
}

void mqtt_task(void* params) {
    elapsed = prvGetTimeMs();

    connect_broker();

    /* This task has nothing left to do, so rather than create the MQTT
     * agent as a separate thread, it simply calls the function that implements
     * the agent - in effect turning itself into the agent. */
    run_agent(NULL);

    /* Should not get here.  Force an assert if the task returns from
     * prvMQTTAgentTask(). */
    configASSERT(params == (void*)~1);
}

static void cmd_complete_cb(MQTTAgentCommandContext_t* ctx,
                            MQTTAgentReturnInfo_t* return_info) {
    ctx->status = return_info->returnCode;
    if (ctx->notify_task != NULL) {
        xTaskNotify(ctx->notify_task, ctx->notify_val, eSetValueWithOverwrite);
    }
}

void publish_state(const dcm_door_status_msg_t* msg) {
    static char payload[128];
    static const char topic[] = "sesame/state";

    char* state;
    switch (msg->state) {
        case DOORSTATE_CLOSED:
            state = "CLOSED";
            break;
        case DOORSTATE_OPEN:
            state = "OPEN";
            break;
        default:
            state = "UNDEF";
    }
    char* dir;
    switch (msg->direction) {
        case DOOR_DIR_UP:
            dir = "up";
            break;
        case DOOR_DIR_DOWN:
            dir = "down";
            break;
        case DOOR_DIR_STOPPED:
            dir = "stopped";
            break;
        default:
            dir = "UNDEF";
    }

    snprintf(payload, sizeof(payload), "{\"contact\":\"%s\",\"dir\":\"%s\"}",
             state, dir);

    MQTTAgentCommandContext_t command_context;
    memset(&command_context, 0, sizeof(command_context));
    command_context.notify_task = xTaskGetCurrentTaskHandle();
    command_context.notify_val = 1;

    MQTTAgentCommandInfo_t command_params = {cmd_complete_cb, &command_context,
                                             500};
    MQTTPublishInfo_t publish_info = {
        MQTTQoS0, false, false, topic, strlen(topic), payload, strlen(payload)};

    MQTTAgent_Publish(&mqtt_agent_context, &publish_info, &command_params);

    uint32_t notify_val = 0;
    xTaskNotifyWait(0, 0, &notify_val, pdMS_TO_TICKS(10000));
    if (notify_val == 1) {
        LogDebug(("MQTT message sent"));
    } else {
        LogInfo(
            ("MQTT message failed to send, status %d", command_context.status));
    }
}

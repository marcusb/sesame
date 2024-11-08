#include <stdbool.h>

#include "app_logging.h"
#include "platform/iot_network.h"
#include "platform/iot_threads.h"
#include "iot_network_manager_private.h"
#include "core_mqtt.h"
#include "iot_demo_runner.h"

static IotSemaphore_t network_conn_semaphore;
static IotNetworkManagerSubscription_t subscription = IOT_NETWORK_MANAGER_SUBSCRIPTION_INITIALIZER;

static void on_network_state_change(uint32_t network, AwsIotNetworkState_t state, void * pContext) {
    IotLogInfo("network state change: state=%d", state);
}

static void wait_for_conn() {
    IotSemaphore_Wait(&network_conn_semaphore);
}

static int init() {
    bool semaphore_created = false;

    if (!AwsIotNetworkManager_Init()) {
        IotLogError("failed to initialize network manager library");
        goto err;
    }
    if (!IotSemaphore_Create(&network_conn_semaphore, 0, 1)) {
        IotLogError("failed to create semaphore for network connection");
        goto err;
    }
    if (!AwsIotNetworkManager_SubscribeForStateChange(
        AWSIOT_NETWORK_TYPE_WIFI, on_network_state_change, NULL, &subscription)) {
        IotLogError("failed to subscribe to network state change");
        goto err;
    }
    if (AwsIotNetworkManager_EnableNetwork(configENABLED_NETWORKS) != configENABLED_NETWORKS) {
        IotLogError("failed to initialize networks");
        goto err;
    }
    if (!(AwsIotNetworkManager_GetConnectedNetworks() & AWSIOT_NETWORK_TYPE_WIFI)) {
        IotLogInfo("waiting for network connection");
        wait_for_conn();
    }

    return 1;

err:
    if (semaphore_created) {
        IotSemaphore_Destroy(&network_conn_semaphore);
    }
    return 0;
}



extern void network_task(void *const params) {
    int res = init();
    if (res) {
        RunCoreMqttAgentDemo(false, NULL, NULL, NULL, NULL);
    }
    
    while (true) {

    }
}

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "transport_plaintext.h"

// application
#include "app_logging.h"
#include "backoff_algorithm.h"
#include "core_http_client.h"
#include "ota.h"

#define RETRY_MAX_ATTEMPTS 3
#define RETRY_MAX_BACKOFF_DELAY_MS 5000
#define RETRY_BACKOFF_BASE_MS 500

static const char METHOD_GET[] = "GET";

// reserve some extra room for headers in the response buffer
#define READ_SIZE 1024
#define BUF_SIZE (READ_SIZE + 512)

struct NetworkContext {
    PlaintextTransportParams_t* pParams;
};

static PlaintextTransportParams_t transport_params;
static NetworkContext_t network_context = {&transport_params};
static TransportInterface_t transport = {
    Plaintext_FreeRTOS_recv, Plaintext_FreeRTOS_send, NULL, &network_context};

static void do_ota_update() {
    ota_upd_state_t ota_state;
    if (ota_init(&ota_state)) {
        return;
    }

    const HTTPRequestInfo_t req = {
        METHOD_GET,
        strlen(req.pMethod),
        "/sesame.fw.bin",
        strlen(req.pPath),
        "172.16.10.1:8000",
        strlen(req.pHost),
        HTTP_REQUEST_KEEP_ALIVE_FLAG,
    };

    BackoffAlgorithmStatus_t retry_status;
    BackoffAlgorithmContext_t retry_params;
    BackoffAlgorithm_InitializeParams(&retry_params, RETRY_BACKOFF_BASE_MS,
                                      RETRY_MAX_BACKOFF_DELAY_MS,
                                      RETRY_MAX_ATTEMPTS);
    uint16_t backoff = 0;
    HTTPStatus_t status;
    int pos = 0;
    int bytes_read = 0;
    uint8_t* buf = pvPortMalloc(BUF_SIZE);

    do {
        PlaintextTransportStatus_t conn_res = Plaintext_FreeRTOS_Connect(
            &network_context, "172.16.1.10", 8000, 1000, 1000);
        if (conn_res != PLAINTEXT_TRANSPORT_SUCCESS) {
            goto conn_err;
        }

        do {
            HTTPRequestHeaders_t headers;
            headers.pBuffer = buf;
            headers.bufferLen = BUF_SIZE;
            status = HTTPClient_InitializeRequestHeaders(&headers, &req);
            if (status != HTTPSuccess) {
                goto req_err;
            }

            status =
                HTTPClient_AddRangeHeader(&headers, pos, pos + READ_SIZE - 1);
            if (status != HTTPSuccess) {
                goto req_err;
            }

            HTTPResponse_t resp = {0};
            resp.pBuffer = buf;
            resp.bufferLen = BUF_SIZE;

            status = HTTPClient_Send(&transport, &headers, NULL, 0, &resp, 0);
            if (status != HTTPSuccess) {
                goto req_err;
            }
            bytes_read = resp.bodyLen;
            LogDebug(("updating mcufw ofs=0x%x, len=%d", pos, bytes_read));
            if (ota_write_chunk(&ota_state, resp.pBody, bytes_read)) {
                goto ret;
            }
            pos += bytes_read;

            BackoffAlgorithm_InitializeParams(
                &retry_params, RETRY_BACKOFF_BASE_MS,
                RETRY_MAX_BACKOFF_DELAY_MS, RETRY_MAX_ATTEMPTS);
        } while (bytes_read == READ_SIZE);

        ota_finish(&ota_state);
        LogInfo(("mcufw update complete"));
        goto ret;  // not reached

    req_err:
        LogError(("HTTP req failed: %s", HTTPClient_strerror(status)));
        Plaintext_FreeRTOS_Disconnect(&network_context);
    conn_err:
        retry_status =
            BackoffAlgorithm_GetNextBackoff(&retry_params, rand(), &backoff);
        if (retry_status == BackoffAlgorithmRetriesExhausted) {
            goto ret;
        }
        LogInfo(("attempt %d of %d failed, sleeping %d ms",
                 retry_params.attemptsDone, retry_params.maxRetryAttempts,
                 backoff));
        vTaskDelay(pdMS_TO_TICKS(backoff));
    } while (true);
ret:
    vPortFree(buf);
}

void ota_task(void* params) {
    QueueHandle_t queue = (QueueHandle_t)params;
    ota_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(queue, &cmd, portMAX_DELAY) == pdPASS) {
            switch (cmd) {
                case OTA_CMD_UPGRADE:
                    LogInfo(("mcufw update requested"));
                    do_ota_update();
                    break;

                case OTA_CMD_PROMOTE:
                    ota_promote_image();
                    break;

                default:
                    LogError(("unknown ota cmd %d", cmd));
            }
        }
    }
}

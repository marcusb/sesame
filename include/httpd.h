#ifndef _HTTPD_H
#define _HTTPD_H

#include "FreeRTOS.h"
#include "queue.h"
void httpd_task(void *);

typedef struct httpd_params {
    QueueHandle_t ota_queue;
} httpd_params_t;

#endif

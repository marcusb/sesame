#include <stdlib.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"

BaseType_t xApplicationGetRandomNumber(uint32_t* pulNumber) {
    *pulNumber = rand();
    return pdTRUE;
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort) {
    uint32_t res;
    xApplicationGetRandomNumber(&res);
    return res;
}

#pragma once

#include "fsl_sdio.h"
#include "fsl_sdmmc_host.h"
#include "fsl_sdmmc_common.h"

#define BOARD_SDMMC_SDIO_HOST_BASEADDR SDIOC
#define BOARD_SDMMC_SDIO_HOST_IRQ      SDIO_IRQn

#define BOARD_SDMMC_SD_CD_TYPE                       kSD_DetectCardByHostCD
#define BOARD_SDMMC_SD_CARD_DETECT_DEBOUNCE_DELAY_MS (100U)

#define BOARD_SDMMC_SDIO_HOST_IRQ_PRIORITY (5U)

#define BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE (1U)

#ifdef SDIO_ENABLED
void BOARD_SDIO_Config(void *card, sd_cd_t cd, uint32_t hostIRQPriority, sdio_int_t cardInt);
#endif

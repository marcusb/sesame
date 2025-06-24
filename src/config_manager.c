#include "app_logging.h"

// wmsdk
#include "psm-v2.h"

// Application
#include "app_config.pb.h"
#include "config_manager.h"
#include "pb_decode.h"
#include "pb_encode.h"

extern psm_hnd_t psm_hnd;

// key is written to flash by PSM, so it cannot reside in memory-mapped
// flash in XIP mode (see mflash_drv_write())
__attribute__((section(".data"))) static const char psm_key_config[] = "appcfg";
static uint8_t buf[1024];

AppConfig app_config = AppConfig_init_zero;

int load_config() {
    int ret = psm_get_variable(psm_hnd, psm_key_config, buf, sizeof(buf));
    if (ret < 0) {
        LogDebug(("psm read config failed %d", ret));
        return ret;
    }

    pb_istream_t stream = pb_istream_from_buffer(buf, ret);
    bool status = pb_decode(&stream, AppConfig_fields, &app_config);
    if (!status) {
        LogWarn(("decode conf object failed: %s", PB_GET_ERROR(&stream)));
        return -1;
    }
    return 0;
}

int save_config() {
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    bool status = pb_encode(&stream, AppConfig_fields, &app_config);
    size_t len = stream.bytes_written;
    if (!status) {
        LogWarn(("encode conf object failed: %s", PB_GET_ERROR(&stream)));
        return -1;
    }

    int ret = psm_set_variable(psm_hnd, psm_key_config, buf, len);
    if (ret < 0) {
        LogError(("psm write ssid failed %d", ret));
        return ret;
    }
    return 0;
}

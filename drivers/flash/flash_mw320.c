#define DT_DRV_COMPAT nxp_mw320_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include "mflash_drv.h"

static int flash_mw320_read(const struct device *dev, off_t offset, void *data, size_t len)
{
    int ret = mflash_drv_read(offset, (uint32_t *)data, len);
    return (ret == 0) ? 0 : -EIO;
}

static int flash_mw320_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
    int ret = mflash_drv_write(offset, (uint32_t *)data, len);
    return (ret == 0) ? 0 : -EIO;
}

static int flash_mw320_erase(const struct device *dev, off_t offset, size_t size)
{
    int ret = mflash_drv_erase(offset, size);
    return (ret == 0) ? 0 : -EIO;
}

static const struct flash_parameters *flash_mw320_get_parameters(const struct device *dev)
{
    static const struct flash_parameters flash_params = {
        .write_block_size = 4,
        .erase_value = 0xff,
    };
    return &flash_params;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_mw320_pages_layout(const struct device *dev,
                                     const struct flash_pages_layout **layout,
                                     size_t *layout_size)
{
    static const struct flash_pages_layout flash_layout = {
        .pages_count = 8192,
        .pages_size = 4096,
    };
    *layout = &flash_layout;
    *layout_size = 1;
}
#endif

static const struct flash_driver_api flash_mw320_api = {
    .read = flash_mw320_read,
    .write = flash_mw320_write,
    .erase = flash_mw320_erase,
    .get_parameters = flash_mw320_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
    .page_layout = flash_mw320_pages_layout,
#endif
};

static int flash_mw320_init(const struct device *dev)
{
    return mflash_drv_init() == 0 ? 0 : -EIO;
}

DEVICE_DT_INST_DEFINE(0, flash_mw320_init, NULL, NULL, NULL, POST_KERNEL,
                      CONFIG_FLASH_INIT_PRIORITY, &flash_mw320_api);

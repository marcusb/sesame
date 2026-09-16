#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/arch/common/semihost.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define NVS_ID_NETWORK_CONFIG 1

#ifdef CONFIG_BOARD_NATIVE_SIM
#define NVS_PARTITION storage_partition
#else
#define NVS_PARTITION psm_partition
#endif

#define NVS_PARTITION_DEVICE DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(DT_NODELABEL(NVS_PARTITION)))
#define NVS_PARTITION_OFFSET DT_REG_ADDR(DT_NODELABEL(NVS_PARTITION))
#define NVS_PARTITION_SIZE DT_REG_SIZE(DT_NODELABEL(NVS_PARTITION))

static struct nvs_fs fs;

static int inject_test_config(void)
{
    printk(">>> inject_test_config started!\n");
    struct flash_pages_info info;
    const struct device* flash_dev = NVS_PARTITION_DEVICE;
    
    if (!device_is_ready(flash_dev)) {
        printk(">>> Flash device not ready\n");
        return -ENODEV;
    }

    fs.flash_device = flash_dev;
    fs.offset = NVS_PARTITION_OFFSET;
    
    if (flash_get_page_info_by_offs(flash_dev, fs.offset, &info) != 0) {
        printk(">>> Failed to get flash page info\n");
        return -1;
    }
    fs.sector_size = info.size;
    fs.sector_count = NVS_PARTITION_SIZE / info.size;

    if (nvs_mount(&fs) != 0) {
        printk(">>> NVS mount failed in injector\n");
        return -1;
    }

#ifdef CONFIG_SEMIHOST
    printk(">>> Opening test_config.bin via semihosting...\n");
    long fd = semihost_open("test_config.bin", SEMIHOST_OPEN_RB);
    if (fd >= 0) {
        long len = semihost_flen(fd);
        printk(">>> semihost_flen returned %ld\n", len);
        if (len > 0) {
            uint8_t buf[256];
            long read_len = semihost_read(fd, buf, len);
            if (len <= sizeof(buf) && read_len == len) {
                int rc = nvs_write(&fs, NVS_ID_NETWORK_CONFIG, buf, len);
                printk(">>> Injected NetworkConfig into NVS, size=%ld, rc=%d\n", len, rc);
            } else {
                printk(">>> Failed to read test_config.bin, read_len=%ld\n", read_len);
            }
        } else {
            printk(">>> File is empty or flen failed\n");
        }
        semihost_close(fd);
    } else {
        printk(">>> Failed to open test_config.bin, fd=%ld\n", fd);
    }
#else
    printk(">>> CONFIG_SEMIHOST is not defined!\n");
#endif

    return 0;
}

SYS_INIT(inject_test_config, APPLICATION, 80);

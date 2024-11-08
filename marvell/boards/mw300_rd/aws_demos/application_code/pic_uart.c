#include <string.h>

#include "FreeRTOS.h"
#include "mdev_gpio.h"
#include "mdev_uart.h"
#include "task.h"
#include "wm_utils.h"

#include "iot_logging_task.h"

#include "pic_uart.h"

#define MAX_MSG_LENGTH 79
#define READ_TIMEOUT_MS 10000
#define RX_BUF_SIZE 256

typedef enum { READ_START = 0, READ_LENGTH = 1, READ_BODY = 2 } ReadState;

static uint8_t buf[80];
static mdev_t *uart_dev;
TickType_t last_read_tick;

static uint8_t calc_chk_sum(uint8_t *p, int n) {
  if (n < 2) {
    return 0;
  }
  uint8_t res = 0;
  while (n > 1) {
    res = res ^ *p++;
  }
  return res;
}

static void read_msg() {
  memset(buf, 0, sizeof(buf));
  int n = 0;
  int msg_len = 0;
  int more = 1;
  uint8_t *p = buf;
  ReadState state = READ_START;
  last_read_tick = xTaskGetTickCount();
  while (true) {
    vTaskDelay(100);
    TickType_t tick = xTaskGetTickCount();
    if (tick > last_read_tick + READ_TIMEOUT_MS) {
      vLoggingPrintfWarn("read timeout, read %d chars, waiting for %d more", n,
                         more);
      return;
    }
    int len = uart_drv_read(uart_dev, p, more);
    if (len == 0) {
      continue;
    }
    n += len;
    uint8_t cmd = buf[3];
    if (state == READ_START) {
      if (*p++ != 0x55) {
        return;
      }
      state = READ_LENGTH;
    } else if (state == READ_LENGTH) {
      msg_len = *p++ + 5;
      if (msg_len >= MAX_MSG_LENGTH) {
        vLoggingPrintfInfo("invalid msg length %d", msg_len);
        return;
      }
      more = msg_len - n;
      state = READ_BODY;
    } else {
      p += len;
      more -= len;
      if (msg_len > 5 && msg_len == n) {
        dump_hex(buf, n);
        tick = xTaskGetTickCount();
        last_read_tick = tick;
        uint8_t chksum = calc_chk_sum(buf, n);
        if (chksum != buf[n - 1]) {
          vLoggingPrintfWarn("chksum mismatch (got 0x%02x, expected 0x%02x)",
                             chksum, buf[n - 1]);
        }
        vLoggingPrintf("read msg type %d", cmd);
      }
    }
  }
}

static int init_uart(void) {
  gpio_drv_init();
  mdev_t *gpio_dev = gpio_drv_open("MDEV_GPIO");
  if (gpio_dev == NULL) {
    vLoggingPrintfError("gpio_drv_open failed");
    return -1;
  }

  gpio_drv_setdir(gpio_dev, 48, GPIO_OUTPUT);
  gpio_drv_write(gpio_dev, 48, GPIO_IO_HIGH);

  gpio_drv_setdir(gpio_dev, 49, GPIO_OUTPUT);
  gpio_drv_write(gpio_dev, 49, GPIO_IO_LOW);

  int err = uart_drv_init(UART1_ID, UART_8BIT);
  if (err) {
    vLoggingPrintfError("uart_drv_init failed (%d)\n", err);
    return -1;
  }

  uart_drv_rxbuf_size(UART1_ID, RX_BUF_SIZE);
  vTaskDelay(25);

  uart_dev = uart_drv_open(UART1_ID, 115200);
  if (uart_dev == NULL) {
    vLoggingPrintfError("uart_drv_open failed (%d)\n", err);
    return -1;
  }

  gpio_drv_write(gpio_dev, 48, GPIO_IO_LOW);
  vTaskDelay(5);

  return 0;
}

extern void picUartTask(void *const params) {
  configPRINT("PIC comm task running\r\n");

  if (init_uart() != 0) {
    vTaskDelete(NULL);
    return;
  }

  while (true) {
    read_msg();
    vTaskDelay(800);
  }
}

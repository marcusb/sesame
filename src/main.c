#include <zephyr/kernel.h>

void main(void) {
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}

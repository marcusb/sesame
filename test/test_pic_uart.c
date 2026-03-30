/*
 * Access static functions in pic_uart.c via direct inclusion.
 * calc_chk_sum and next_token are pure-logic functions with no hardware
 * dependencies.  process_serial_data is exercised with real wire frames
 * captured from the device; hardware calls (gpio_set/clear,
 * UART_TransferReceiveNonBlocking) are satisfied by no-op stubs.
 */
/* pic_uart.c brings in fsl_uart.h, fsl_gpio.h, gpio.h — all SDK types available below */
#include "pic_uart.c"

#include <string.h>
#include "controller.h"
#include "unity.h"

/*
 * gpio_set/gpio_clear stubs — replace gpio.c which is not linked in the test
 * binary (it would pull in GPIO interrupt and init code we don't need).
 * UART_TransferReceiveNonBlocking and UART_TransferAbortReceive are already
 * provided by libmw320_drivers_uart.a; the real implementations harmlessly
 * set up a receive that never fires since no UART hardware is running.
 */
void gpio_set(int pin)   { (void)pin; }
void gpio_clear(int pin) { (void)pin; }

/* ---- calc_chk_sum ---- */

static void test_calc_chk_sum_n_zero(void) {
    uint8_t data[] = {0x55, 0x10};
    TEST_ASSERT_EQUAL(0, calc_chk_sum(data, 0));
}

static void test_calc_chk_sum_n_one(void) {
    uint8_t data[] = {0x55, 0x10};
    TEST_ASSERT_EQUAL(0, calc_chk_sum(data, 1));
}

static void test_calc_chk_sum_n_two(void) {
    /* n=2: XOR bytes[1..1] only */
    uint8_t data[] = {0x55, 0xAB};
    TEST_ASSERT_EQUAL(0xAB, calc_chk_sum(data, 2));
}

static void test_calc_chk_sum_known(void) {
    /* XOR bytes[1..3] = 0x02 ^ 0x01 ^ 0x10 = 0x13 */
    uint8_t data[] = {0x55, 0x02, 0x01, 0x10};
    TEST_ASSERT_EQUAL(0x13, calc_chk_sum(data, 4));
}

static void test_calc_chk_sum_all_same(void) {
    /* 0x55 ^ 0x55 ^ 0x55 = 0x55 (odd count XOR) */
    uint8_t data[] = {0x00, 0x55, 0x55, 0x55};
    TEST_ASSERT_EQUAL(0x55, calc_chk_sum(data, 4));
}

/* ---- next_token ---- */

static void test_next_token_never_zero(void) {
    for (int i = 0; i < 300; i++) {
        TEST_ASSERT_NOT_EQUAL(0, next_token());
    }
}

static void test_next_token_never_77_or_85(void) {
    for (int i = 0; i < 300; i++) {
        uint8_t t = next_token();
        TEST_ASSERT_NOT_EQUAL(77, t);
        TEST_ASSERT_NOT_EQUAL(85, t);
    }
}

static void test_next_token_range(void) {
    for (int i = 0; i < 300; i++) {
        uint8_t t = next_token();
        TEST_ASSERT_GREATER_OR_EQUAL(1, t);
        TEST_ASSERT_LESS_OR_EQUAL(252, t);
    }
}

/* ---- process_serial_data ---- */

/*
 * Real DCM_MSG_DOOR_STATUS_UPDATE wire frames captured from the device log.
 * Layout: [0x55][len=0x0e][seq][type=0x16][14-byte payload][checksum]
 */

/* door open, moving up: state=OPEN(1), dir=UP(1), up_lim=32881, down_lim=32763 */
static const uint8_t frame_open_up[] = {
    0x55, 0x0e, 0xda, 0x16,
    0x11, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x80, 0x71, 0x80, 0xfb, 0x7f,
    0x00, 0x00, 0x20
};

/* door open, moving down: state=OPEN(1), dir=DOWN(0) */
static const uint8_t frame_open_down[] = {
    0x55, 0x0e, 0xde, 0x16,
    0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x67, 0x80, 0x71, 0x80, 0xfb, 0x7f,
    0x00, 0x00, 0x51
};

/* door closed, stopped: state=CLOSED(0), dir=STOPPED(2) */
static const uint8_t frame_closed_stopped[] = {
    0x55, 0x0e, 0xe0, 0x16,
    0x12, 0x00, 0x00, 0x00, 0x00, 0x02, 0xfc, 0x7f, 0x71, 0x80, 0xfb, 0x7f,
    0x00, 0x00, 0x1e
};

/* Reset module-level statics (accessible since we #include "pic_uart.c"). */
static void reset_pic_uart_state(void) {
    state               = DCM_DOOR_STATE_UNKNOWN;
    direction           = DCM_DOOR_DIR_UNKNOWN;
    pos                 = 0;
    down_limit          = 0;
    up_limit            = 0;
    last_state_pub_time = 0;
    read_state          = READ_HEADER;
}

/*
 * Feed a complete DCM wire frame through process_serial_data() in two calls —
 * header phase then body phase — mirroring how the UART ISR drives the task.
 * UART_TransferReceiveNonBlocking is a stub no-op, so we fill the body bytes
 * into rx_buf manually before the second call.
 */
static void feed_frame(const uint8_t *frame, size_t len) {
    memcpy(rx_buf, frame, 4);
    read_state = READ_HEADER;
    process_serial_data();
    memcpy(rx_buf + 4, frame + 4, len - 4);
    process_serial_data();
}

static void test_door_status_open_up(void) {
    reset_pic_uart_state();
    ctrl_queue = xQueueCreate(4, sizeof(ctrl_msg_t));
    TEST_ASSERT_NOT_NULL(ctrl_queue);

    feed_frame(frame_open_up, sizeof(frame_open_up));

    ctrl_msg_t msg;
    TEST_ASSERT_TRUE(xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_STATE_UPDATE, msg.type);
    TEST_ASSERT_EQUAL(DCM_DOOR_STATE_OPEN,  msg.msg.door_state.state);
    TEST_ASSERT_EQUAL(DCM_DOOR_DIR_UP,      msg.msg.door_state.direction);

    vQueueDelete(ctrl_queue);
    ctrl_queue = NULL;
}

static void test_door_status_open_down(void) {
    reset_pic_uart_state();
    ctrl_queue = xQueueCreate(4, sizeof(ctrl_msg_t));
    TEST_ASSERT_NOT_NULL(ctrl_queue);

    feed_frame(frame_open_down, sizeof(frame_open_down));

    ctrl_msg_t msg;
    TEST_ASSERT_TRUE(xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_STATE_UPDATE, msg.type);
    TEST_ASSERT_EQUAL(DCM_DOOR_STATE_OPEN,  msg.msg.door_state.state);
    TEST_ASSERT_EQUAL(DCM_DOOR_DIR_DOWN,    msg.msg.door_state.direction);

    vQueueDelete(ctrl_queue);
    ctrl_queue = NULL;
}

static void test_door_status_closed_stopped(void) {
    reset_pic_uart_state();
    ctrl_queue = xQueueCreate(4, sizeof(ctrl_msg_t));
    TEST_ASSERT_NOT_NULL(ctrl_queue);

    feed_frame(frame_closed_stopped, sizeof(frame_closed_stopped));

    ctrl_msg_t msg;
    TEST_ASSERT_TRUE(xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_STATE_UPDATE, msg.type);
    TEST_ASSERT_EQUAL(DCM_DOOR_STATE_CLOSED,    msg.msg.door_state.state);
    TEST_ASSERT_EQUAL(DCM_DOOR_DIR_STOPPED,     msg.msg.door_state.direction);

    vQueueDelete(ctrl_queue);
    ctrl_queue = NULL;
}

static void test_door_status_bad_header(void) {
    reset_pic_uart_state();
    ctrl_queue = xQueueCreate(4, sizeof(ctrl_msg_t));
    TEST_ASSERT_NOT_NULL(ctrl_queue);

    uint8_t bad[sizeof(frame_open_up)];
    memcpy(bad, frame_open_up, sizeof(bad));
    bad[0] = 0xAA;

    memcpy(rx_buf, bad, 4);
    read_state = READ_HEADER;
    process_serial_data();

    ctrl_msg_t msg;
    TEST_ASSERT_FALSE(xQueueReceive(ctrl_queue, &msg, 0));

    vQueueDelete(ctrl_queue);
    ctrl_queue = NULL;
}

static void test_door_status_checksum_mismatch(void) {
    reset_pic_uart_state();
    ctrl_queue = xQueueCreate(4, sizeof(ctrl_msg_t));
    TEST_ASSERT_NOT_NULL(ctrl_queue);

    uint8_t bad[sizeof(frame_open_up)];
    memcpy(bad, frame_open_up, sizeof(bad));
    bad[sizeof(bad) - 1] ^= 0xff;  /* corrupt checksum */

    feed_frame(bad, sizeof(bad));

    ctrl_msg_t msg;
    TEST_ASSERT_FALSE(xQueueReceive(ctrl_queue, &msg, 0));

    vQueueDelete(ctrl_queue);
    ctrl_queue = NULL;
}

void run_test_pic_uart(void) {
    RUN_TEST(test_calc_chk_sum_n_zero);
    RUN_TEST(test_calc_chk_sum_n_one);
    RUN_TEST(test_calc_chk_sum_n_two);
    RUN_TEST(test_calc_chk_sum_known);
    RUN_TEST(test_calc_chk_sum_all_same);
    RUN_TEST(test_next_token_never_zero);
    RUN_TEST(test_next_token_never_77_or_85);
    RUN_TEST(test_next_token_range);
    RUN_TEST(test_door_status_open_up);
    RUN_TEST(test_door_status_open_down);
    RUN_TEST(test_door_status_closed_stopped);
    RUN_TEST(test_door_status_bad_header);
    RUN_TEST(test_door_status_checksum_mismatch);
}

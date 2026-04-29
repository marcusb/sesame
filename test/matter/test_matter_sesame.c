#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "berry.h"
#include "controller.h"
#include "matter_task.h"
#include "matter_test_utils.h"
#include "queue.h"
#include "task.h"
#include "unity.h"

extern QueueHandle_t ctrl_queue;

void setUp(void) {
    matter_test_setup();
    ctrl_queue = xQueueCreate(1, sizeof(ctrl_msg_t));
}

void tearDown(void) {
    matter_test_teardown();
    vQueueDelete(ctrl_queue);
}

void test_sesame_door_open(void) {
    be_assert_success("import sesame; sesame.door_cmd(1)");

    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_CONTROL, msg.type);
    TEST_ASSERT_EQUAL(DOOR_CMD_OPEN, msg.msg.door_control.command);
}

void test_sesame_door_close(void) {
    be_assert_success("import sesame; sesame.door_cmd(2)");

    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_CONTROL, msg.type);
    TEST_ASSERT_EQUAL(DOOR_CMD_CLOSE, msg.msg.door_control.command);
}

void test_sesame_door_stop(void) {
    be_assert_success("import sesame; sesame.door_cmd(0)");

    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_CONTROL, msg.type);
    TEST_ASSERT_EQUAL(DOOR_CMD_STOP, msg.msg.door_control.command);
}

void test_matter_module_load(void) {
    be_assert_success(
        "import matter; assert(matter.Counter != nil); "
        "assert(matter.Verhoeff != nil); assert(matter.Commissioning != nil)");
}

void test_matter_door_plugin_control(void) {
    // 1. Define the plugin class (copied from matter_task.c)
    be_assert_success(
        "import matter\n"
        "import sesame\n"
        "class Matter_Door_Plugin : matter.Plugin_Shutter\n"
        "  static var TYPE = \"sesame_door\"\n"
        "  def init(device, endpoint, conf)\n"
        "    super(self).init(device, endpoint, conf)\n"
        "    self.shadow_shutter_inverted = 1\n"
        "  end\n"
        "  def invoke_request(session, val, ctx)\n"
        "    var cluster = ctx.cluster\n"
        "    var command = ctx.command\n"
        "    if cluster == 0x0102\n"
        "      if   command == 0x0000 sesame.door_cmd(1) return true\n"
        "      elif command == 0x0001 sesame.door_cmd(2) return true\n"
        "      elif command == 0x0002 sesame.door_cmd(0) return true\n"
        "      end\n"
        "    end\n"
        "    return super(self).invoke_request(session, val, ctx)\n"
        "  end\n"
        "end\n");

    // 2. Instantiate it
    be_assert_success("p = Matter_Door_Plugin(nil, 2, {})");

    // 3. Simulate UpOrOpen command (0x00)
    be_assert_success(
        "class DummyCtx var cluster, command end\n"
        "ctx = DummyCtx()\n"
        "ctx.cluster = 0x0102; ctx.command = 0x00\n"
        "p.invoke_request(nil, nil, ctx)");
    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(DOOR_CMD_OPEN, msg.msg.door_control.command);

    // 4. Simulate DownOrClose command (0x01)
    be_assert_success("ctx.command = 0x01; p.invoke_request(nil, nil, ctx)");
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(DOOR_CMD_CLOSE, msg.msg.door_control.command);

    // 5. Simulate StopMotion command (0x02)
    be_assert_success("ctx.command = 0x02; p.invoke_request(nil, nil, ctx)");
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(DOOR_CMD_STOP, msg.msg.door_control.command);
}

void test_matter_door_plugin_report(void) {
    // 1. Setup plugin
    be_assert_success(
        "import matter\n"
        "import sesame\n"
        "import global\n"
        "import tasmota\n"
        "class Matter_Profiler_Stub\n"
        "  def init() end\n"
        "  def start() end\n"
        "  def log(s) end\n"
        "  def dump(s) end\n"
        "  def set_active(v) end\n"
        "  def set_cb(cb) end\n"
        "end\n"
        "matter.Profiler = Matter_Profiler_Stub\n"
        "matter.profiler = Matter_Profiler_Stub()\n"
        "global.log = def (m,l) tasmota.log(m, l) end\n"
        "class Matter_Door_Plugin : matter.Plugin_Shutter\n"
        "  static var TYPE = \"sesame_door\"\n"
        "  def init(device, endpoint, conf)\n"
        "    super(self).init(device, endpoint, conf)\n"
        "    self.shadow_shutter_inverted = 1\n"
        "  end\n"
        "end\n"
        "matter.plugins_classes = {'root': matter.Plugin_Root, 'sesame_door': "
        "Matter_Door_Plugin}\n"
        "var d = matter.Device()\n"
        "global.matter_device = d\n"
        "d.plugins.push(matter.Plugin_Root(d, 0, {}))\n"
        "d.plugins.push(Matter_Door_Plugin(d, 2, {}))\n");

    // 2. Report state: Open, Opening, 50%
    door_state_msg_t msg = {
        .state = DCM_DOOR_STATE_OPEN, .direction = DCM_DOOR_DIR_UP, .pos = 50};
    matter_report_door_state(&msg);

    // 3. Verify shadow state
    be_assert_success(
        "var p = matter_device.find_plugin_by_endpoint(2)\n"
        "assert(p.shadow_shutter_pos == 50)\n"
        "assert(p.shadow_shutter_direction == 1)");

    // 4. Report state: Closed, Stopped, 100%
    msg.state = DCM_DOOR_STATE_CLOSED;
    msg.direction = DCM_DOOR_DIR_STOPPED;
    msg.pos = 100;
    matter_report_door_state(&msg);

    be_assert_success(
        "p = matter_device.find_plugin_by_endpoint(2)\n"
        "assert(p.shadow_shutter_pos == 100)\n"
        "assert(p.shadow_shutter_direction == 0)");
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_sesame_door_open);
    RUN_TEST(test_sesame_door_close);
    RUN_TEST(test_sesame_door_stop);
    RUN_TEST(test_matter_module_load);
    RUN_TEST(test_matter_door_plugin_control);
    RUN_TEST(test_matter_door_plugin_report);
}

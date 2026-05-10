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
#include "psm.h"
#include "queue.h"
#include "stubs/stub_psm.h"
#include "unity.h"

extern QueueHandle_t ctrl_queue;

void setUp(void) {
    matter_test_setup();
    be_assert_success(
        "import path\n"
        "path.remove('_matter_device.json')\n"
        "path.remove('_matter_commissioning.json')\n"
        "path.remove('_matter_fabrics.json')\n");
    ctrl_queue = xQueueCreate(1, sizeof(ctrl_msg_t));
}

void tearDown(void) {
    matter_test_teardown();
    if (ctrl_queue) {
        vQueueDelete(ctrl_queue);
        ctrl_queue = NULL;
    }
}

static void test_sesame_door_open(void) {
    be_assert_success(
        "import sesame\n"
        "sesame.door_cmd(0)");
    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(DOOR_CMD_OPEN, msg.msg.door_control.command);
}

static void test_sesame_door_close(void) {
    be_assert_success(
        "import sesame\n"
        "sesame.door_cmd(1)");
    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(DOOR_CMD_CLOSE, msg.msg.door_control.command);
}

static void test_sesame_door_stop(void) {
    be_assert_success(
        "import sesame\n"
        "sesame.door_cmd(2)");
    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(DOOR_CMD_STOP, msg.msg.door_control.command);
}

static void test_matter_module_load(void) {
    be_assert_success("import matter");
}

static void test_matter_door_plugin_control(void) {
    // 1. Setup plugin
    be_assert_success(
        "class Matter_Door_Plugin : matter.Plugin_Shutter\n"
        "  static var TYPE = \"sesame_door\"\n"
        "  def init(device, endpoint, conf)\n"
        "    super(self).init(device, endpoint, conf)\n"
        "    self.shadow_shutter_inverted = 1\n"
        "  end\n"
        "  def invoke_request(session, val, ctx)\n"
        "    import sesame\n"
        "    var cluster = ctx.cluster\n"
        "    var command = ctx.command\n"
        "    if cluster == 0x0102\n"
        "      if   command == 0x0000 sesame.door_cmd(0) return true\n"
        "      elif command == 0x0001 sesame.door_cmd(1) return true\n"
        "      elif command == 0x0002 sesame.door_cmd(2) return true\n"
        "      end\n"
        "    end\n"
        "    return super(self).invoke_request(session, val, ctx)\n"
        "  end\n"
        "end\n"
        "var d = matter.Device()\n"
        "p = Matter_Door_Plugin(d, 2, {})\n"
        "global.p = p");

    // 2. Invoke command (Open)
    be_assert_success(
        "var ctx = matter.Path()\n"
        "ctx.endpoint = 2; ctx.cluster = 0x0102; ctx.command = 0x00\n"
        "p.invoke_request(nil, nil, ctx)");
    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(DOOR_CMD_OPEN, msg.msg.door_control.command);
}

static void test_matter_door_plugin_report(void) {
    // 1. Setup plugin
    be_assert_success(
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
}

/* Regression: when the controller stops acking under wildcard-read load
 * (commonly an IPv6 link-local CASE session with broken source-address
 * selection), Matter_UDPServer.packets_sent used to grow without bound and
 * consume the entire heap. Bootstrap installs a capped subclass; verify it
 * actually drops the oldest unacked entry past MAX_PACKETS_QUEUED. */
static void test_udpserver_packets_sent_is_capped(void) {
    be_assert_success(
        /* Reproduce the bootstrap subclass definition under test. */
        "class Matter_UDPServer_Capped : matter.UDPServer\n"
        "  static var MAX_PACKETS_QUEUED = 4\n"
        "  def send_UDP(msg)\n"
        "    var packet = matter.UDPPacket_sent(msg)\n"
        /* skip the actual self.send(packet) — no socket in this test */
        "    if packet.msg_id\n"
        "      while size(self.packets_sent) >= self.MAX_PACKETS_QUEUED\n"
        "        self.packets_sent.remove(0)\n"
        "      end\n"
        "      self.packets_sent.push(packet)\n"
        "    end\n"
        "  end\n"
        "end\n"
        /* Fake "msg" objects with the fields UDPPacket_sent.init reads. */
        "class FakeMsg\n"
        "  var raw, remote_ip, remote_port, x_flag_r, message_counter\n"
        "  var exchange_id, local_session_id\n"
        "  def init(id)\n"
        "    self.raw = bytes('AA')\n"
        "    self.remote_ip = '::1'\n"
        "    self.remote_port = 5540\n"
        "    self.x_flag_r = true\n"
        "    self.message_counter = id\n"
        "    self.exchange_id = id\n"
        "    self.local_session_id = 1\n"
        "  end\n"
        "end\n"
        "var srv = Matter_UDPServer_Capped(nil, '', 5540)\n"
        "for i : 0..9\n"
        "  srv.send_UDP(FakeMsg(1000 + i))\n"
        "end\n"
        "assert(size(srv.packets_sent) == 4, "
        "'expected cap=4, got ' + str(size(srv.packets_sent)))\n"
        /* Cap drops the OLDEST: the surviving msg_ids must be the last 4. */
        "assert(srv.packets_sent[0].msg_id == 1006)\n"
        "assert(srv.packets_sent[3].msg_id == 1009)");
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_sesame_door_open);
    RUN_TEST(test_sesame_door_close);
    RUN_TEST(test_sesame_door_stop);
    RUN_TEST(test_matter_module_load);
    RUN_TEST(test_matter_door_plugin_control);
    RUN_TEST(test_matter_door_plugin_report);
    RUN_TEST(test_udpserver_packets_sent_is_capped);
}

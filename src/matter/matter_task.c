#include "matter_task.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "be_mapping.h"
#include "be_vm.h"
#include "berry.h"
#include "embedded_be.h"
#include "matter_mdns.h"
#include "matter_tasmota_shim.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

/* Shared between the matter task and (host-test) cmd handlers. */
bvm* g_matter_vm;
SemaphoreHandle_t g_matter_vm_lock;
static StaticSemaphore_t g_matter_vm_lock_buf;

static void matter_task(void* pvParameters) {
    (void)pvParameters;
    LogInfo(("[matter] entry"));

    bvm* vm = be_vm_new();
    if (!vm) {
        LogError(("[matter] failed to create berry vm"));
        vTaskDelete(NULL);
        return;
    }
    g_matter_vm = vm;
    LogInfo(("[matter] vm=%p", vm));

    /* Solidified matter module exposes `be_const_ctype_func(...)` entries
     * (e.g. get_cluster_name, is_attribute_reportable); without a registered
     * handler the VM errors with "missing ctype_func handler" the first time
     * any of them is invoked (observed during Matter_Device.init() calling
     * commissioning start paths). */
    be_set_ctype_func_handler(vm, be_call_ctype_func);

    LogInfo(("[matter] load crypto"));
    extern int be_load_crypto_module(bvm * vm);
    be_load_crypto_module(vm);

    /* Define crypto.SPAKE2P_Matter (Berry source — solidify pipeline isn't
     * working yet). Must run after the crypto module exists. */
    if (be_dostring(vm, embedded_be_crypto_spake2p_matter) != 0) {
        LogError(("[matter] failed to define SPAKE2P_Matter: %s",
                  be_tostring(vm, -1)));
        be_pop(vm, 1);
    } else if (be_dostring(vm,
                           "import crypto\n"
                           "crypto.SPAKE2P_Matter = SPAKE2P_Matter\n") != 0) {
        LogError(("[matter] failed to attach SPAKE2P_Matter: %s",
                  be_tostring(vm, -1)));
        be_pop(vm, 1);
    }

    LogInfo(("[matter] init globals"));
    if (be_dostring(vm,
                    "_matter_fast_cbs = []\n"
                    "_matter_fast_cbs_once = []\n"
                    "_matter_net_cbs = []\n"
                    "_matter_drivers = []\n") != 0) {
        LogError(("[matter] failed to init matter globals: %s",
                  be_tostring(vm, -1)));
    }

    LogInfo(("[matter] import tasmota"));
    if (be_dostring(vm, "import tasmota\nreturn tasmota") == 0) {
        be_setglobal(vm, "tasmota");
    } else {
        LogError(("[matter] failed to load tasmota globally: %s",
                  be_tostring(vm, -1)));
        be_pop(vm, 1);
    }

    static const char* steps[] = {
        "import matter\nimport global\nglobal.matter = matter",
        "import sesame\nimport global\nglobal.sesame = sesame",
        /* Tasmota exposes `log(msg [, level])` as a plain global; berry_matter
         * relies on it in many places. Forward to tasmota.log. */
        "import global\nglobal.log = def (m,l) tasmota.log(m, l) end",
        "class Matter_Door_Plugin : matter.Plugin_Shutter\n"
        "  static var TYPE = \"sesame_door\"\n"
        "  def init(device, endpoint, conf)\n"
        "    super(self).init(device, endpoint, conf)\n"
        "    self.shadow_shutter_inverted = 1\n"
        "  end\n"
        "  def update_shadow() end\n"
        "  def invoke_request(session, val, ctx)\n"
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
        "end",
        /* Matter_Profiler and Matter_Autoconf are stripped from the sesame
         * build; provide no-op stubs that Matter_Device.init() and
         * autoconf_device() can call without erroring. */
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
        /* Autoconf's only job here is instantiate_plugins_from_config: walk
         * plugins_config and push matter.plugins_classes.find(type)(...). */
        "class Matter_Autoconf_Stub\n"
        "  var device\n"
        "  def init(d) self.device = d end\n"
        "  def autoconf_device_map() return {} end\n"
        "  def instantiate_plugins_from_config(cfg)\n"
        "    if cfg == nil return end\n"
        "    var d = self.device\n"
        "    for k : cfg.keys()\n"
        "      var ep = int(k)\n"
        "      var conf = cfg[k]\n"
        "      var cl = d.plugins_classes.find(conf.find('type'))\n"
        "      if cl != nil d.plugins.push(cl(d, ep, conf)) end\n"
        "    end\n"
        "  end\n"
        "end\n"
        "matter.Autoconf = Matter_Autoconf_Stub",
        "matter.plugins_classes = {\"root\": matter.Plugin_Root,"
        "\"aggregator\": matter.Plugin_Aggregator,"
        "\"shutter\": matter.Plugin_Shutter,"
        "\"sesame_door\": Matter_Door_Plugin}",
        "matter_device = matter.Device()\n"
        /* Solidified Matter_Device.init() does NOT copy
         * matter.plugins_classes to the instance slot (it's only set when
         * Matter_Plugin subclasses register themselves in upstream Tasmota's
         * class-registration Berry code that we don't ship). Set it here. */
        "matter_device.plugins_classes = matter.plugins_classes",
        "matter_device.plugins_persist = true",
        "matter_device.plugins_config = {\"0\":{\"type\":\"root\"},"
        "\"2\":{\"type\":\"sesame_door\"}}",
        /* Reseed with the canonical Matter test pair (discriminator 3840,
         * passcode 20202021) and reopen commissioning with the new SPAKE2+
         * verifier so chip-tool can pair without us decoding the random
         * manual pairing code on every reboot. */
        "matter_device.commissioning.stop_basic_commissioning()\n"
        "matter_device.root_discriminator = 3840\n"
        "matter_device.root_passcode = 20202021\n"
        "matter_device.commissioning.start_root_basic_commissioning()\n"
        "log('MTR: discriminator=' + str(matter_device.root_discriminator) "
        "+ ' passcode=' + str(matter_device.root_passcode), 2)",
        /* start() is fired from tasmota.when_network_up (registered from
         * init_basic_commissioning); the callback runs in _matter_net_cbs,
         * which matter_tasmota_notify_network_up drains after bootstrap.
         * We still wrap a direct call here so we can catch and log any
         * exception, instead of having it surface as a net-cb error. */
        "var _s = 'autoconf_device'\n"
        "try\n"
        "  matter_device.autoconf_device()\n"
        "  print('[matter] autoconf ok; plugins=', "
        "size(matter_device.plugins))\n"
        "  _s = '_start_udp'\n"
        "  matter_device._start_udp(matter_device.UDP_PORT)\n"
        "  print('[matter] _start_udp ok')\n"
        "  _s = 'patch_device'\n"
        "  def _mtr_device_received_ack_fixed(self, msg)\n"
        "    var id = msg.ack_message_counter\n"
        "    var exch = msg.exchange_id\n"
        "    if id == nil return end\n"
        "    var ps = self.udp_server.packets_sent\n"
        "    var idx = 0\n"
        "    while idx < size(ps)\n"
        "      var packet = ps[idx]\n"
        "      if packet.msg_id == id && (packet.exchange_id & 0xFFFF) == "
        "(exch & 0xFFFF)\n"
        "        ps.remove(idx)\n"
        "      else\n"
        "        idx += 1\n"
        "      end\n"
        "    end\n"
        "  end\n"
        "  matter_device.received_ack = _mtr_device_received_ack_fixed\n"
        "  _s = 'start_mdns'\n"
        "  matter_device.commissioning.start_mdns_announce_hostnames()\n"
        "  print('[matter] start_mdns ok')\n"
        "except .. as e, m\n"
        "  print('[matter] start fail at', _s, ':', e, '|', m)\n"
        "end",
        NULL};
    for (int i = 0; steps[i]; i++) {
        LogInfo(("[matter] bootstrap step %d", i));
        if (be_dostring(vm, steps[i]) != 0) {
            LogError(("[matter] bootstrap step %d failed: %s", i,
                      be_tostring(vm, -1)));
            break;
        }
    }

    /* Task is started from the network-up handler, so fire the callbacks now.
     */
    matter_tasmota_notify_network_up(vm);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        xSemaphoreTakeRecursive(g_matter_vm_lock, portMAX_DELAY);
        matter_tasmota_tick(vm);
        xSemaphoreGiveRecursive(g_matter_vm_lock);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

void matter_init(void) {
    /* PRINTF deadlocks when called from this network-up hook context, so the
     * log lines must go through LogInfo (the regular logger). */
    LogInfo(("matter_init: start"));
    matter_mdns_init();
    g_matter_vm_lock =
        xSemaphoreCreateRecursiveMutexStatic(&g_matter_vm_lock_buf);
    BaseType_t rc =
        xTaskCreate(matter_task, "matter", 8192, NULL, tskIDLE_PRIORITY, NULL);
    LogInfo(("matter_init: xTaskCreate rc=%ld", (long)rc));
}

void matter_report_door_state(const door_state_msg_t* msg) {
    if (!g_matter_vm) return;

    xSemaphoreTakeRecursive(g_matter_vm_lock, portMAX_DELAY);

    /* Map Sesame direction (DCM_DOOR_DIR_*) to Tasmota direction (-1, 0, 1)
     * DCM_DOOR_DIR_UP (Opening) -> 1
     * DCM_DOOR_DIR_DOWN (Closing) -> -1
     * DCM_DOOR_DIR_STOPPED -> 0
     */
    int tas_dir = 0;
    if (msg->direction == DCM_DOOR_DIR_UP) {
        tas_dir = 1;
    } else if (msg->direction == DCM_DOOR_DIR_DOWN) {
        tas_dir = -1;
    }

    /* Update the Matter plugin shadow state and trigger attribute reports.
     * Plugin "2" is our sesame_door. parse_sensors updates shadow_shutter_pos
     * and shadow_shutter_direction, and calls attribute_updated() which
     * triggers the subscription engine. */
    char code[160];
    snprintf(
        code, sizeof(code),
        "if global.matter_device\n"
        "  var p = matter_device.find_plugin_by_endpoint(2)\n"
        "  if p p.parse_sensors({'Shutter1':{'Position':%d,'Direction':%d}}) "
        "end\n"
        "end",
        (int)msg->pos, tas_dir);

    if (be_dostring(g_matter_vm, code) != 0) {
        LogError(("[matter] failed to report door state: %s",
                  be_tostring(g_matter_vm, -1)));
        be_pop(g_matter_vm, 1);
    }

    xSemaphoreGiveRecursive(g_matter_vm_lock);
}

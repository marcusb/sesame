#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "fsl_debug_console.h"
#undef LOG_INFO
#undef LOG_ERROR
#define LOG_INFO(fmt, ...) LogInfo((fmt, ##__VA_ARGS__))
#define LOG_ERROR(fmt, ...) LogError((fmt, ##__VA_ARGS__))
#include "be_mapping.h"
#include "be_vm.h"
#include "berry.h"
#include "embedded_be.h"
#include "matter_mdns.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

extern void matter_tasmota_tick(bvm* vm);
extern void matter_tasmota_notify_network_up(bvm* vm);

/* Shared between the matter task and (host-test) cmd handlers. */
bvm* g_matter_vm;
SemaphoreHandle_t g_matter_vm_lock;
static StaticSemaphore_t g_matter_vm_lock_buf;

/* Berry bootstrap: defines the door plugin, registers it, instantiates
 * matter.Device with a hard-coded plugin config, and starts it. We bypass
 * autoconf_device_map (which queries Tasmota-specific light/shutter status
 * via tasmota.cmd) by marking plugins_persist=true with a pre-filled
 * plugins_config — instantiate_plugins_from_config then pushes the plugins
 * without calling autoconf_device_map. */
static const char matter_bootstrap[] =
    "import matter\n"
    "import sesame\n"
    "class Matter_Door_Plugin : matter.Plugin_Shutter\n"
    "  static var TYPE = \"sesame_door\"\n"
    "  static var DISPLAY_NAME = \"Sesame Door\"\n"
    "  def init(device, endpoint, conf)\n"
    "    super(self).init(device, endpoint, conf)\n"
    "    self.shadow_shutter_inverted = 1\n"
    "  end\n"
    "  def update_shadow()\n"
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
    "end\n"
    /* Matter_Plugin_z_All.be's top-level code (which would populate
     * matter.plugins_classes on Tasmota) is not included in the sesame
     * build, so seed the map ourselves with just the classes we need. */
    "matter.plugins_classes = {\"root\": matter.Plugin_Root,"
    "\"aggregator\": matter.Plugin_Aggregator,"
    "\"shutter\": matter.Plugin_Shutter,"
    "\"sesame_door\": Matter_Door_Plugin}\n"
    "matter_device = matter.Device()\n"
    "matter_device.plugins_persist = true\n"
    "matter_device.plugins_config = {\"0\":{\"type\":\"root\"},"
    "\"2\":{\"type\":\"sesame_door\"}}\n"
    "matter_device.start()\n";

static void matter_task(void* pvParameters) {
    (void)pvParameters;
    PRINTF("[matter] entry\r\n");

    PRINTF("[matter] be_vm_new\r\n");
    bvm* vm = be_vm_new();
    if (!vm) {
        PRINTF("[matter] failed to create berry vm\r\n");
        vTaskDelete(NULL);
        return;
    }
    g_matter_vm = vm;
    PRINTF("[matter] vm=%p\r\n", vm);

    /* Solidified matter module exposes `be_const_ctype_func(...)` entries
     * (e.g. get_cluster_name, is_attribute_reportable); without a registered
     * handler the VM errors with "missing ctype_func handler" the first time
     * any of them is invoked (observed during Matter_Device.init() calling
     * commissioning start paths). */
    be_set_ctype_func_handler(vm, be_call_ctype_func);

    PRINTF("[matter] load crypto\r\n");
    extern int be_load_crypto_module(bvm * vm);
    be_load_crypto_module(vm);

    /* Define crypto.SPAKE2P_Matter (Berry source — solidify pipeline isn't
     * working yet). Must run after the crypto module exists. */
    if (be_dostring(vm, embedded_be_crypto_spake2p_matter) != 0) {
        PRINTF("[matter] failed to define SPAKE2P_Matter: %s\r\n",
               be_tostring(vm, -1));
        be_pop(vm, 1);
    } else if (be_dostring(vm,
                           "import crypto\n"
                           "crypto.SPAKE2P_Matter = SPAKE2P_Matter\n") != 0) {
        PRINTF("[matter] failed to attach SPAKE2P_Matter: %s\r\n",
               be_tostring(vm, -1));
        be_pop(vm, 1);
    }

    PRINTF("[matter] init globals\r\n");
    if (be_dostring(vm,
                    "_matter_fast_cbs = []\n"
                    "_matter_fast_cbs_once = []\n"
                    "_matter_net_cbs = []\n"
                    "_matter_drivers = []\n") != 0) {
        PRINTF("[matter] failed to init matter globals: %s\r\n",
               be_tostring(vm, -1));
    }

    PRINTF("[matter] import tasmota\r\n");
    if (be_dostring(vm, "import tasmota\nreturn tasmota") == 0) {
        be_setglobal(vm, "tasmota");
    } else {
        PRINTF("[matter] failed to load tasmota globally: %s\r\n",
               be_tostring(vm, -1));
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
        "  def update_shadow() end\n"
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
        "  _s = 'start_mdns'\n"
        "  matter_device.commissioning.start_mdns_announce_hostnames()\n"
        "  print('[matter] start_mdns ok')\n"
        "except .. as e, m\n"
        "  print('[matter] start fail at', _s, ':', e, '|', m)\n"
        "end",
        NULL};
    for (int i = 0; steps[i]; i++) {
        if (be_dostring(vm, steps[i]) != 0) {
            PRINTF("[matter] bootstrap step %d failed: %s\r\n", i,
                   be_tostring(vm, -1));
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
    BaseType_t rc = xTaskCreate(matter_task, "matter", 8192, NULL,
                                tskIDLE_PRIORITY + 1, NULL);
    LogInfo(("matter_init: xTaskCreate rc=%ld", (long)rc));
}

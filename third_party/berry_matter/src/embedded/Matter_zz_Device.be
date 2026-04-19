# import matter

#@ solidify:Matter_Device,weak

class Matter_Device
  static var UDP_PORT = 5540          # this is the default port for group multicast, we also use it for unicast
  static var VENDOR_ID = 0xFFF1
  static var PRODUCT_ID = 0x8000
  static var FILENAME = "_matter_device.json"
  static var EP = 2                   # first available endpoint number for devices
  var started                         # is the Matter Device started (configured, mDNS and UDPServer started) - 'nil' means that we wait for Wifi to connect, 'false' means that the start is scheduled but not yet triggered
  var plugins                         # list of plugins instances
  var plugins_persist                 # true if plugins configuration needs to be saved
  static var plugins_classes          # map of registered classes by type name
  var plugins_config                  # map of JSON configuration for plugins
  var udp_server                      # `matter.UDPServer()` object
  var profiler
  var message_handler                 # `matter.MessageHandler()` object
  var commissioning                   # `matter.Commissioning()` object
  var autoconf                        # `matter.Autoconf()` objects
  var sessions                        # `matter.Session_Store()` objet
  var ui
  var tick                            # increment at each tick, avoids to repeat too frequently some actions
  # Events
  var events                          # Event handler
  # saved in parameters
  var root_discriminator              # as `int`
  var root_passcode                   # as `int`
  var ipv4only                        # advertize only IPv4 addresses (no IPv6)
  var disable_bridge_mode             # default is bridge mode, this flag disables this mode for some non-compliant controllers
  var next_ep                         # next endpoint to be allocated for bridge, start at 1
  var debug                           # debug mode, output all values when responding to read request with wildcard

  #############################################################
  def init()
    if type(self.plugins_classes) == 'nil'
      self.plugins_classes = matter.plugins_classes
    end
    import crypto
    matter.profiler = matter.Profiler()
    self.tick = 0
    self.plugins = []
    self.plugins_persist = false                  # plugins need to saved only when the first fabric is associated
    self.next_ep = self.EP                        # start at endpoint 2 for dynamically allocated endpoints (1 reserved for aggregator)
    self.ipv4only = false
    self.disable_bridge_mode = false
    self.commissioning = matter.Commissioning(self)
    self.load_param()

    self.sessions = matter.Session_Store(self)
    self.sessions.load_fabrics()
    self.message_handler = matter.MessageHandler(self)
    self.events = matter.EventHandler(self)

    # tasmota.when_network_up(def () self.start() end)    # start when network is connected
    self.commissioning.init_basic_commissioning()
  end

  #############################################################
  # Start Matter device server when the first network is coming up
  def start()
    # autoconfigure other plugins if needed
    self.autoconf_device()
    self._start_udp(self.UDP_PORT)
    self.commissioning.start_mdns_announce_hostnames()
  end

  #####################################################################
  # Remove a fabric and clean all corresponding values and mDNS entries
  def remove_fabric(fabric)
    if fabric != nil
      self.message_handler.im.subs_shop.remove_by_fabric(fabric)
      self.commissioning.mdns_remove_op_discovery(fabric)
      self.sessions.remove_fabric(fabric)
    end
    self.sessions.save_fabrics()
  end

  #############################################################
  # dispatch every second click to sub-objects that need it
  def every_second()
    self.sessions.every_second()
    self.message_handler.every_second()
    self.events.every_second()      # periodically remove bytes() representation of events
    self.commissioning.every_second()
  end

  #############################################################
  # dispatch every 250ms to all plugins
  def every_250ms()
    var idx = 0
    while idx < size(self.plugins)
      self.plugins[idx].every_250ms()
      idx += 1
    end
  end

  #############################################################
  # dispatch every 50ms
  # ticks
  def every_50ms()
    self.tick += 1
    self.message_handler.every_50ms()
  end

  #############################################################
  def stop()
    if self.udp_server    self.udp_server.stop() end
  end

  #############################################################
  # Called on 'network_down' event, before WiFi teardown.
  # Flush the UDP socket to discard any queued packets whose
  # pbufs reference the WiFi netif (about to be destroyed).
  def network_down()
    if self.udp_server
      self.udp_server.flush_socket()
    end
  end

  #############################################################
  # Called on 'network_up' event, after WiFi teardown when
  # Ethernet is still active (or when WiFi reconnects).
  # Reopen the UDP socket with a clean receive buffer.
  def network_up()
    if self.udp_server
      self.udp_server.reopen_socket()
    end
  end

  #############################################################
  # Callback when message is received.
  # Send to `message_handler`
  def msg_received(raw, addr, port)
    return self.message_handler.msg_received(raw, addr, port)
  end

  #############################################################
  # Global entry point for sending a message.
  # Delegates to `udp_server`
  def msg_send(msg)
    return self.udp_server.send_UDP(msg)
  end

  #############################################################
  # Signals that a ack was received.
  # Delegates to `udp_server` to remove from resending list.
  def received_ack(msg)
    return self.udp_server.received_ack(msg)
  end

  #############################################################
  # (internal) Start UDP Server
  def _start_udp(port)
    if self.udp_server    return end        # already started
    if port == nil      port = 5540 end
    self.udp_server = matter.UDPServer(self, "", port)
    self.udp_server.start(/ raw, addr, port -> self.msg_received(raw, addr, port))
  end


  #################################################################################
  # Simple insertion sort - sorts the list in place, and returns the list
  # remove duplicates
  #################################################################################
  static def sort_distinct(l)
    # insertion sort
    for i:1..size(l)-1
      var k = l[i]
      var j = i
      while (j > 0) && (l[j-1] > k)
        l[j] = l[j-1]
        j -= 1
      end
      l[j] = k
    end
    # remove duplicate now that it's sorted
    var i = 1
    if size(l) <= 1  return l end     # no duplicate if empty or 1 element
    var prev = l[0]
    while i < size(l)
      if l[i] == prev
        l.remove(i)
      else
        prev = l[i]
        i += 1
      end
    end
    return l
  end

  #############################################################
  # Signal that an attribute has been changed and propagate
  # to any active subscription.
  #
  # Delegates to `message_handler`
  def attribute_updated(endpoint, cluster, attribute, fabric_specific)
    if fabric_specific == nil   fabric_specific = false end
    var ctx = matter.Path()
    ctx.endpoint = endpoint
    ctx.cluster = cluster
    ctx.attribute = attribute
    self.message_handler.im.subs_shop.attribute_updated_ctx(ctx, fabric_specific)
  end

  #############################################################
  # Proceed to attribute expansion (used for Attribute Read/Write/Subscribe)
  #
  # calls `cb(pi, ctx, direct)` for each attribute expanded.
  # `pi`: plugin instance targeted by the attribute (via endpoint). Note: nothing is sent if the attribute is not declared in supported attributes in plugin.
  # `ctx`: context object with `endpoint`, `cluster`, `attribute` (no `command`)
  # `direct`: `true` if the attribute is directly targeted, `false` if listed as part of a wildcard
  # returns: `true` if processed succesfully, `false` if error occured. If `direct`, the error is returned to caller, but if expanded the error is silently ignored and the attribute skipped.
  # In case of `direct` but the endpoint/cluster/attribute is not suppported, it calls `cb(nil, ctx, true)` so you have a chance to encode the exact error (UNSUPPORTED_ENDPOINT/UNSUPPORTED_CLUSTER/UNSUPPORTED_ATTRIBUTE/UNREPORTABLE_ATTRIBUTE)
  def process_attribute_expansion(ctx, cb)
    var endpoint = ctx.endpoint
    var cluster = ctx.cluster
    var attribute = ctx.attribute

    # build the generator for all endpoint/cluster/attributes candidates
    var path_generator = matter.PathGenerator(self)
    path_generator.start(endpoint, cluster, attribute)

    var direct = path_generator.is_direct()
    var concrete_path
    while ((concrete_path := path_generator.next_attribute()) != nil)
      var finished = cb(path_generator.get_pi(), concrete_path)   # call the callback with the plugin and the context
    end
  end

  #############################################################
  # Optimized version for a single endpoint/cluster/attribute
  #
  # Retrieve the plugin for a read, or nil if not found
  # In case of error, ctx.status is updated accordingly
  def resolve_attribute_read_solo(ctx)
    var endpoint = ctx.endpoint
    var cluster = ctx.cluster
    var attribute = ctx.attribute

    # all 3 elements must be non-nil
    if (endpoint == nil) || (cluster == nil) || (attribute == nil)      return nil    end

    # look for plugin
    var pi = self.find_plugin_by_endpoint(endpoint)
    if (pi == nil)
      ctx.status = 0x7F #-matter.UNSUPPORTED_ENDPOINT-#
      return nil
    else
      if   !pi.contains_cluster(cluster)
        ctx.status = 0xC3 #-matter.UNSUPPORTED_CLUSTER-#
        return nil
      elif !pi.contains_attribute(cluster, attribute)
        ctx.status = 0x86 #-matter.UNSUPPORTED_ATTRIBUTE-#
        return nil
      end
    end

    # all good
    return pi
  end

  #############################################################
  # Return the list of endpoints from all plugins (distinct), exclud endpoint zero if `exclude_zero` is `true`
  def get_active_endpoints(exclude_zero)
    var ret = []
    for p:self.plugins
      var ep = p.get_endpoint()
      if exclude_zero && ep == 0   continue end
      if ret.find(ep) == nil
        ret.push(ep)
      end
    end
    return ret
  end

  #############################################################
  # Find plugin by endpoint
  def find_plugin_by_endpoint(ep)
    var idx = 0
    while idx < size(self.plugins)
      var pl = self.plugins[idx]
      if pl.get_endpoint() == ep
        return pl
      end
      idx += 1
    end
    return nil
  end

  #############################################################
  # Find plugin by endpoint
  def find_plugin_by_friendly_name(name)
    if (name == nil) || (size(name) == 0)   return nil      end     # invalid name

    var idx = 0
    while idx < size(self.plugins)
      var pl = self.plugins[idx]
      var pl_name = pl.get_name()
      if (pl_name != nil) && (size(pl_name) > 0) && (pl_name == name)
        return pl
      end
      idx += 1
    end
    return nil
  end

  #############################################################
  # Persistance of Matter Device parameters
  #
  #############################################################
  # 
  def save_param()
    import json

    var j = format('{"distinguish":%i,"passcode":%i,"ipv4only":%s,"disable_bridge_mode":%s,"nextep":%i', self.root_discriminator, self.root_passcode, self.ipv4only ? 'true':'false', self.disable_bridge_mode ? 'true':'false', self.next_ep)
    if self.debug
      j += ',"debug":true'
    end
    if self.plugins_persist
      j += ',\n"config":'
      j += json.dump(self.plugins_config)
    end
    j += '}'
    try
      var f = open(self.FILENAME, "w")
      f.write(j)
      f.close()
      return j
    except .. as e, m
      return j
    end
  end
    
  #############################################################
  # Reset configuration like a fresh new device
  def reset_param()
    self.plugins_persist = false
    self.next_ep = self.EP
    self.save_param()
  end

  #############################################################
  # Load Matter Device parameters
  def load_param()
    import crypto
    var dirty = false
    try

      var f = open(self.FILENAME)
      var s = f.read()
      f.close()
      import json
      var j = json.load(s)

      self.root_discriminator = j.find("distinguish", self.root_discriminator)
      self.root_passcode = j.find("passcode", self.root_passcode)
      self.ipv4only = bool(j.find("ipv4only", false))
      self.disable_bridge_mode = bool(j.find("disable_bridge_mode", false))
      self.next_ep = j.find("nextep", self.next_ep)
      self.plugins_config = j.find("config", {})
      self.debug = bool(j.find("debug"))    # bool converts nil to false
      if self.plugins_config != nil
        self.adjust_next_ep()
        dirty = self.check_config_ep()
        self.plugins_persist = true
      end
    except .. as e, m
    end

    if self.root_discriminator == nil
      self.root_discriminator = crypto.random(2).get(0,2) & 0xFFF
      dirty = true
    end
    if self.root_passcode == nil
      self.root_passcode = self.commissioning.generate_random_passcode()
      dirty = true
    end
    if dirty    self.save_param() end
  end

  #############################################################
  # Convert a configuration to a log string
  static def conf_to_log(plugin_conf)
    var param_log = ''
    for k:_class.k2l(plugin_conf)
      if k == 'type'  continue  end
      param_log += format(" %s:%s", k, plugin_conf[k])
    end
    return param_log
  end

  #############################################################
  # Matter plugin management
  #
  # Plugins allow to specify response to read/write attributes
  # and command invokes
  #############################################################
  def invoke_request(session, val, ctx)
    var idx = 0
    var endpoint = ctx.endpoint
    while idx < size(self.plugins)
      var plugin = self.plugins[idx]

      if plugin.endpoint == endpoint
        return plugin.invoke_request(session, val, ctx)
      end

      idx += 1
    end
    ctx.status = 0x7F #-matter.UNSUPPORTED_ENDPOINT-#
  end

  #############################################################
  # Try to clean MDNS entries before restart.
  #
  # Called by Tasmota loop as a Tasmota driver.
  def save_before_restart()
    self.commissioning.stop_basic_commissioning()
    self.commissioning.mdns_remove_op_discovery_all_fabrics()
  end

  #############################################################
  # Autoconfigure device from template
  #
  # Applies only if there are no plugins already configured
  ## TODO generate map instead
  def autoconf_device()
    import json

    if size(self.plugins) > 0   return end                    # already configured
    if (self.autoconf == nil)   self.autoconf = matter.Autoconf(self)   end

    if !self.plugins_persist
      self.plugins_config = self.autoconf.autoconf_device_map()
      self.adjust_next_ep()
    end
    self.autoconf.instantiate_plugins_from_config(self.plugins_config)

    if !self.plugins_persist && self.sessions.count_active_fabrics() > 0
      self.plugins_persist = true
      self.save_param()
    end
  end

  # get keys of a map in sorted order
  static def k2l(m) var l=[] if m==nil return l end for k:m.keys() l.push(k) end
    for i:1..size(l)-1 var k = l[i] var j = i while (j > 0) && (l[j-1] > k) l[j] = l[j-1] j -= 1 end l[j] = k end return l
  end

  # get keys of a map in sorted order, as numbers
  static def k2l_num(m) var l=[] if m==nil return l end for k:m.keys() l.push(int(k)) end
    for i:1..size(l)-1 var k = l[i] var j = i while (j > 0) && (l[j-1] > k) l[j] = l[j-1] j -= 1 end l[j] = k end return l
  end

  #############################################################
  # get_plugin_class_displayname
  #
  # get a class name light "light0" and return displayname
  def get_plugin_class_displayname(name)
    var cl = self.plugins_classes.find(name)
    return cl ? cl.DISPLAY_NAME : ""
  end

  #############################################################
  # Dynamic adding and removal of endpoints (bridge mode)
  #############################################################
  # Add endpoint
  #
  # Args:
  # `pi_class_name`: name of the type of pluging, ex: `light3`
  # `plugin_conf`: map of configuration as native Berry map
  # returns endpoint number newly allocated, or `nil` if failed
  def bridge_add_endpoint(pi_class_name, plugin_conf)
    var pi_class = self.plugins_classes.find(pi_class_name)
    if pi_class == nil    return  end

    # get the next allocated endpoint number
    var ep = self.next_ep
    var ep_str = str(ep)

    var pi = pi_class(self, ep, plugin_conf)
    self.plugins.push(pi)

    # add to in-memoru config
    # Example: {'filter': 'AXP192#Temperature', 'type': 'temperature'}
    var pi_conf = {'type': pi_class_name}
    # copy args
    for k:plugin_conf.keys()
      pi_conf[k] = plugin_conf[k]
    end
    # add to main
    self.plugins_config[ep_str] = pi_conf
    self.plugins_persist = true
    self.next_ep += 1     # increment next allocated endpoint before saving

    # try saving parameters
    self.save_param()
    self.signal_endpoints_changed()

    return ep
  end

  #############################################################
  # Remove an existing endpoint
  #
  def bridge_remove_endpoint(ep)
    import json

    var ep_str = str(ep)
    var config
    var f_in

    if !self.plugins_config.contains(ep_str)
      return
    end
    self.plugins_config.remove(ep_str)
    self.plugins_persist = true

    # now remove from in-memory configuration
    var idx = 0
    while idx < size(self.plugins)
      if ep == self.plugins[idx].get_endpoint()
        self.plugins.remove(idx)
        break
      else
        idx += 1
      end
    end

    # try saving parameters
    self.save_param()
    self.signal_endpoints_changed()
  end

  #############################################################
  # Signal to controller that endpoints changed via subcriptions
  #
  def signal_endpoints_changed()
    # mark parts lists as changed
    self.attribute_updated(0x0000, 0x001D, 0x0003, false)
    self.attribute_updated(0x0001 #-matter.AGGREGATOR_ENDPOINT-#, 0x001D, 0x0003, false)
  end

  #############################################################
  # Check that all ep are valid, i.e. don't collied with root or aggregator
  #
  # return `true` if configuration was adjusted and needs to be saved
  def check_config_ep()
    # copy into list so we can change the map on the fly
    var dirty = false
    var keys = []
    for k: self.plugins_config.keys()   keys.push(int(k))    end
    for ep: keys
      if ep == 0x0001 #-matter.AGGREGATOR_ENDPOINT-#
        dirty = true
        self.plugins_config[str(self.next_ep)] = self.plugins_config[str(ep)]
        self.plugins_config.remove(str(ep))
        self.next_ep += 1
      end
    end
    return dirty
  end

  #############################################################
  # Adjust next_ep
  #
  # Make sure that next_ep (used to allow dynamic endpoints)
  # will not collide with an existing ep
  def adjust_next_ep()
    for k: self.plugins_config.keys()
      var ep = int(k)
      if ep >= self.next_ep
        self.next_ep = ep + 1
      end
    end
  end

  #####################################################################
  # Events
  #####################################################################
  def event_fabrics_saved()
    # if the plugins configuration was not persisted and a new fabric is saved, persist it
    if self.sessions.count_active_fabrics() > 0 && !self.plugins_persist
      self.plugins_persist = true
      self.save_param()
    end
  end

end

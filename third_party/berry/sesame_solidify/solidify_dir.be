#
# solidify_dir.be — compile every .be file in an input directory into a
# solidified_*.h header under an output directory.
#
# Usage:
#   berry_host solidify_dir.be <input_dir> <output_dir>
#
# Each .be is `compile()`d, registering its classes/modules/functions in the
# host VM. The directives `#@ solidify:Name[,weak]` inside the source tell us
# which top-level objects to dump.
#
# Stable invocation contract:
#   * Output dir is created if missing; existing files are left alone unless
#     this run overwrites them. This script never wipes the directory — if
#     a run fails halfway, the previous good headers remain readable for
#     incremental rebuilds (the build system, not the script, decides when
#     to invalidate the cache).
#   * Any error (compile failure, missing global, IO error) aborts with a
#     non-zero exit so the build fails loudly.
#

import os
import global
import solidify
import string
import re
import sys

# Tasmota-style globals that the .be sources reference at module scope. The
# compiler refuses to parse an unqualified `log(...)` (or similar) unless the
# identifier is already a known global, so we pre-create them as nil. Their
# real implementations only matter at runtime, which the device firmware
# provides — solidify only needs to introspect the compiled bytecode.
var globs = [
  # Tasmota framework globals
  "tasmota", "gpio", "light", "light_state", "udp", "webclient", "webserver",
  "tcpclient", "tcpclientasync", "tcpserver", "mqtt", "mdns", "persist",
  "crypto", "json", "energy", "MD5", "ccronexpr", "ctypes_bytes",
  "ctypes_bytes_dyn", "path", "load", "lv", "_lvgl", "int64",
  # Tasmota log/format helpers
  "log", "format", "print", "assert",
  # LVGL bits referenced in plugin code
  "lv_clock", "lv_clock_icon", "lv_signal_arcs", "lv_signal_bars",
  "lv_wifi_arcs_icon", "lv_wifi_arcs", "lv_wifi_bars_icon", "lv_wifi_bars",
]
for g : globs
  global.(g) = nil
end

if size(_argv) < 3
  print("usage: solidify_dir.be <input_dir> <output_dir>")
  raise "usage_error"
end

var input_dir = _argv[1]
var output_dir = _argv[2]
if input_dir[-1] != '/'
  input_dir = input_dir + '/'
end
if output_dir[-1] != '/'
  output_dir = output_dir + '/'
end

# Ensure output dir exists. os.mkdir errors if the directory already exists,
# so guard with os.path.exists.
if !os.path.exists(output_dir)
  os.mkdir(output_dir)
end

# .be sources use `import matter` (matter.be is in the same dir); push the
# input dir onto sys.path so the import resolves regardless of cwd.
sys.path().push(input_dir)

def insertion_sort(l)
  for i:1..size(l)-1
    var k = l[i]
    var j = i
    while (j > 0) && (l[j-1] > k)
      l[j] = l[j-1]
      j -= 1
    end
    l[j] = k
  end
  return l
end

var directive_pat = "#@\\s*solidify:([A-Za-z0-9_.,]+)"

def solidify_file(fname, input_dir, output_dir)
  print("solidify: ", fname)
  var f = open(input_dir + fname)
  var src = f.read()
  f.close()
  var compiled = compile(src)
  compiled()

  var stem = string.split(fname, '.be')[0]
  var out_path = output_dir + "solidified_" + stem + ".h"
  var fout = open(out_path, "w")
  fout.write(format("/* Solidification of %s.h */\n", stem))
  fout.write("/********************************************************************\\\n")
  fout.write("* Generated code, don't edit                                         *\n")
  fout.write("\\********************************************************************/\n")
  fout.write("#include \"be_constobj.h\"\n")

  var directives = re.searchall(directive_pat, src)
  for directive : directives
    var parts = string.split(directive[1], ',')
    var object_name = parts[0]
    var weak = false
    for p : parts
      if p == "weak"
        weak = true
      end
    end
    var o = global
    var cl_name = nil
    var obj_name = nil
    for subname : string.split(object_name, '.')
      o = o.(subname)
      cl_name = obj_name
      obj_name = subname
      if   type(o) == 'class'
        obj_name = 'class_' + obj_name
      elif type(o) == 'module'
        obj_name = 'module_' + obj_name
      end
    end
    solidify.dump(o, weak, fout, cl_name)
  end

  fout.write("/********************************************************************/\n")
  fout.write("/* End of solidification */\n")
  fout.close()
end

var files = os.listdir(input_dir)
files = insertion_sort(files)
for fname : files
  if fname[0] == '.'  continue end
  if !string.endswith(fname, '.be')  continue end
  # Skip *_test.be — those files run smoke tests at module scope that
  # depend on real implementations of matter.TLV etc. Solidifying them
  # would require a working VM, not just enough state to register the
  # bytecode.
  if string.find(fname, '_test.be') >= 0  continue end
  solidify_file(fname, input_dir, output_dir)
end

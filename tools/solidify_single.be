import sys
import global
import solidify
import string

def solidify_file(input_file, output_file)
    print(string.format("Solidifying: %s -> %s", input_file, output_file))
    
    # Read input
    var f = open(input_file, "r")
    var src = f.read()
    f.close()

    # Pre-populate globals
    var globs = "path,ctypes_bytes_dyn,tasmota,ccronexpr,gpio,light,webclient,load,MD5,lv,light_state,udp,tcpclientasync,lv_clock,lv_clock_icon,lv_signal_arcs,lv_signal_bars,lv_wifi_arcs_icon,lv_wifi_arcs,lv_wifi_bars_icon,lv_wifi_bars,_lvgl,int64,crypto,json,webserver"
    for g : string.split(globs, ",")
        global.(g) = nil
    end

    var matter_mod = module("matter")
    matter_mod.plugins_classes = {}
    global.matter = matter_mod

    # Compile source
    var compiled = compile(src)
    compiled() # Execute to register modules/classes/functions
    
    # Open output
    var fout = open(output_file, "w")
    fout.write(string.format("/* Solidified %s */\n", input_file))
    fout.write("#include \"be_constobj.h\"\n\n")
    
    # Search for solidify directives manually without regex
    var search_str = "#@ solidify:"
    var pos = 0
    while true
        pos = string.find(src, search_str, pos)
        if pos < 0
            break
        end
        pos += size(search_str)
        
        # Find end of line or space
        var end_pos = pos
        while end_pos < size(src)
            var c = src[end_pos]
            if c == '\n' || c == '\r' || c == ' ' || c == '\t' || c == ','
                break
            end
            end_pos += 1
        end
        
        var object_name = src[pos .. end_pos-1]
        
        var parts = string.split(object_name, ",")
        if size(parts) > 0
            object_name = parts[0]
        end
        parts = string.split(object_name, ":")
        if size(parts) > 1
            object_name = parts[1]
        end

        pos = end_pos
        
        var o = global
        var cl_name = nil
        
        # Traverse global objects (e.g. module.class)
        for subname : string.split(object_name, ".")
            cl_name = object_name
            o = o.(subname)
        end
        
        if o != nil
            solidify.dump(o, true, fout, nil)
        else
            print(string.format("Warning: Object %s not found in global scope", object_name))
        end
    end
    
    fout.close()
end

# Main entry point
if size(_argv) >= 3
    solidify_file(_argv[1], _argv[2])
else
    print("Usage: berry solidify_single.be <input.be> <output.h>")
end

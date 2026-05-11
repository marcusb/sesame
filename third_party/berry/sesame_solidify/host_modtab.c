/*
 * host_modtab.c — module table used by the host `berry_host` solidifier.
 *
 * This deliberately does NOT alias `tasmota`, `matter`, `crypto`, `_class`
 * etc. to the `sys` native module. Those aliases (used by the upstream
 * berry default host build) make `import matter` return the (shared,
 * effectively read-only) sys module, which breaks solidify_all.be: it
 * needs to assign attributes onto a freshly-created module.
 *
 * Leaving those names unregistered makes `import <name>` fall through to
 * Berry-level module loading, which yields a fresh mutable module object
 * that solidify can introspect.
 */

#include "berry.h"

be_extern_native_module(string);
be_extern_native_module(json);
be_extern_native_module(math);
be_extern_native_module(time);
be_extern_native_module(os);
be_extern_native_module(global);
be_extern_native_module(sys);
be_extern_native_module(debug);
be_extern_native_module(gc);
be_extern_native_module(solidify);
be_extern_native_module(introspect);
be_extern_native_module(strict);
be_extern_native_module(undefined);
be_extern_native_module(re);

const bntvmodule_t* const be_module_table[] = {
    &be_native_module(string),
    &be_native_module(json),
    &be_native_module(math),
    &be_native_module(time),
    &be_native_module(os),
    &be_native_module(global),
    &be_native_module(sys),
    &be_native_module(debug),
    &be_native_module(gc),
    &be_native_module(solidify),
    &be_native_module(introspect),
    &be_native_module(strict),
    &be_native_module(undefined),
    &be_native_module(re),
    NULL,
};

const bclass_ptr be_class_table[] = {
    NULL,
};

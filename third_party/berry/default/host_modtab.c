#include "berry.h"

/* default modules declare */
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

#define be_native_module_tasmota be_native_module_sys
#define be_native_module_crypto be_native_module_sys
#define be_native_module_matter be_native_module_sys
#define be_native_module__class be_native_module_sys

/* list of native modules */
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
    &be_native_module(tasmota),
    &be_native_module(crypto),
    &be_native_module(matter),
    &be_native_module(_class),
    NULL /* list terminator */
};

/* list of native classes */
const bclass_ptr be_class_table[] = {
    NULL /* list terminator */
};
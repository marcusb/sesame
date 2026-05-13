/*
 * Minimal endpoint_config.h for Sesame.
 *
 * All endpoints (root node + window covering) are registered dynamically at
 * runtime via emberAfSetDynamicEndpoint() in matter_app.cpp.  This file only
 * satisfies the compile-time requirements of attribute-storage.cpp.
 */
#pragma once

#include <app/util/endpoint-config-defines.h>

/* No static (fixed) endpoints; all registered dynamically. */
#define FIXED_ENDPOINT_COUNT 0

/* Max size of any single attribute in bytes.  Used to size stack buffers in
 * attribute-storage.cpp.  256 covers all standard Matter attribute types. */
#define ATTRIBUTE_LARGEST 256

/* Empty attribute metadata array required by attribute-storage.cpp. */
#define GENERATED_ATTRIBUTES \
    {                        \
    }

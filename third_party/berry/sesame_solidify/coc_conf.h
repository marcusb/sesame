/*
 * Overrides scanned by `coc` (the const-table generator) when building the
 * host-side `berry_host` solidifier.
 *
 * coc walks every `#define` it sees in its -c files and merges them into a
 * single macro map (last-write-wins). Passing this file *after* the standard
 * default/berry_conf.h flips the modules the host needs back on, without
 * affecting the on-device cross-compile (which doesn't reference this file).
 */
#define BE_USE_SOLIDIFY_MODULE          1
#define BE_USE_OS_MODULE                1
#define BE_USE_RE_MODULE                1
#define BE_USE_INTROSPECT_MODULE        1
#define BE_USE_STRICT_MODULE            1
#define BE_USE_DEBUG_MODULE             1
#define BE_USE_GC_MODULE                1
#define BE_USE_TIME_MODULE              1
#define BE_USE_FILE_SYSTEM              1
#define BE_USE_SCRIPT_COMPILER          1
#define BE_USE_BYTECODE_SAVER           1
#define BE_USE_BYTECODE_LOADER          1

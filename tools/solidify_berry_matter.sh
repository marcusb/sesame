#!/usr/bin/env bash
# Build a host Berry compiler and regenerate third_party/berry_matter/src/solidify/
# from the *.be sources in src/embedded/.
#
# WORK IN PROGRESS: the solidify_all.be harness expects a Tasmota-style global
# environment (tasmota, gpio, light, mqtt, etc. pre-defined) that we don't have
# on the host. Running this currently errors on the first .be file, after which
# solidify_all.be has already emptied src/solidify/ via its clean_directory()
# step. Do NOT invoke until the harness is made tolerant of missing globals or
# a staged output directory is wired in. See the "pipeline follow-up" notes.
set -euo pipefail
echo "WIP: solidify harness not ready; see comment in $0" >&2
exit 1

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

BERRY_SRC="$REPO/third_party/berry"
BERRY_MATTER="$REPO/third_party/berry_matter"
BUILD_DIR="$REPO/build-native/berry-host"

mkdir -p "$BUILD_DIR"

# Mirror the berry tree into BUILD_DIR, replacing sesame-specific files that
# collide with the upstream host build. We don't mutate the source tree.
rsync -a --delete \
    --exclude='*.o' --exclude='*.d' --exclude='berry' \
    "$BERRY_SRC/" "$BUILD_DIR/berry/"
rsync -a "$REPO/third_party/re1.5/"          "$BUILD_DIR/re1.5/"
rsync -a "$REPO/third_party/berry_mapping/"  "$BUILD_DIR/berry_mapping/"
rsync -a "$REPO/third_party/berry_int64/"    "$BUILD_DIR/berry_int64/"

# Use the vanilla (multi-module) berry_conf.h for the host build.
cp "$BUILD_DIR/berry/default/default/berry_conf.h" "$BUILD_DIR/berry/default/berry_conf.h"

# Remove the Tasmota-heavy modtab (references mqtt, gpio, light, etc. that we
# don't have on the host); keep be_re_lib.c so the 're' module compiles.
rm -f "$BUILD_DIR/berry/default/be_modtab.c" \
      "$BUILD_DIR/berry/default/be_port.c"

# Upstream be_port.c provides the full POSIX filesystem backing for os/file.
cp "$BUILD_DIR/berry/default/default/be_port.c" "$BUILD_DIR/berry/default/be_port.c"
# Drop host_port.c to avoid duplicate be_fseek etc. (be_port.c supersedes it).
rm -f "$BUILD_DIR/berry/default/host_port.c"

# Patch host_modtab.c:
#  - register the 're' module
#  - drop the `be_native_module_matter = be_native_module_sys` style aliases,
#    so `import matter` / `import tasmota` / `import crypto` fall through to
#    Berry-level module loading (which yields mutable modules that solidify
#    needs to assign attributes on).
python3 - "$BUILD_DIR/berry/default/host_modtab.c" <<'PY'
import sys, pathlib, re as _re
p = pathlib.Path(sys.argv[1])
s = p.read_text()
for alias in ("tasmota", "crypto", "matter", "_class"):
    s = _re.sub(
        rf"^#define be_native_module_{alias} be_native_module_sys\n",
        "",
        s,
        flags=_re.MULTILINE,
    )
    s = _re.sub(
        rf"^    &be_native_module\({alias}\),\n",
        "",
        s,
        flags=_re.MULTILINE,
    )
if "be_native_module(re)" not in s:
    s = s.replace(
        "be_extern_native_module(undefined);",
        "be_extern_native_module(undefined);\nbe_extern_native_module(re);",
        1,
    )
    s = s.replace(
        "    &be_native_module(undefined),",
        "    &be_native_module(undefined),\n    &be_native_module(re),",
        1,
    )
p.write_text(s)
PY

( cd "$BUILD_DIR/berry" && make clean >/dev/null && make -j"$(nproc)" )

BERRY_BIN="$BUILD_DIR/berry/berry"
test -x "$BERRY_BIN"

# solidify_all.be expects to be run from a directory that has src/embedded/.
# Copy it next to the matter sources and invoke there.
cp "$REPO/third_party/berry_custom/solidify_all.be" "$BERRY_MATTER/solidify_all.be"
trap 'rm -f "$BERRY_MATTER/solidify_all.be"' EXIT

( cd "$BERRY_MATTER" && "$BERRY_BIN" solidify_all.be )
echo "Regenerated $BERRY_MATTER/src/solidify/"

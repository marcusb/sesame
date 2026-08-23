#!/bin/bash
set -e
export FORCE_APP_VERSION="0.2.1+0"
rm -rf build && west build --sysbuild
cp build/sesame/zephyr/zephyr.signed.confirmed.bin zephyr.signed.A.bin
./tools/OpenOCD/flashprog.py --erase 0,0x1A0000,0x170000
./tools/OpenOCD/flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin --image-0 zephyr.signed.A.bin -r

export FORCE_APP_VERSION="0.2.2+0"
rm -rf build && west build --sysbuild
cp build/sesame/zephyr/zephyr.signed.bin zephyr.signed.B.bin

export FORCE_APP_VERSION="0.2.3+0"
rm -rf build && west build --sysbuild
cp build/sesame/zephyr/zephyr.signed.bin zephyr.signed.C.bin

/* Stubs required to link the CHIP stack against Sesame's build environment.
 *
 * emberAfClusterInitCallback is defined in sesame_chip_app_callbacks.cpp
 * (compiled into chip_app inside the linker group).
 *
 * CCM stubs are in sesame_mbedtls_ccm_stubs.c (in chip_crypto, compiled
 * before mbedcrypto in the linker group).
 *
 * stderr is defined in matter_app.cpp to ensure it is always extracted from
 * the archive (matter_app_start is a strong referenced symbol in that TU).
 */

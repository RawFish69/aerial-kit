#ifndef AK_HOST_IDF_ESP_ETH_H
#define AK_HOST_IDF_ESP_ETH_H

/*
 * The Ethernet medium's declarations, empty.
 *
 * `src/arch/esp32/net.c` includes them at the top - the medium is a
 * compile-time choice and the includes are not inside either branch of it -
 * but every line that *uses* them is in the `#else` of
 * `#if CONFIG_AK_NET_WIFI`, and this stand-in is compiled as the Wi-Fi build,
 * which is the half nothing else here can run. The Ethernet half is the half
 * QEMU runs end to end, over its emulated OpenCores MAC: see
 * `docs/evidence/esp32-network.txt`.
 *
 * So these are here to be included and nothing more. A model of the Ethernet
 * MAC would be a second simulator for a path this repository already exercises
 * against the real IDF driver, and the thing that would go stale is the part
 * that is not the firmware's.
 */

#endif /* AK_HOST_IDF_ESP_ETH_H */

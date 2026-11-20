#pragma once

#if defined(AERIALKIT_DUAL_ROLE)
void aerialkitBootPreflight();
void aerialkitHardwareUndefinedFault();
// One ASCII line on native USB after setup: role, slot, GPIO4, reset reason,
// ELF hash prefix and LED driver health. ASCII never contains 0xC8, so CRSF
// host parsers resynchronise on the next frame.
void aerialkitBootReport();
// Sample FEM pins every loop and emit short-lived AK-RF counter lines for
// no-instrument bring-up. The sample is cheap; reporting is limited to the
// first 15 seconds after boot.
void aerialkitRfSample(uint32_t now);
#endif

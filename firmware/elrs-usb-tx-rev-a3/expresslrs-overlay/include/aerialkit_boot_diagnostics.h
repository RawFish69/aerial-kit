#pragma once

#if defined(AERIALKIT_DUAL_ROLE)
void aerialkitBootPreflight();
void aerialkitHardwareUndefinedFault();
// One ASCII line on native USB after setup: role, slot, GPIO4, reset reason,
// ELF hash prefix and LED driver health. ASCII never contains 0xC8, so CRSF
// host parsers resynchronise on the next frame.
void aerialkitBootReport();
#endif

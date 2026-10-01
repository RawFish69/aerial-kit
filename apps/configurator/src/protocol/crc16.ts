/**
 * CRC-16/CCITT-FALSE, the same one the firmware computes.
 *
 * Bitwise rather than table-driven on purpose: this runs over a handful of
 * bytes per frame at 50 Hz, which is nothing, and a table is one more thing
 * that can be transcribed wrongly from a polynomial nobody re-derives. The
 * golden vectors in the tests are what actually pin it to the firmware.
 */
export function crc16(data: Uint8Array): number {
  let crc = 0xffff;
  for (let i = 0; i < data.length; i++) {
    crc ^= (data[i]! & 0xff) << 8;
    for (let bit = 0; bit < 8; bit++) {
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

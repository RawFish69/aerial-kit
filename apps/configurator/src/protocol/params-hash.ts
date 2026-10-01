/**
 * `ak_params_hash`, in TypeScript.
 *
 * This is a reimplementation of a C function, which this project avoids on
 * principle — so it is worth being precise about why this one is here and what
 * keeps it honest.
 *
 * `hello` carries the hash of the board's parameter table. The demo board has
 * no C to compute it with, and reporting a made-up number would be worse than
 * reporting none: `config_hash` is what a saved configuration is filed under,
 * and a value that did not come from the algorithm would silently mis-file
 * every backup taken against a demo board. So the simulator computes it — the
 * same FNV-1a over the same bytes.
 *
 * What keeps it from becoming a second authority is that it is *checked*
 * against the first one: `firmware/tools/akproto_firmware_check.py` reads the
 * real board's table over `hello` and `param get`, recomputes this same hash in
 * Python, and compares it with the number the firmware sent. Two
 * implementations, one number, asserted on every `make test`. A change to
 * either side that the other does not follow fails there.
 *
 * The byte stream is `ak_params_serialize`'s: `name=value\n` per row, in table
 * order — **not sorted**, because the order is a property of the build and two
 * builds that register the same parameters differently are different builds.
 */

/** FNV-1a over the firmware's own serialisation of a parameter table. */
export function paramsHash(rows: readonly { name: string; value: string }[]): number {
  let hash = 0x811c9dc5;
  for (const row of rows) {
    for (const byte of new TextEncoder().encode(`${row.name}=${row.value}\n`)) {
      hash = Math.imul(hash ^ byte, 16777619) >>> 0;
    }
  }
  return hash;
}

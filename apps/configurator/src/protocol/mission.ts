/**
 * The mission's waypoints, read out of the board's own parameter table.
 *
 * **The waypoints are not in the mission opcode and this file is why that is not
 * a gap.** They are parameters — `wp0_lat`, `wp0_lon`, `wp1_lat`, … — registered
 * by `main.c` in `AK_PARAM_GROUP_NAVIGATION`, and `wp_count` says how many of
 * them are the mission. So they are already on this wire through `0x03`, they
 * are range-checked by the table that owns them (`±90` and `±180` degrees, seven
 * decimals), they appear in a backup and `save` keeps them. `MISSION`'s `start`
 * flies them; an `add` verb over there would have been a second way to write
 * them, and the two would have disagreed the first time one of them grew a bound
 * the other did not.
 *
 * **The slot count is discovered, never assumed.** This scans the table for the
 * naming convention rather than carrying a copy of `AK_NAV_WAYPOINTS`, so a
 * firmware that grows from four waypoints to six needs no change here. A table
 * with no such rows produces no slots — which is a true statement about that
 * board and renders as one.
 *
 * **A value that is not a number is `null`, never zero.** The board's reply for
 * a row is text, and `0` is a real position in the Gulf of Guinea: a row the
 * board answered with something this app cannot read must not become a waypoint
 * on the equator.
 */

/** A waypoint slot, as the board's table carries it. */
export interface WaypointSlot {
  /** The number in the row's name: `wp3_lat` is slot 3. */
  readonly index: number;
  readonly latName: string;
  readonly lonName: string;
  /** The board's latitude in degrees, or `null` when it did not answer with a
   *  number this app can read. */
  readonly lat: number | null;
  readonly lon: number | null;
  /**
   * One of the pair is in the table and the other is not.
   *
   * A slot the app cannot plot, and it is a fact about the table rather than a
   * zero: half a waypoint is not a waypoint, and drawing the missing half at
   * nought degrees would put it in the Atlantic.
   */
  readonly incomplete: boolean;
}

/** The naming convention `main.c` registers waypoints under. Matched rather
 *  than listed, so the slots are whatever the board actually has. */
export const WAYPOINT_LAT = /^wp(\d+)_lat$/;
export const WAYPOINT_LON = /^wp(\d+)_lon$/;

/**
 * The waypoint slots in a parameter table, in slot order.
 *
 * The rows are matched by name and the values come from `boardValue` — **what
 * the board last said, never what somebody has staged in a box.** A mission is
 * flown from what the board holds, so a grid drawn from an unsent edit would be
 * a picture of a request rather than of the aircraft's list. Staged edits belong
 * on the Parameters tab, where they are still marked as unsent.
 */
export function waypointSlots(
  rows: readonly { readonly name: string; readonly boardValue: string | null }[],
): readonly WaypointSlot[] {
  const lat = new Map<number, { name: string; value: number | null }>();
  const lon = new Map<number, { name: string; value: number | null }>();

  for (const row of rows) {
    const asLat = WAYPOINT_LAT.exec(row.name);
    if (asLat !== null) {
      lat.set(Number(asLat[1]), { name: row.name, value: degrees(row.boardValue) });
      continue;
    }
    const asLon = WAYPOINT_LON.exec(row.name);
    if (asLon !== null) {
      lon.set(Number(asLon[1]), { name: row.name, value: degrees(row.boardValue) });
    }
  }

  // Both halves of the union, so a table carrying only `wp2_lon` still produces
  // slot 2 — as an incomplete slot rather than as nothing at all. Dropping it
  // would report a table this app had tidied up instead of the one it read.
  const indices = [...new Set([...lat.keys(), ...lon.keys()])].sort((a, b) => a - b);

  return indices.map((index) => {
    const latitude = lat.get(index);
    const longitude = lon.get(index);
    return {
      index,
      latName: latitude?.name ?? `wp${index}_lat`,
      lonName: longitude?.name ?? `wp${index}_lon`,
      lat: latitude?.value ?? null,
      lon: longitude?.value ?? null,
      incomplete: latitude === undefined || longitude === undefined,
    };
  });
}

/** One row's text as a number of degrees, or `null` for anything this app
 *  cannot read as one. An empty string is `Number('') === 0`, so it is refused
 *  explicitly rather than becoming a position. */
function degrees(value: string | null): number | null {
  if (value === null) return null;
  const text = value.trim();
  if (text === '') return null;
  const parsed = Number(text);
  return Number.isFinite(parsed) ? parsed : null;
}

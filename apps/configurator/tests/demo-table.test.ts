import { describe, expect, it } from 'vitest';
import table from '../src/firmware/demo-table.json';

/**
 * What the demo board's starting table has to be true of.
 *
 * This file was `parameter-table.test.ts` and it read
 * `src/firmware/parameter-table.json`, and until milestone 3 that file was the
 * app's *only* source of groups, ranges, help text and decimal places: the wire
 * did not carry them, so a configurator that wanted to show "0.000..3.000"
 * beside a box either read them out of the firmware's source or invented them.
 *
 * It is now `src/firmware/demo-table.json` and it is a **fixture**. A real
 * board's description arrives over `param info` and `param help`, and
 * `AerialKitSession` reads it from there; nothing in the app attaches these rows
 * to a board that did not send them. The demo board is handed this table, and
 * then serves it back over the same two opcodes, so what the app renders came
 * off the wire either way — which is the whole point of keeping the file.
 *
 * **The staleness this file used to document is gone, and how it went is worth
 * keeping.** As of 2026-09-30 the file held 32 rows against a board reporting
 * 33: `arm_accel_lpf_hz` was missing and every row from index 13 on was
 * displaced by one. Nothing failed, because nothing *could* — the file was
 * internally consistent, and every consumer joined it to the board's reply by
 * name, so a name the file lacked produced a row with no range, which reads
 * exactly like a parameter that has none. `tools/check-table-drift.py` is the
 * check that catches it, it is wired into `npm run verify:drift`, and it now
 * passes rather than being red on purpose.
 */
describe('the demo board’s starting table', () => {
  const rows = table.parameters;

  it('is ordered by index, so a row can be found without a search', () => {
    rows.forEach((row, position) => {
      expect(row.index, `row ${position} (${row.name})`).toBe(position);
    });
  });

  it('names each parameter exactly once', () => {
    const names = rows.map((row) => row.name);
    expect(new Set(names).size).toBe(names.length);
  });

  it('carries every field the demo board serves over `param info`', () => {
    for (const row of rows) {
      expect(typeof row.name, row.name).toBe('string');
      expect(row.name.length, row.name).toBeGreaterThan(0);
      expect(['float', 'u32'], row.name).toContain(row.type);
      expect(Number.isInteger(row.decimals), row.name).toBe(true);
      expect(typeof row.help, row.name).toBe('string');
      // `null` is a real answer here — the extractor refuses to guess a bound it
      // cannot evaluate — so the check is that it is a number or null, never
      // undefined and never a string.
      for (const bound of [row.min, row.max]) {
        expect(bound === null || typeof bound === 'number', row.name).toBe(true);
      }
      if (row.min !== null && row.max !== null) {
        expect(row.min, row.name).toBeLessThanOrEqual(row.max);
      }
      // The group is the *last* argument of every registration in the firmware,
      // which is what makes a forgotten one a compile error there. Here it is
      // the number the demo board serves and the app renders under a name it
      // carries itself — so `undefined` would mean a row the board describes
      // with a group nobody stated, which is not a thing the firmware can write.
      expect(Number.isInteger(row.group), row.name).toBe(true);
      expect(row.group, row.name).toBeGreaterThan(0);
    }
  });

  it('says where it came from, so a hand-edit is visible', () => {
    // The header is the file's own account of itself, and `captured_product` is
    // what the values were read from. It is provenance now rather than an
    // authority: nothing gates on it any more, because the app no longer
    // decides whether to trust this file based on which board answered.
    expect(table.parameters.length).toBeGreaterThan(0);
    expect(table._comment).toMatch(/do not edit by hand/i);
    expect(table._comment).toMatch(/fixture/i);
    expect(table.captured_product).toBe('aerialkit-f405');
    // The demo board's product string is deliberately not the captured one: it
    // is how the app can tell a simulated board from the board in the capture.
    expect(table.demo_product).toBe('aerialkit-demo');
    expect(table.demo_product).not.toBe(table.captured_product);
  });
});

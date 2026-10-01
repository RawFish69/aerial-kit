import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { AerialKitClient } from '../src/protocol/client';
import { Command, InfoStatus } from '../src/protocol/constants';
import { buildFrame } from '../src/protocol/frame';
import { parseParamInfoPage } from '../src/protocol/messages';
import type { DemoParameter } from '../src/transport/demo-board';
import { BoardLink, makeBoard, realParameters } from './helpers';

/**
 * The board's description of itself, read over the wire.
 *
 * This is the half of the parameter table that used to live in a JSON file
 * beside the app. It is here now, in `AK_PROTO_CMD_PARAM_INFO` (0x0A) and
 * `AK_PROTO_CMD_PARAM_HELP` (0x0B), which means the app can be wrong about it in
 * three new ways that no file could produce: a walk that stops early, a page
 * whose `carried` byte disagrees with the bytes after it, and an offset walk
 * whose slices are joined at the wrong place — *prose joined at a wrong offset
 * reads perfectly*, which is why the last one is worth a test rather than a
 * glance.
 *
 * The peer is `DemoBoard`, which is held to the firmware's byte layout and to
 * its answers about itself: it must be able to look like a board from before
 * this command existed, because that is the state every board in the field is
 * in, and a demo board that only ever models the newest firmware tests the path
 * nobody has.
 */

let clients: AerialKitClient[] = [];

function clientOn(link: BoardLink): AerialKitClient {
  const client = new AerialKitClient(link);
  clients.push(client);
  return client;
}

beforeEach(() => {
  vi.useFakeTimers();
});

afterEach(() => {
  for (const client of clients) client.close();
  clients = [];
  vi.useRealTimers();
});

/** A row whose entry is bigger than a frame, so the board has to refuse it. */
function tooBig(nameLength = 90): DemoParameter {
  return {
    name: 'x'.repeat(nameLength),
    value: '1',
    default: '1',
    help: 'a row that cannot fit',
    type: 'float',
    decimals: 3,
    min: 0,
    max: 1,
    group: 1,
  };
}

describe('param info', () => {
  it('pages, and says on each page how many entries it actually carried', async () => {
    // 33 rows of ~30 bytes do not fit one 96-byte frame, so a walk that came
    // back with all of them would be a walk that had not happened.
    const board = makeBoard();
    const client = clientOn(new BoardLink(board));
    const first = await client.paramInfo(0);
    expect(first.status).toBe(InfoStatus.OK);
    expect(first.entries.length).toBeGreaterThan(0);
    expect(first.entries.length).toBeLessThan(33);
    expect(first.first).toBe(0);
    // The first entry is the row at index 0, described — not the row after it,
    // and not a row from anywhere else.
    expect(first.entries[0]!.name).toBe('rate_kp_roll');
    expect(first.entries[0]!.min).toBe('0.000');
  });

  it('picks up exactly where the previous page stopped, with no gap and no overlap', async () => {
    // A page that stopped *between* entries is fine; a page that stopped inside
    // one, or skipped one, produces a table that is right about most rows and
    // silently wrong about the boundary. Walking the pages by their own
    // `carried` count is the only way to see it.
    const board = makeBoard();
    const client = clientOn(new BoardLink(board));
    const names: string[] = [];
    let index = 0;
    let pages = 0;
    while (index < 33) {
      const page = await client.paramInfo(index);
      expect(page.first, `page starting at ${index}`).toBe(index);
      if (page.entries.length === 0) break;
      for (const entry of page.entries) names.push(entry.name);
      index += page.entries.length;
      pages++;
      expect(pages).toBeLessThan(40);
    }
    expect(names.length).toBe(33);
    expect(new Set(names).size).toBe(names.length);
    // And the same names, in the same order, as the value walk the app does
    // first. Two enumerations of one table that disagree is how this table has
    // failed before.
    const values: string[] = [];
    for (let i = 0; i < 33; i++) {
      const item = await client.paramGet(i);
      // `null` is the board saying there is no such index, which would make
      // this comparison vacuous — so it is asserted rather than asserted away.
      expect(item, `param get ${i}`).not.toBeNull();
      values.push(item!.name);
    }
    expect(names).toEqual(values);
  });

  it('reports a row too large to fit one frame, and does not stop there', async () => {
    // The failure this guards is the quiet one: `carried == 0` with status 0
    // means "the end of the table", and a client that read status 2 the same way
    // would render a 33-row table as one row shorter and never say so.
    const board = makeBoard({ parameters: [tooBig(), ...realParameters().slice(0, 2)] });
    const client = clientOn(new BoardLink(board));

    const refused = await client.paramInfo(0);
    expect(refused.status).toBe(InfoStatus.TOO_BIG);
    expect(refused.entries).toEqual([]);

    const walked = await client.paramInfoList(3);
    expect(walked.map((row) => row.index)).toEqual([0, 1, 2]);
    expect(walked[0]!.meta).toBeNull();
    expect(walked[0]!.unavailable).toMatch(/too large/i);
    // The rows after the hole are described normally: the hole is a hole, not
    // the end of the table.
    expect(walked[1]!.meta).not.toBeNull();
    expect(walked[2]!.meta).not.toBeNull();
  });

  it('refuses a request that named no index rather than answering from zero', async () => {
    // A page built from an assumed zero is indistinguishable from a complete
    // table, so the protocol answers this shape with one byte and no header.
    const board = makeBoard();
    const reply = board.feed(buildFrame(Command.PARAM_INFO, new Uint8Array(0)));
    expect(reply).not.toBeNull();
    // Sync, version, command|response, length, payload, crc.
    const command = reply![3]!;
    expect(command).toBe(Command.PARAM_INFO | 0x80);
    const payload = reply!.slice(5, reply!.length - 2);
    expect(payload.length).toBe(1);
    expect(payload[0]).toBe(InfoStatus.NO_INDEX);
    // And the parser turns it into that status rather than into entries.
    const page = parseParamInfoPage(payload, 0);
    expect(page.status).toBe(InfoStatus.NO_INDEX);
    expect(page.entries).toEqual([]);
  });
});

describe('param help', () => {
  it('reassembles a sentence longer than one frame, byte for byte', async () => {
    // 300 characters against a 89-byte window: four frames, and the offsets have
    // to line up or the sentence comes back as prose that reads fine and is not
    // what the board said.
    const long = Array.from({ length: 30 }, (_, i) => `sentence ${i} of the help text.`).join(' ');
    const rows = realParameters();
    rows[1] = { ...rows[1]!, help: long };
    const board = makeBoard({ parameters: rows });
    const client = clientOn(new BoardLink(board));
    expect(await client.paramHelp(1)).toBe(long);
  });

  it('tells an empty sentence apart from one it could not read', async () => {
    // The board saying it has no help is an empty string and travels as one.
    // "Not read" is this app's own state and never comes off the wire, which is
    // why the panel renders them differently.
    const rows = realParameters();
    rows[0] = { ...rows[0]!, help: '' };
    const board = makeBoard({ parameters: rows });
    const client = clientOn(new BoardLink(board));
    expect(await client.paramHelp(0)).toBe('');
    expect(await client.paramHelp(1)).not.toBe('');
  });

  it('refuses an index the board does not have rather than answering empty', async () => {
    const board = makeBoard();
    const client = clientOn(new BoardLink(board));
    await expect(client.paramHelp(200)).rejects.toThrow(/refused index 200/);
  });
});

describe('a board from before any of this', () => {
  /** A board whose hello ends before the capability word — the state every
   *  board built before 2026-09-30 is in. */
  function oldBoard(parameters: DemoParameter[] = realParameters()) {
    return makeBoard({ parameters, features: null });
  }

  it('answers 0x7f rather than going silent, and the app can read that answer', async () => {
    // The distinction the whole capability word rests on. A board that does not
    // implement a command still answers — with the protocol's unknown-command
    // byte, correlated to the request — so the app gets a fact in one round trip
    // instead of a two-second timeout and a shrug.
    const client = clientOn(new BoardLink(oldBoard()));
    await expect(client.paramInfo(0)).rejects.toThrow(/0x7[fF]/);
    await expect(client.paramInfo(0)).rejects.toThrow(/predates/i);
    await expect(client.paramHelp(0)).rejects.toThrow(/0x7[fF]/);
  });

  it('still serves the values, because the description is not the table', async () => {
    // Every name, index and value here is real and comes from the board. What
    // an old board cannot do is describe them, and the app must not lose the
    // table because it could not read the description.
    const client = clientOn(new BoardLink(oldBoard()));
    const hello = await client.hello();
    expect(hello.features).toBeNull();
    expect(hello.parameterCount).toBe(33);
    const row = await client.paramGet(0);
    expect(row).not.toBeNull();
    expect(row!.name).toBe('rate_kp_roll');
    expect(row!.value).toBe('0.250');
    await expect(client.paramInfoList(33)).rejects.toThrow(/0x7[fF]/);
  });

  it('is not a board with a word that says zero', async () => {
    // The two are different claims and the demo board can be either. A board
    // that answers its own word with no bits set is claiming it supports none
    // of the optional commands; a board whose reply ends early cannot say. The
    // bytes differ, so the app must too.
    const silent = clientOn(new BoardLink(oldBoard()));
    expect((await silent.hello()).features).toBeNull();

    const zero = clientOn(new BoardLink(makeBoard({ features: 0 })));
    const hello = await zero.hello();
    expect(hello.features).toBe(0);
    // Both answer 0x7F here, because neither sets the PARAM_INFO bit — the
    // *bit* is what decides, and the word's absence only changes the sentence.
    await expect(zero.paramInfo(0)).rejects.toThrow(/predates/i);
  });
});

describe('the two enumerations of one table', () => {
  it('agree index by index, name and group, for every row the board has', async () => {
    // `param get` and `param info` are two ways of walking one table, and the
    // way this table has failed before is the two drifting apart — a value walk
    // that reached 33 rows beside a description that covered 32, with nothing
    // anywhere comparing them. The firmware's own check makes this comparison
    // over the wire from the other side (`akproto_firmware_check.py`, 92
    // described and 92 named); this is the same invariant asserted where the app
    // would notice it breaking.
    const rows = realParameters();
    const client = clientOn(new BoardLink(makeBoard({ parameters: rows })));
    const descriptors = await client.paramInfoList(rows.length);
    expect(descriptors.length).toBe(rows.length);
    for (const [index, row] of rows.entries()) {
      expect(descriptors[index]!.index, `index ${index}`).toBe(index);
      expect(descriptors[index]!.meta!.name, `index ${index}`).toBe(row.name);
      // The group is the field this milestone added, and the one the Parameters
      // panel draws its headings from. Asserted against the fixture rather than
      // against the wire alone, so a board that grouped everything under 0 would
      // fail here rather than render one "unknown group" heading and pass.
      expect(descriptors[index]!.meta!.group, `index ${index}`).toBe(row.group);
      expect(descriptors[index]!.meta!.groupName, `index ${index}`).not.toBeNull();
    }
  });
});

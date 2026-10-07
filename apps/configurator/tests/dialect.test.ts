import { describe, expect, it } from 'vitest';

import {
  CRC_EXTRA,
  MAVLINK_MESSAGES,
  MavlinkDecoder,
  bytesBeyondTable,
  decode,
  deriveCrcExtra,
  mavLayout,
  type MavFieldType,
  type MavlinkFrame,
} from '../src/protocol/mavlink';
import dialect from './fixtures/dialect.json';

/**
 * The hand-written message table against the published MAVLink dialect.
 *
 * What this replaces, and why it needed replacing. The table in
 * `src/protocol/mavlink.ts` was already checked against two things, and both of
 * them were transcriptions: a `published` block of CRC_EXTRA values typed into
 * `mavlink.test.ts`, and a `crc_extra` block typed into the fixture by
 * `capture-mavlink-fixtures.py`. A transcription cannot disagree with the thing
 * it was transcribed from, so those tests caught typos and nothing else. Neither
 * could tell you whether `GPS_RAW_INT` really declares `yaw` after
 * `<extensions/>`, or whether `SERVO_OUTPUT_RAW` stops at eight servos — and
 * those are exactly the facts a decoder gets wrong and a checksum notices.
 *
 * `fixtures/dialect.json` is not transcribed. `tools/check-dialect.py` derives
 * it from the `common.xml` that **pymavlink** ships — the dialect both ArduPilot
 * and PX4 are generated from — including the include of `minimal.xml`, where
 * `HEARTBEAT` lives. So this is the first check in this suite where the app's
 * idea of what a message *is* is compared with something it did not write.
 *
 * **This is not a SITL test.** It needs no autopilot binary, and it does not
 * show that ArduPilot behaves as documented. It shows that the definitions this
 * app validates checksums against are the published ones. The tests that need a
 * running vehicle are in `sitl.test.ts`, and they say so.
 */

interface DialectField {
  readonly name: string;
  /** Typed as this app's own union so the oracle's fields can be handed to
   *  `deriveCrcExtra` directly. That is a declared claim, not a check — if the
   *  dialect ever grew a wire type this app does not know, the *runtime*
   *  field-for-field comparison below is what would catch it, by string. */
  readonly type: MavFieldType;
  readonly count?: number;
  readonly ext?: boolean;
}

interface DialectMessage {
  readonly id: number;
  readonly name: string;
  readonly crc_extra: number;
  readonly fields: readonly DialectField[];
}

const messages = dialect.messages as unknown as readonly DialectMessage[];

describe('the message table, against the published dialect', () => {
  it('carries exactly the messages the dialect file was built for', () => {
    expect(MAVLINK_MESSAGES.map((def) => `${def.id} ${def.name}`)).toEqual(
      messages.map((message) => `${message.id} ${message.name}`),
    );
  });

  it('agrees field for field, in declaration order', () => {
    // Declaration order, not wire order: the wire order is derived from these
    // declarations, so the declarations are what has to be right. A field that
    // moved in the XML would silently move on the wire too.
    for (const message of messages) {
      const ours = MAVLINK_MESSAGES.find((def) => def.id === message.id);
      expect(ours, `${message.name} is missing`).toBeDefined();
      const mine = ours!.fields.map((field) => ({
        name: field.name,
        type: field.type,
        ...(field.count === undefined ? {} : { count: field.count }),
        ...(field.ext === true ? { ext: true } : {}),
      }));
      expect(mine, `${message.name} fields`).toEqual(message.fields);
    }
  });

  it('derives the CRC_EXTRA the reference implementation derives', () => {
    // The CRC_EXTRA is a function of the message name, the field types, the
    // field names, the array lengths and which fields are extensions - so a
    // table that reproduces all fourteen has been checked field by field
    // against a constant derived by pymavlink, not typed in by hand.
    for (const message of messages) {
      const def = MAVLINK_MESSAGES.find((item) => item.id === message.id)!;
      expect(`${message.name}=${deriveCrcExtra(def)}`).toBe(
        `${message.name}=${message.crc_extra}`,
      );
      expect(CRC_EXTRA.get(message.id), message.name).toBe(message.crc_extra);
    }
  });

  it('puts every extension field last and outside the checksum', () => {
    // The two consequences of `ext`, asserted together because a table can have
    // one without the other and the second is the one that breaks a link: the
    // layout keeps an extension (it is on the wire) while the CRC_EXTRA
    // excludes it (which is what let `STATUSTEXT.id` be added at all).
    for (const message of messages) {
      const layout = mavLayout(message.id)!;
      const extension = message.fields.filter((field) => field.ext === true);
      const plain = message.fields.filter((field) => field.ext !== true);
      expect(layout.fields.length, message.name).toBe(message.fields.length);

      // Extensions come after every non-extension field on the wire, whatever
      // their size - the one place the size-sorting rule does not apply.
      const lastPlain = Math.max(...plain.map((field) => layout.fields.findIndex((f) => f.name === field.name)));
      for (const field of extension) {
        const at = layout.fields.findIndex((item) => item.name === field.name);
        expect(at, `${message.name}.${field.name} is not after the plain fields`).toBeGreaterThan(lastPlain);
      }

      if (extension.length > 0) {
        // And the checksum must not see them: the *dialect's* plain fields, with
        // the extensions dropped, have to derive the same CRC_EXTRA pymavlink
        // derived for the definition that has them. Derived from the oracle
        // rather than from our own table on purpose — a table that had lost a
        // plain field would otherwise still agree with itself here. That is the
        // whole reason `STATUSTEXT.id` could be added without breaking every
        // existing link, so it is worth checking against the source.
        expect(
          deriveCrcExtra({ id: message.id, name: message.name, fields: plain }),
          message.name,
        ).toBe(message.crc_extra);
      }
    }
  });

  it('names the dialect it was built from, so a stale file is visible', () => {
    // A regeneration from a newer pymavlink is a change to the evidence, and
    // the diff should say so in this line rather than only in the field lists.
    expect(dialect.source).toBe('common.xml');
    expect(typeof dialect.pymavlink).toBe('string');
    expect(dialect.pymavlink.length).toBeGreaterThan(0);
  });
});

/**
 * The same table, against a peer that is *newer* than it.
 *
 * Everything above asks whether this app's definitions are the published ones.
 * This asks the other question a ground station has to answer honestly: what
 * does it do with a vehicle that has moved on? MAVLink's answer is that a
 * dialect may append fields after `<extensions/>`, that those go on the wire
 * after every ordinary field, and that they are **excluded from the CRC_EXTRA**
 * — so a frame may validate its checksum while carrying bytes past the
 * definition the validator holds, and that is a legal frame rather than a
 * newer dialect to refuse.
 *
 * The frames below are the ones that rule produces, built by
 * `tools/check-dialect.py` from **PX4 1.17.0's own `common.xml`** — the file a
 * real autopilot is generated from — packed to MAVLink's wire rule and
 * checksummed with the `crc_extra` pymavlink derived. Six of the fifteen
 * messages this app carries have grown since pymavlink 2.4.49 was pinned.
 *
 * What the app got wrong is worth stating plainly, because it is the reason
 * this block exists: `decode()` used to throw on a payload longer than the
 * message it claimed to be, so a real PX4's `SYS_STATUS` — battery voltage,
 * current and sensor health — was dropped on arrival, one warning per frame,
 * for as long as the link was up.
 */

interface NewerPeerMessage {
  readonly name: string;
  readonly id: number;
  readonly crc_extra: number;
  readonly base_bytes: number;
  readonly full_bytes: number;
  readonly extension_fields: readonly string[];
  readonly frame_hex: string;
  /** The *base* fields as the script packed them. Extension values are
   *  deliberately absent: what the app does with bytes it has no definition for
   *  is asserted as "the fields it does model are unchanged", not as a value. */
  readonly expect: Readonly<Record<string, number | readonly number[] | string>>;
}

interface NewerPeer {
  readonly measured: string | null;
  readonly source: string | null;
  readonly messages: readonly NewerPeerMessage[];
}

const newer = dialect.newer_peer as unknown as NewerPeer;

function frameFrom(hex: string): MavlinkFrame {
  const bytes = new Uint8Array(hex.length / 2);
  for (let i = 0; i < bytes.length; i++) {
    bytes[i] = parseInt(hex.slice(i * 2, i * 2 + 2), 16);
  }
  const { frames, issues } = new MavlinkDecoder().push(bytes);
  expect(issues, `${hex.slice(0, 24)}…`).toEqual([]);
  expect(frames).toHaveLength(1);
  return frames[0]!;
}

/** A decoded field against the value the script packed, in the shape `decode`
 *  produces — numeric arrays come back comma-joined, because that is what this
 *  app's `DecodedMessage` is. */
function asDecoded(expect: number | readonly number[] | string): number | string {
  return Array.isArray(expect) ? expect.join(',') : (expect as number | string);
}

describe('a peer newer than this app’s table', () => {
  it('names where the newer definition was read, or says it was not read here', () => {
    // The block is carried forward on a machine with no PX4 checkout, and the
    // carried copy still names the source it was measured from. A `null`
    // `measured` with a `source` is exactly that case, and it is a reading
    // rather than a silence.
    expect(newer.messages.length).toBeGreaterThan(0);
    expect(newer.source).toBe('px4-1.17.0 common.xml');
    expect(newer.measured === null || newer.measured.includes('message_definitions')).toBe(true);
  });

  it('finds grown messages only among the ones this app carries, and by how much', () => {
    // The script refuses to write the file at all if PX4 and pymavlink disagree
    // about a *base* field list, because MAVLink does not change those. What is
    // left for this test is the arithmetic against the app's own layout, which
    // sits between PX4's base and PX4's full: it is at least the base, because
    // the app models the declared message, and never longer than what PX4 now
    // sends, because everything past that is an extension by definition.
    //
    // Some of these the app already models in full — `GPS_RAW_INT`'s `yaw` is
    // in its table — and some it does not. Both are what a pin looks like, and
    // the tests below separate them by measurement rather than by name.
    for (const entry of newer.messages) {
      const layout = mavLayout(entry.id);
      expect(layout, `${entry.name} is not in this app's table`).toBeDefined();
      expect(layout!.name, `${entry.name}`).toBe(entry.name);
      expect(layout!.length, `${entry.name} layout`).toBeGreaterThanOrEqual(entry.base_bytes);
      expect(layout!.length, `${entry.name} layout`).toBeLessThanOrEqual(entry.full_bytes);
      expect(entry.full_bytes, `${entry.name} grew by something`).toBeGreaterThan(entry.base_bytes);
      expect(entry.extension_fields.length, `${entry.name} extension fields`).toBeGreaterThan(0);
    }
  });

  it('reads the frame from its prefix, and every base field is unchanged', () => {
    for (const entry of newer.messages) {
      const frame = frameFrom(entry.frame_hex);
      expect(frame.name, entry.name).toBe(entry.name);
      expect(frame.payload.length, `${entry.name} as it arrived`).toBe(entry.full_bytes);

      const decoded = decode(frame);
      for (const [name, packed] of Object.entries(entry.expect)) {
        expect(decoded[name], `${entry.name}.${name}`).toEqual(asDecoded(packed));
      }
    }
  });

  it('leaves unread exactly the bytes past the app’s own definition', () => {
    // The count is against *this app's* layout and not PX4's base, because the
    // app models some extension fields and the two are different questions. For
    // a message the app knows in full this is zero — nothing is unread, and the
    // session has nothing to report.
    for (const entry of newer.messages) {
      const frame = frameFrom(entry.frame_hex);
      const known = mavLayout(entry.id)!.length;
      expect(bytesBeyondTable(frame), entry.name).toBe(entry.full_bytes - known);
    }
  });

  //: The messages this app's table is genuinely behind on: PX4 sends more than
  //: the app knows. Measured here rather than listed, so a table that catches
  //: up makes these two tests vacuous *and* says so below.
  const behind = newer.messages.filter(
    (entry) => entry.full_bytes > mavLayout(entry.id)!.length,
  );

  it('is genuinely behind on some of them, so the two tests below are not idle', () => {
    // Without this, both tests below would pass on an empty list — the shape of
    // check that quietly stops checking anything. `SYS_STATUS` is the one the
    // defect was found on and is named here for that reason.
    expect(behind.map((entry) => entry.name)).toContain('SYS_STATUS');
  });

  it('reads a frame it is behind on the same as the frame without the extra bytes', () => {
    // The strongest statement available: the bytes the app cannot name make no
    // difference at all to the fields it can. If they did, tolerating them
    // would be tolerating a *wrong reading* rather than a longer one.
    for (const entry of behind) {
      const frame = frameFrom(entry.frame_hex);
      const trimmed: MavlinkFrame = {
        ...frame,
        payload: frame.payload.subarray(0, mavLayout(entry.id)!.length),
      };
      expect(decode(frame), entry.name).toEqual(decode(trimmed));
    }
  });

  it('does not invent a value for a field it has no definition for', () => {
    for (const entry of behind) {
      const decoded = decode(frameFrom(entry.frame_hex));
      for (const field of entry.extension_fields) {
        // Only the ones past the app's own layout: a field it *does* carry is
        // read, and this test is about the ones it does not.
        if (mavLayout(entry.id)!.fields.some((item) => item.name === field)) continue;
        expect(decoded[field], `${entry.name}.${field}`).toBeUndefined();
      }
    }
  });

  it('still refuses a newer frame whose checksum does not match', () => {
    // Tolerance for length must not become tolerance for corruption: the whole
    // reason an over-long payload is safe to read from its prefix is that the
    // checksum already proved the base fields, and that argument only holds if
    // the checksum is still being checked.
    const entry = newer.messages.find((item) => item.name === 'SYS_STATUS')!;
    const bytes = new Uint8Array(entry.frame_hex.length / 2);
    for (let i = 0; i < bytes.length; i++) {
      bytes[i] = parseInt(entry.frame_hex.slice(i * 2, i * 2 + 2), 16);
    }
    // A bit flipped *inside the extension region*, which is the only part of the
    // frame the app does not model — so a decoder that had given up on the tail
    // would still be catching this.
    bytes[bytes.length - 5]! ^= 0x01;
    const { frames, issues } = new MavlinkDecoder().push(bytes);
    expect(frames).toEqual([]);
    expect(issues).toEqual([{ kind: 'checksum', msgid: entry.id }]);
  });
});

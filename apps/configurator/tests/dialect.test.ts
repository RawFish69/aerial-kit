import { describe, expect, it } from 'vitest';

import {
  CRC_EXTRA,
  MAVLINK_MESSAGES,
  deriveCrcExtra,
  mavLayout,
  type MavFieldType,
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

import { describe, expect, it, vi } from 'vitest';

import { detectFirmware } from '../src/protocol/detect';
import { FOREIGN_LIMITATIONS, MspBoard } from '../src/session/foreign';
import { BoardLink, FakeClock, makeBoard, ManualLink } from './helpers';
import { FixtureLink, FixtureTransport } from './msp-link';

/** A board that answers our own protocol, with everything it was sent kept. */
class RecordingBoardLink extends BoardLink {
  readonly written: Uint8Array[] = [];
  override write(bytes: Uint8Array): void {
    this.written.push(bytes);
    super.write(bytes);
  }
}

describe('telling the firmwares apart', () => {
  it('recognises our own board and asks it nothing else', async () => {
    const link = new RecordingBoardLink(makeBoard());
    const detection = await detectFirmware(link);

    expect(detection.family).toBe('aerialkit');
    expect(detection.detail).toContain('aerialkit-demo');
    // The point of the order. A board that answered our hello is ours, and is
    // never asked an MSP question — on a real board that port carries the
    // console too, and typing at a console is doing.
    expect(link.written.length).toBeGreaterThan(0);
    expect(link.written.every((frame) => frame[0] === 0xaa && frame[1] === 0x55)).toBe(true);
  });

  it('recognises a Betaflight board and says it is read-only', async () => {
    const detection = await detectFirmware(new FixtureLink('betaflight'));
    expect(detection.family).toBe('msp');
    expect(detection.detail).toContain('BTFL');
    expect(detection.detail).toContain('read-only');
  });

  it('recognises INAV as INAV', async () => {
    expect((await detectFirmware(new FixtureLink('inav'))).detail).toContain('INAV');
  });

  it('reports silence as silence', async () => {
    vi.useFakeTimers();
    try {
      const pending = detectFirmware(new ManualLink(), { probeMs: 100, listenMs: 100 });
      await vi.advanceTimersByTimeAsync(1000);
      const detection = await pending;
      expect(detection.family).toBe('silent');
      expect(detection.detail).toContain('nothing answered');
    } finally {
      vi.useRealTimers();
    }
  });

  it('reports a link that is talking in something else as unrecognised, not as a guess', async () => {
    vi.useFakeTimers();
    try {
      const link = new ManualLink();
      // A MAVLink v2 magic and a length, arriving on their own. This app *does*
      // have a MAVLink adapter now, and it still says "I heard something and it
      // was not mine" — because three bytes cannot satisfy a checksum that needs
      // a per-message constant, so no MAVLink frame was actually heard. A stray
      // `0xfd` in console noise must not become a PX4.
      setTimeout(() => link.deliver(Uint8Array.of(0xfd, 0x09, 0x00)), 400);
      const pending = detectFirmware(link, { probeMs: 100, listenMs: 300 });
      await vi.advanceTimersByTimeAsync(1000);
      const detection = await pending;
      expect(detection.family).toBe('unrecognised');
      expect(detection.firstByte).toBe(0xfd);
      expect(detection.detail).toContain('not in a protocol this app implements');
    } finally {
      vi.useRealTimers();
    }
  });
});

describe('a foreign board, read-only', () => {
  it('identifies itself and reads live state, and offers no way to write', async () => {
    const clock = new FakeClock();
    const transport = new FixtureTransport();
    const board = new MspBoard(transport, { now: clock.now });
    await board.open();

    const snapshot = board.snapshot;
    expect(snapshot.phase).toBe('ready');
    expect(snapshot.identity?.variant.variant).toBe('BTFL');
    expect(snapshot.identity?.board?.boardName).toBe('AERIALKIT-SIM');
    expect(snapshot.identity?.api.apiMinor).toBe(48);
    expect(snapshot.live.armed).toBe('disarmed');
    expect(snapshot.live.stale).toBe(false);
    expect(snapshot.live.attitude).not.toBeNull();
    expect(snapshot.live.gps?.satellites).toBe(11);

    // The type is the safety property: this class has no `write`, `edit`,
    // `save` or `stream` to call. Asserted rather than described, because a
    // method added later by someone in a hurry would otherwise go unnoticed.
    for (const forbidden of ['write', 'edit', 'save', 'stream', 'writeAll', 'reread']) {
      expect(board).not.toHaveProperty(forbidden);
    }
  });

  it('reads a setting by name, because the protocol has no way to list them', async () => {
    const board = new MspBoard(new FixtureTransport());
    await board.open();
    await board.lookup('failsafe_throttle');
    await board.describe('failsafe_throttle');

    const setting = board.snapshot.settings.find((item) => item.name === 'failsafe_throttle');
    expect(setting?.value).toBe('1050');
    expect(setting?.refusal).toBeNull();
    expect(setting?.info?.fields['min']).toBe('1000');
    expect(setting?.info?.fields['max']).toBe('2000');
  });

  it('records a refusal as the board saying no, not as an empty value', async () => {
    const board = new MspBoard(new FixtureTransport());
    await board.open();
    await board.lookup('no_such_setting_at_all');

    const setting = board.snapshot.settings.find((item) => item.name === 'no_such_setting_at_all');
    // The two are different facts. A board that has no such setting is not a
    // board whose setting is empty, and a screen that showed "" would be
    // reporting a bug of its own as a property of the aircraft.
    expect(setting?.value).toBeNull();
    expect(setting?.refusal).toContain('does not have a setting by that name');
  });

  it('lets the armed state go stale into unknown, never into disarmed', async () => {
    const clock = new FakeClock();
    const board = new MspBoard(new FixtureTransport(), { now: clock.now });
    await board.open();
    expect(board.snapshot.live.armed).toBe('disarmed');

    clock.advance(2500);
    board.age();
    expect(board.snapshot.live.stale).toBe(true);
    expect(board.snapshot.live.armed).toBe('unknown');
  });

  it('states what it cannot establish', () => {
    // Every claim in here is one this app is making about itself, so the list
    // is asserted rather than left to drift.
    expect(FOREIGN_LIMITATIONS.join(' ')).toContain('Read-only');
    expect(FOREIGN_LIMITATIONS.join(' ')).toContain('no "list the settings" command');
    expect(FOREIGN_LIMITATIONS.join(' ')).toContain('MAVLink is not implemented');
    expect(FOREIGN_LIMITATIONS.join(' ')).toContain('physical board');
  });
});

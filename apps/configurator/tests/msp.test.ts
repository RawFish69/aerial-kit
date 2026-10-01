import { describe, expect, it, vi } from 'vitest';

import { bytes, captured, FixtureLink, hex, keys, payloadOf, type Variant } from './msp-link';
import {
  buildRequest,
  buildRequestV2,
  crc8DvbS2,
  MspClient,
  MspCommand,
  MspDecoder,
  MspLinkClosedError,
  MSP2_CLI_SETTING,
  MSP2_CLI_SETTING_INFO,
  MspProtocolError,
  MspRefusal,
  MspTimeoutError,
  parseAnalog,
  parseApiVersion,
  parseAttitude,
  parseBoardInfo,
  parseFcVariant,
  parseFcVersion,
  parseMotor,
  parseRawGps,
  parseRc,
  parseSetting,
  parseSettingInfo,
  parseStatus,
} from '../src/protocol/msp';

describe('MSP framing', () => {
  it('computes the CRC the board itself computed', () => {
    // The captured v2 request for `failsafe_throttle` ends in the CRC the
    // stand-in accepted, so this is the board's own arithmetic and not ours.
    const frame = bytes(captured('betaflight', 'setting_known').request);
    const crc = frame[frame.length - 1]!;
    expect(crc8DvbS2(frame.subarray(3, frame.length - 1))).toBe(crc);
  });

  it('builds requests byte-identical to the ones a real client sent', () => {
    // These are not round-trips through our own decoder: they are compared
    // against bytes `firmware/tools/msp.py` put on the wire and a stand-in
    // answered.
    for (const variant of ['betaflight', 'inav'] as const) {
      expect(hex(buildRequest(MspCommand.FC_VARIANT))).toBe(
        captured(variant, 'fc_variant').request,
      );
      expect(hex(buildRequest(MspCommand.STATUS))).toBe(captured(variant, 'status').request);
      expect(hex(buildRequest(MspCommand.BOARD_INFO))).toBe(
        captured(variant, 'board_info').request,
      );
    }
    // MSP2_CLI_SETTING takes the bare name; MSP2_CLI_SETTING_INFO is the one
    // that wants `name\0` plus an offset. Sending the wrong one gets a refusal.
    expect(hex(buildRequestV2(MSP2_CLI_SETTING, new TextEncoder().encode('failsafe_throttle')))).toBe(
      captured('betaflight', 'setting_known').request,
    );
    expect(
      hex(buildRequestV2(MSP2_CLI_SETTING, new TextEncoder().encode('no_such_setting_at_all'))),
    ).toBe(captured('betaflight', 'setting_absent').request);
    expect(
      hex(buildRequestV2(MSP2_CLI_SETTING_INFO, new TextEncoder().encode('failsafe_throttle\0'))),
    ).toBe(captured('betaflight', 'setting_info_known').request);
  });

  it('refuses to build a v1 frame whose payload does not fit a size byte', () => {
    expect(() => buildRequest(MspCommand.STATUS, new Uint8Array(300))).toThrow(RangeError);
  });

  it('decodes every captured reply', () => {
    for (const variant of ['betaflight', 'inav'] as const) {
      for (const key of keys(variant)) {
        const { reply } = captured(variant, key);
        const { frames, issues } = new MspDecoder().push(bytes(reply));
        expect(issues, `${variant}/${key}`).toEqual([]);
        expect(frames, `${variant}/${key}`).toHaveLength(1);
        expect(frames[0]!.payload, `${variant}/${key}`).toEqual(payloadOf(reply));
      }
    }
  });

  it('gets the same answer whatever sizes the transport delivered', () => {
    const frame = bytes(captured('betaflight', 'board_info').reply);
    const oneByteAtATime = new MspDecoder();
    let collected: number[] = [];
    for (const byte of frame) {
      const { frames } = oneByteAtATime.push(Uint8Array.of(byte));
      // Nothing may be emitted before the last byte arrives, and nothing may
      // be lost when it does.
      if (frames.length > 0) collected = collected.concat(...frames.map((f) => f.command));
      else expect(collected).toEqual([]);
    }
    expect(collected).toEqual([MspCommand.BOARD_INFO]);
    expect(oneByteAtATime.held).toBe(0);
  });

  it('finds a frame in a stream that starts with something else', () => {
    // Detection types at a board that speaks another protocol, and people paste
    // things into a console. A decoder that only worked from a clean start
    // would be unusable on the link this app actually has.
    const frame = bytes(captured('betaflight', 'status').reply);
    const noisy = new Uint8Array(7 + frame.length);
    noisy.set(new TextEncoder().encode('# hello'), 0);
    noisy.set(frame, 7);
    const { frames, issues } = new MspDecoder().push(noisy);
    expect(issues).toEqual([]);
    expect(frames.map((f) => f.command)).toEqual([MspCommand.STATUS]);
  });

  it('reports a corrupted frame and still finds the good one behind it', () => {
    const good = bytes(captured('betaflight', 'status').reply);
    const bad = Uint8Array.from(good);
    bad[bad.length - 1] = bad[bad.length - 1]! ^ 0xff;
    const joined = new Uint8Array(bad.length + good.length);
    joined.set(bad, 0);
    joined.set(good, bad.length);

    const { frames, issues } = new MspDecoder().push(joined);
    expect(issues).toEqual([{ kind: 'checksum', version: 1 }]);
    expect(frames.map((f) => f.command)).toEqual([MspCommand.STATUS]);
  });

  it('reads a refusal as a refusal', () => {
    const { frames } = new MspDecoder().push(bytes(captured('betaflight', 'setting_absent').reply));
    expect(frames).toHaveLength(1);
    expect(frames[0]!.refused).toBe(true);
    expect(frames[0]!.command).toBe(MSP2_CLI_SETTING);
    expect(frames[0]!.payload).toHaveLength(0);
  });
});

describe('MSP messages', () => {
  const payloadFor = (variant: Variant, key: string): Uint8Array =>
    payloadOf(captured(variant, key).reply);

  it('reads the protocol version', () => {
    expect(parseApiVersion(payloadFor('betaflight', 'api_version'))).toEqual({
      protocolVersion: 0,
      apiMajor: 1,
      apiMinor: 48,
    });
  });

  it('tells Betaflight and INAV apart', () => {
    expect(parseFcVariant(payloadFor('betaflight', 'fc_variant'))).toEqual({
      variant: 'BTFL',
      known: true,
    });
    expect(parseFcVariant(payloadFor('inav', 'fc_variant'))).toEqual({
      variant: 'INAV',
      known: true,
    });
    // A firmware this app has never heard of is reported, not hidden: the
    // string decides which parameter names apply, and guessing would be worse
    // than saying so.
    expect(parseFcVariant(new TextEncoder().encode('QUAD'))).toEqual({
      variant: 'QUAD',
      known: false,
    });
  });

  it('reads the release and the board description', () => {
    expect(parseFcVersion(payloadFor('betaflight', 'fc_version')).version).toBe('2026.6.1');
    expect(parseBoardInfo(payloadFor('betaflight', 'board_info'))).toEqual({
      boardIdentifier: 'S405',
      targetName: 'STM32F405',
      boardName: 'AERIALKIT-SIM',
      manufacturer: 'AK',
    });
  });

  it('reads arming, sensor health and the profile from status', () => {
    const status = parseStatus(payloadFor('betaflight', 'status'));
    expect(status).toEqual({
      cycleTimeUs: 1000,
      i2cErrors: 0,
      sensors: { accel: true, baro: true, mag: false, gps: true, gyro: true },
      armed: false,
      modeFlags: 0,
      profile: 0,
    });
  });

  it('reads attitude, GPS, motors and the channel values', () => {
    const attitude = parseAttitude(payloadFor('betaflight', 'attitude'));
    expect(Number.isFinite(attitude.rollDeg)).toBe(true);
    expect(Number.isFinite(attitude.pitchDeg)).toBe(true);
    expect(Number.isFinite(attitude.yawDeg)).toBe(true);

    // These are exact because the stand-in's own source says so: `RAW_GPS`
    // carries `bytes([3, 11])` and `pack("<HHH", 41, 512, 2750)`.
    expect(parseRawGps(payloadFor('betaflight', 'raw_gps'))).toMatchObject({
      fixType: 3,
      satellites: 11,
      altitudeM: 41,
      speedCmS: 512,
      courseDeg: 275,
    });

    // `MSP_RC` is `pack("<8H", 992, 992, 992, 172, 992, 1811, 992, 992)`.
    expect(parseRc(payloadFor('betaflight', 'rc'))).toEqual([
      992, 992, 992, 172, 992, 1811, 992, 992,
    ]);

    expect(parseMotor(payloadFor('betaflight', 'motor'))).toHaveLength(8);

    const analog = parseAnalog(payloadFor('betaflight', 'analog'));
    expect(analog.rssi).toBe(90);
    expect(analog.amps).toBe(1250);
    expect(analog.volts).toBeGreaterThan(0);
  });

  it('reads a setting and its description', () => {
    expect(parseSetting(payloadFor('betaflight', 'setting_known'))).toEqual({
      name: 'failsafe_throttle',
      value: '1050',
    });
    const info = parseSettingInfo(payloadFor('betaflight', 'setting_info_known'));
    expect(info.fields['pgn']).toBe('21');
    expect(info.fields['type']).toBe('uint16');
    expect(info.fields['min']).toBe('1000');
    expect(info.fields['max']).toBe('2000');
    // The whole description is longer than the window carrying it, which is the
    // entire reason the command takes an offset.
    expect(info.totalLength).toBeGreaterThan(info.window.length);
  });

  it('throws on a payload that is too short rather than inventing zeros', () => {
    // A zero latitude is a real place in the Atlantic. "The board did not say"
    // and "the board said zero" are different answers and must not be merged.
    expect(() => parseApiVersion(new Uint8Array([0, 1]))).toThrow(MspProtocolError);
    expect(() => parseRawGps(new Uint8Array([3, 11, 0, 0]))).toThrow(MspProtocolError);
    expect(() => parseBoardInfo(new Uint8Array([0x53, 0x34]))).toThrow(MspProtocolError);
  });
});

describe('MSP client', () => {
  it('reads a board by asking it, never by assuming', async () => {
    const link = new FixtureLink('betaflight');
    const client = new MspClient(link);
    const identity = await client.identify();

    expect(identity.variant.variant).toBe('BTFL');
    expect(identity.api.apiMinor).toBe(48);
    expect(identity.release.version).toBe('2026.6.1');
    expect(identity.board?.boardIdentifier).toBe('S405');
    // Every one of those was a read. Nothing this app sent a board it has not
    // identified was anything but a question.
    expect(link.written.map((frame) => frame[4])).toEqual([
      MspCommand.API_VERSION,
      MspCommand.FC_VARIANT,
      MspCommand.FC_VERSION,
      MspCommand.BOARD_INFO,
    ]);
  });

  it('identifies INAV as INAV', async () => {
    const client = new MspClient(new FixtureLink('inav'));
    expect((await client.identify()).variant.variant).toBe('INAV');
  });

  it('reports a refused setting as a refusal, not as an empty answer', async () => {
    const client = new MspClient(new FixtureLink());
    await expect(
      client.requestV2(MSP2_CLI_SETTING, new TextEncoder().encode('no_such_setting_at_all')),
    ).rejects.toBeInstanceOf(MspRefusal);
  });

  it('gives up on a board that stays silent', async () => {
    vi.useFakeTimers();
    try {
      const link = new FixtureLink();
      link.ignore();
      const client = new MspClient(link, { timeoutMs: 250 });
      const pending = client.request(MspCommand.API_VERSION);
      const assertion = expect(pending).rejects.toBeInstanceOf(MspTimeoutError);
      await vi.advanceTimersByTimeAsync(300);
      await assertion;
    } finally {
      vi.useRealTimers();
    }
  });

  it('sends one request at a time', async () => {
    // MSP ties a reply to its request by the command number alone — no response
    // bit, no sequence. Two of the same command outstanding at once could not be
    // told apart, so the second must not go out until the first is answered.
    const link = new FixtureLink();
    const client = new MspClient(link);
    const first = client.request(MspCommand.STATUS);
    const second = client.request(MspCommand.STATUS);
    expect(link.written).toHaveLength(1);
    await Promise.all([first, second]);
    expect(link.written).toHaveLength(2);
  });

  it('counts a frame nobody asked for instead of mistaking it for the answer', async () => {
    const link = new FixtureLink();
    const seen: number[] = [];
    const client = new MspClient(link, { onUnsolicited: (frame) => seen.push(frame.command) });
    const pending = client.request(MspCommand.API_VERSION);
    link.deliver(bytes(captured('betaflight', 'status').reply));
    await Promise.resolve();
    expect(seen).toEqual([MspCommand.STATUS]);
    // The question is still outstanding, and is then answered properly.
    expect(parseApiVersion(await pending).apiMajor).toBe(1);
  });

  it('fails what is in flight when the cable is pulled', async () => {
    const link = new FixtureLink();
    link.ignore();
    const client = new MspClient(link, { timeoutMs: 5000 });
    const pending = client.request(MspCommand.STATUS);
    link.hangUp();
    await expect(pending).rejects.toBeInstanceOf(MspLinkClosedError);
  });
});

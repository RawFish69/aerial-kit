/**
 * MAVLink — the protocol ArduPilot and PX4 speak, from the browser's side.
 *
 * The third and last family. AerialKit speaks its own protocol, Betaflight and
 * INAV speak MSP, and a vehicle speaks MAVLink. This file knows MAVLink.
 *
 * Everything below is written from the wire format and from the published
 * message definitions, and every decoder is pinned against frames captured from
 * `firmware/tools/mavlink_fake_vehicle.py` (`tools/capture-mavlink-fixtures.py`),
 * whose frames are built by **pymavlink** — the reference implementation behind
 * Mission Planner, QGroundControl and MAVProxy. So a fixture in
 * `tests/fixtures/mavlink.json` has been agreed on by two independent readers,
 * and the capture script's framing is longhand rather than imported.
 *
 * Five things about MAVLink differ from both protocols already here, and each
 * one changes the shape of this file:
 *
 *  - **It cannot be probed. It is listened to.** MSP answers a question and our
 *    own board answers a hello; a vehicle heartbeats on its own, unasked, and
 *    there is nothing safe to send a link you have not identified. So this
 *    module is *passive* first: `MavlinkDecoder` is fed whatever arrives and
 *    reports what it recognised. Detection never transmits on a MAVLink link,
 *    and neither does anything else until a heartbeat has been heard.
 *  - **The CRC needs a per-message secret.** MAVLink's checksum is CRC-16/MCRF4XX
 *    over the frame with one extra byte accumulated at the end, and that byte is
 *    a function of the message definition — its name, field types, field names
 *    and array lengths — and is *not on the wire*. A decoder that does not carry
 *    a message's definition cannot verify its checksum at all, which is why
 *    `MAVLINK_MESSAGES` below is the actual table of contents for what this app
 *    can read. Anything else is counted and reported as unread, not guessed at.
 *  - **Field order on the wire is not declaration order.** Fields are sorted by
 *    type size, largest first, with extension fields appended in declaration
 *    order afterwards. That rule is implemented once, in `wireLayout()`, and
 *    checked in two independent ways: the derived CRC_EXTRA must equal the
 *    published one for every message, and the derived offsets must reproduce the
 *    bytes pymavlink actually packed. Both are tests.
 *  - **v2 truncates trailing zeros.** The length byte is the payload length, not
 *    the struct size. `decode()` zero-pads to the struct size, so a short
 *    payload is a short payload and not a parse error — which is the whole point
 *    of the feature and the one thing a decoder written from the struct alone
 *    gets wrong.
 *  - **Nothing correlates a reply to a request.** There is no sequence number in
 *    the reply and no response bit; ACKs and values carry their own subject
 *    (a command id, a parameter index) and everything else is a stream.
 *
 * **Read-only, deliberately, and enforced by what can be sent.** MAVLink's
 * workhorse message is `COMMAND_LONG`, and the same message id carries
 * `MAV_CMD_COMPONENT_ARM_DISARM`, `MAV_CMD_NAV_TAKEOFF` and
 * `MAV_CMD_DO_SET_HOME`. A configurator that exposed a general `command()` would
 * be one typo from spinning a propeller. So this file has no general command
 * builder. Exactly two messages can leave it — `PARAM_REQUEST_LIST` and
 * `COMMAND_LONG` carrying `MAV_CMD_SET_MESSAGE_INTERVAL` and nothing else — and
 * `buildFrame` is not exported. A test drives the whole client and asserts the
 * set of message ids that reached the wire.
 */

// ---- framing ---------------------------------------------------------------

export const MAVLINK_STX_V1 = 0xfe;
export const MAVLINK_STX_V2 = 0xfd;
/** v2's incompatibility flag for a signed frame. The signature is 13 bytes of
 *  SHA-256 truncated; this app does not verify it and says so rather than
 *  implying the frame is authenticated. */
export const MAVLINK_IFLAG_SIGNED = 0x01;
export const MAVLINK_SIGNATURE_LENGTH = 13;
/** The length byte is a byte. Used to bound what a stream of noise can make
 *  this allocate. */
export const MAVLINK_MAX_PAYLOAD = 255;

// ---- the message definitions ----------------------------------------------

/**
 * Wire scalar types, named as the dialect names them.
 *
 * These spellings are not cosmetic: `dialectTypeName` feeds the CRC_EXTRA
 * derivation, and one character out changes every extra and silently stops
 * every frame validating.
 */
export type MavFieldType =
  | 'uint8_t' | 'int8_t'
  | 'uint16_t' | 'int16_t'
  | 'uint32_t' | 'int32_t'
  | 'uint64_t' | 'int64_t'
  | 'float' | 'double'
  | 'char';

/** The size the *sort* uses. A char array sorts as one byte, however long it
 *  is — which is the single most surprising thing in MAVLink's field ordering
 *  and the reason `PARAM_VALUE` puts its 16-byte `param_id` after two `u16`s. */
const SORT_SIZE: Record<MavFieldType, number> = {
  uint8_t: 1, int8_t: 1, char: 1,
  uint16_t: 2, int16_t: 2,
  uint32_t: 4, int32_t: 4, float: 4,
  uint64_t: 8, int64_t: 8, double: 8,
};

const SCALAR_SIZE: Record<MavFieldType, number> = {
  uint8_t: 1, int8_t: 1, char: 1,
  uint16_t: 2, int16_t: 2,
  uint32_t: 4, int32_t: 4, float: 4,
  uint64_t: 8, int64_t: 8, double: 8,
};

export interface MavFieldDef {
  readonly name: string;
  readonly type: MavFieldType;
  /** Array length, for `char[n]` and `uint8_t[n]`. Absent means a scalar. */
  readonly count?: number;
  /**
   * True for a field the dialect appends after `<extensions/>`.
   *
   * Two consequences, and both matter: extension fields go to the *end* of the
   * wire layout unsorted, and they are **excluded from the CRC_EXTRA**. That
   * exclusion is the entire point of extensions — it is what let MAVLink add
   * `STATUSTEXT.id` and `GPS_RAW_INT.yaw` without breaking every existing
   * ground station — so getting this flag wrong makes the derived extra wrong
   * for that message and no frame of it will ever validate.
   */
  readonly ext?: boolean;
}

export interface MavMessageDef {
  readonly id: number;
  readonly name: string;
  /** Declaration order, exactly as the dialect declares them. The wire order is
   *  derived, never written down here. */
  readonly fields: readonly MavFieldDef[];
}

/**
 * Every message this app can read, and therefore every message whose checksum
 * it can verify.
 *
 * This is a subset of `common.xml`, deliberately. MAVLink's CRC_EXTRA is not on
 * the wire, so an implementation either carries a definition or cannot validate
 * a frame at all; a real ground station ships the whole dialect. This one ships
 * what it draws on screen and reports the rest as unread, which is the honest
 * version of the same fact.
 *
 * The parameter and command messages are here because this app sends them, not
 * only because it reads them.
 */
export const MAVLINK_MESSAGES: readonly MavMessageDef[] = [
  {
    id: 0,
    name: 'HEARTBEAT',
    fields: [
      { name: 'type', type: 'uint8_t' },
      { name: 'autopilot', type: 'uint8_t' },
      { name: 'base_mode', type: 'uint8_t' },
      { name: 'custom_mode', type: 'uint32_t' },
      { name: 'system_status', type: 'uint8_t' },
      { name: 'mavlink_version', type: 'uint8_t' },
    ],
  },
  {
    id: 1,
    name: 'SYS_STATUS',
    fields: [
      { name: 'onboard_control_sensors_present', type: 'uint32_t' },
      { name: 'onboard_control_sensors_enabled', type: 'uint32_t' },
      { name: 'onboard_control_sensors_health', type: 'uint32_t' },
      { name: 'load', type: 'uint16_t' },
      { name: 'voltage_battery', type: 'uint16_t' },
      { name: 'current_battery', type: 'int16_t' },
      { name: 'battery_remaining', type: 'int8_t' },
      { name: 'drop_rate_comm', type: 'uint16_t' },
      { name: 'errors_comm', type: 'uint16_t' },
      { name: 'errors_count1', type: 'uint16_t' },
      { name: 'errors_count2', type: 'uint16_t' },
      { name: 'errors_count3', type: 'uint16_t' },
      { name: 'errors_count4', type: 'uint16_t' },
    ],
  },
  {
    id: 21,
    name: 'PARAM_REQUEST_LIST',
    fields: [
      { name: 'target_system', type: 'uint8_t' },
      { name: 'target_component', type: 'uint8_t' },
    ],
  },
  {
    id: 22,
    name: 'PARAM_VALUE',
    fields: [
      { name: 'param_id', type: 'char', count: 16 },
      { name: 'param_value', type: 'float' },
      { name: 'param_type', type: 'uint8_t' },
      { name: 'param_count', type: 'uint16_t' },
      { name: 'param_index', type: 'uint16_t' },
    ],
  },
  {
    id: 24,
    name: 'GPS_RAW_INT',
    fields: [
      { name: 'time_usec', type: 'uint64_t' },
      { name: 'fix_type', type: 'uint8_t' },
      { name: 'lat', type: 'int32_t' },
      { name: 'lon', type: 'int32_t' },
      { name: 'alt', type: 'int32_t' },
      { name: 'eph', type: 'uint16_t' },
      { name: 'epv', type: 'uint16_t' },
      { name: 'vel', type: 'uint16_t' },
      { name: 'cog', type: 'uint16_t' },
      { name: 'satellites_visible', type: 'uint8_t' },
      { name: 'alt_ellipsoid', type: 'int32_t', ext: true },
      { name: 'h_acc', type: 'uint32_t', ext: true },
      { name: 'v_acc', type: 'uint32_t', ext: true },
      { name: 'vel_acc', type: 'uint32_t', ext: true },
      { name: 'hdg_acc', type: 'uint32_t', ext: true },
      { name: 'yaw', type: 'uint16_t', ext: true },
    ],
  },
  {
    id: 30,
    name: 'ATTITUDE',
    fields: [
      { name: 'time_boot_ms', type: 'uint32_t' },
      { name: 'roll', type: 'float' },
      { name: 'pitch', type: 'float' },
      { name: 'yaw', type: 'float' },
      { name: 'rollspeed', type: 'float' },
      { name: 'pitchspeed', type: 'float' },
      { name: 'yawspeed', type: 'float' },
    ],
  },
  {
    id: 33,
    name: 'GLOBAL_POSITION_INT',
    fields: [
      { name: 'time_boot_ms', type: 'uint32_t' },
      { name: 'lat', type: 'int32_t' },
      { name: 'lon', type: 'int32_t' },
      { name: 'alt', type: 'int32_t' },
      { name: 'relative_alt', type: 'int32_t' },
      { name: 'vx', type: 'int16_t' },
      { name: 'vy', type: 'int16_t' },
      { name: 'vz', type: 'int16_t' },
      { name: 'hdg', type: 'uint16_t' },
    ],
  },
  {
    id: 36,
    name: 'SERVO_OUTPUT_RAW',
    fields: [
      { name: 'time_usec', type: 'uint32_t' },
      // `port` sits second in the dialect, not after `servo8_raw`. It makes no
      // difference to the wire — it is a `uint8_t` and gets sorted after the
      // sixteen-bit servos either way — and none to the CRC_EXTRA, which is a
      // function of the field set and not its order. It is fixed here because
      // this list claims to be declaration order, and because the harmless case
      // is only harmless by accident: two adjacent plain fields of the *same*
      // size would have their wire order decided by this position.
      { name: 'port', type: 'uint8_t' },
      { name: 'servo1_raw', type: 'uint16_t' },
      { name: 'servo2_raw', type: 'uint16_t' },
      { name: 'servo3_raw', type: 'uint16_t' },
      { name: 'servo4_raw', type: 'uint16_t' },
      { name: 'servo5_raw', type: 'uint16_t' },
      { name: 'servo6_raw', type: 'uint16_t' },
      { name: 'servo7_raw', type: 'uint16_t' },
      { name: 'servo8_raw', type: 'uint16_t' },
      { name: 'servo9_raw', type: 'uint16_t', ext: true },
      { name: 'servo10_raw', type: 'uint16_t', ext: true },
      { name: 'servo11_raw', type: 'uint16_t', ext: true },
      { name: 'servo12_raw', type: 'uint16_t', ext: true },
      { name: 'servo13_raw', type: 'uint16_t', ext: true },
      { name: 'servo14_raw', type: 'uint16_t', ext: true },
      { name: 'servo15_raw', type: 'uint16_t', ext: true },
      { name: 'servo16_raw', type: 'uint16_t', ext: true },
    ],
  },
  {
    id: 65,
    name: 'RC_CHANNELS',
    fields: [
      { name: 'time_boot_ms', type: 'uint32_t' },
      // Second in the dialect, like `SERVO_OUTPUT_RAW.port`. Same reasoning:
      // sorted onto the wire by size either way, so this costs nothing to get
      // right and would cost a silent byte shift to get wrong if a same-sized
      // field ever sat next to it.
      { name: 'chancount', type: 'uint8_t' },
      { name: 'chan1_raw', type: 'uint16_t' },
      { name: 'chan2_raw', type: 'uint16_t' },
      { name: 'chan3_raw', type: 'uint16_t' },
      { name: 'chan4_raw', type: 'uint16_t' },
      { name: 'chan5_raw', type: 'uint16_t' },
      { name: 'chan6_raw', type: 'uint16_t' },
      { name: 'chan7_raw', type: 'uint16_t' },
      { name: 'chan8_raw', type: 'uint16_t' },
      { name: 'chan9_raw', type: 'uint16_t' },
      { name: 'chan10_raw', type: 'uint16_t' },
      { name: 'chan11_raw', type: 'uint16_t' },
      { name: 'chan12_raw', type: 'uint16_t' },
      { name: 'chan13_raw', type: 'uint16_t' },
      { name: 'chan14_raw', type: 'uint16_t' },
      { name: 'chan15_raw', type: 'uint16_t' },
      { name: 'chan16_raw', type: 'uint16_t' },
      { name: 'chan17_raw', type: 'uint16_t' },
      { name: 'chan18_raw', type: 'uint16_t' },
      { name: 'rssi', type: 'uint8_t' },
    ],
  },
  {
    id: 74,
    name: 'VFR_HUD',
    fields: [
      { name: 'airspeed', type: 'float' },
      { name: 'groundspeed', type: 'float' },
      { name: 'heading', type: 'int16_t' },
      { name: 'throttle', type: 'uint16_t' },
      { name: 'alt', type: 'float' },
      { name: 'climb', type: 'float' },
    ],
  },
  {
    id: 76,
    name: 'COMMAND_LONG',
    fields: [
      { name: 'target_system', type: 'uint8_t' },
      { name: 'target_component', type: 'uint8_t' },
      { name: 'command', type: 'uint16_t' },
      { name: 'confirmation', type: 'uint8_t' },
      { name: 'param1', type: 'float' },
      { name: 'param2', type: 'float' },
      { name: 'param3', type: 'float' },
      { name: 'param4', type: 'float' },
      { name: 'param5', type: 'float' },
      { name: 'param6', type: 'float' },
      { name: 'param7', type: 'float' },
    ],
  },
  {
    id: 77,
    name: 'COMMAND_ACK',
    fields: [
      { name: 'command', type: 'uint16_t' },
      { name: 'result', type: 'uint8_t' },
      { name: 'progress', type: 'uint8_t', ext: true },
      { name: 'result_param2', type: 'int32_t', ext: true },
      { name: 'target_system', type: 'uint8_t', ext: true },
      { name: 'target_component', type: 'uint8_t', ext: true },
    ],
  },
  {
    id: 148,
    name: 'AUTOPILOT_VERSION',
    fields: [
      { name: 'capabilities', type: 'uint64_t' },
      { name: 'flight_sw_version', type: 'uint32_t' },
      { name: 'middleware_sw_version', type: 'uint32_t' },
      { name: 'os_sw_version', type: 'uint32_t' },
      { name: 'board_version', type: 'uint32_t' },
      { name: 'flight_custom_version', type: 'uint8_t', count: 8 },
      { name: 'middleware_custom_version', type: 'uint8_t', count: 8 },
      { name: 'os_custom_version', type: 'uint8_t', count: 8 },
      { name: 'vendor_id', type: 'uint16_t' },
      { name: 'product_id', type: 'uint16_t' },
      { name: 'uid', type: 'uint64_t' },
      { name: 'uid2', type: 'uint8_t', count: 18, ext: true },
    ],
  },
  {
    id: 253,
    name: 'STATUSTEXT',
    fields: [
      { name: 'severity', type: 'uint8_t' },
      { name: 'text', type: 'char', count: 50 },
      { name: 'id', type: 'uint16_t', ext: true },
      { name: 'chunk_seq', type: 'uint8_t', ext: true },
    ],
  },
];

export interface MavWireField extends MavFieldDef {
  readonly offset: number;
  /** Bytes this field occupies: one scalar, or `count` of them. */
  readonly size: number;
}

export interface MavWireLayout {
  readonly id: number;
  readonly name: string;
  readonly fields: readonly MavWireField[];
  /** The struct size. Not the frame's payload length — see v2 truncation. */
  readonly length: number;
}

/**
 * The wire order, derived rather than written down.
 *
 * The rule, from MAVLink's own generator: sort the non-extension fields by
 * type size descending, keeping declaration order among equal sizes, then
 * append the extension fields in declaration order. Arrays sort by their
 * *element* size, which is why a 50-byte `char` array can sit in the middle of
 * `STATUSTEXT` and a 16-byte `char` array can sit after two `uint16_t`s.
 *
 * Written down, this table would be fourteen chances to transcribe a byte
 * offset wrongly and produce a decoder that reads a plausible number from the
 * wrong field — the exact failure this app is built to avoid. Derived, it is
 * one rule that a test can hold against pymavlink's packing.
 */
function wireLayout(def: MavMessageDef): MavWireLayout {
  const ordinary = def.fields.filter((field) => field.ext !== true);
  const extensions = def.fields.filter((field) => field.ext === true);
  // `Array.prototype.sort` is stable as of ES2019, which the declaration-order
  // tiebreak depends on. Stated because it is load-bearing, not incidental.
  const ordered = [
    ...ordinary.slice().sort((a, b) => SORT_SIZE[b.type] - SORT_SIZE[a.type]),
    ...extensions,
  ];
  let offset = 0;
  const fields = ordered.map((field) => {
    const size = SCALAR_SIZE[field.type] * (field.count ?? 1);
    const placed: MavWireField = { ...field, offset, size };
    offset += size;
    return placed;
  });
  return { id: def.id, name: def.name, fields, length: offset };
}

const LAYOUT_BY_ID = new Map<number, MavWireLayout>(
  MAVLINK_MESSAGES.map((def) => [def.id, wireLayout(def)]),
);

export function mavLayout(msgid: number): MavWireLayout | undefined {
  return LAYOUT_BY_ID.get(msgid);
}

// ---- the checksum ----------------------------------------------------------

/**
 * One byte through CRC-16/MCRF4XX — X.25's polynomial, reflected, no final XOR.
 *
 * Kept as its own function because MAVLink's checksum is this function applied
 * to the frame *and then once more* to the message's CRC_EXTRA byte. An
 * implementation that appends the extra to the buffer and also passes it
 * separately counts it twice and no frame ever validates.
 */
export function crcAccumulate(crc: number, byte: number): number {
  let tmp = (byte & 0xff) ^ (crc & 0xff);
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}

/** The frame checksum: everything from the length byte to the payload, then the
 *  message's CRC_EXTRA, then the high and low bytes are XORed together. */
export function frameChecksum(body: Uint8Array, crcExtra: number): number {
  let crc = 0xffff;
  for (const byte of body) crc = crcAccumulate(crc, byte);
  crc = crcAccumulate(crc, crcExtra);
  return crc;
}

/** The CRC_EXTRA derivation, from the message definition and nothing else.
 *
 * Accumulate `NAME ` (no `MAVLink_` prefix), then for each **non-extension**
 * field `type `, `name `, and one byte of array length where the field is an
 * array; finally XOR the two halves of the accumulator.
 *
 * **In wire order, not declaration order.** This is the detail that makes the
 * check worth having: the fields are visited in the order `wireLayout` puts
 * them on the wire, which for `HEARTBEAT` means `custom_mode` first and for
 * `STATUSTEXT` means the 50-byte text in the middle. Deriving over declaration
 * order gives a different byte for eleven of the fourteen messages — measured,
 * not assumed — and every frame of those messages then fails its checksum with
 * no hint as to why.
 *
 * This exists so the table above is checkable. A wrong field order, a wrong
 * type, a wrong array length or a wrong extension flag all change this byte,
 * and the published value is in the test — so the table cannot drift from the
 * dialect without a test going red. */
export function deriveCrcExtra(def: MavMessageDef): number {
  let crc = 0xffff;
  for (const ch of `${def.name} `) crc = crcAccumulate(crc, ch.charCodeAt(0));
  for (const field of wireLayout(def).fields) {
    if (field.ext === true) continue;
    for (const ch of `${field.type} `) crc = crcAccumulate(crc, ch.charCodeAt(0));
    for (const ch of `${field.name} `) crc = crcAccumulate(crc, ch.charCodeAt(0));
    if (field.count !== undefined) crc = crcAccumulate(crc, field.count);
  }
  return ((crc & 0xff) ^ (crc >> 8)) & 0xff;
}

/** Every message's CRC_EXTRA, derived from the table above. */
export const CRC_EXTRA: ReadonlyMap<number, number> = new Map(
  MAVLINK_MESSAGES.map((def) => [def.id, deriveCrcExtra(def)]),
);

// ---- decoding --------------------------------------------------------------

export interface MavlinkFrame {
  /** 1 or 2. Which framing carried it. */
  readonly version: 1 | 2;
  readonly msgid: number;
  readonly name: string;
  readonly systemId: number;
  readonly componentId: number;
  readonly seq: number;
  /** The payload as it arrived: possibly shorter than the struct, because v2
   *  strips trailing zeros. `decode()` is what zero-pads. */
  readonly payload: Uint8Array;
  /** The struct size. `payload.length < structLength` is v2 truncation, which
   *  is a normal frame and not a fault. */
  readonly structLength: number;
  /** True when the frame carried a signature. It is **not verified** — this app
   *  has no key and says so rather than implying the frame is authenticated. */
  readonly signed: boolean;
}

export type MavIssue =
  | { readonly kind: 'checksum'; readonly msgid: number }
  /** A message whose CRC_EXTRA this app does not carry, so its checksum could
   *  not be verified and its length could not be trusted. */
  | { readonly kind: 'unknown-message'; readonly msgid: number }
  | { readonly kind: 'truncated-frame' };

export interface MavDecodeResult {
  readonly frames: MavlinkFrame[];
  readonly issues: MavIssue[];
}

/**
 * A byte-fed MAVLink decoder, v1 and v2.
 *
 * Fed whatever the transport produced, in whatever sizes it chose. It never
 * throws: a bad frame on a live link is a statistic, not an exception, and a
 * client that gave up on the first corrupted byte would be unusable on a real
 * UART. Same reasoning as the other two decoders in this directory.
 *
 * **Unknown messages are dropped, not skipped by their length byte.** Without a
 * CRC_EXTRA there is no way to tell a corrupted length byte from a real one, so
 * trusting it would mean skipping an arbitrary number of bytes on a corrupted
 * frame. One byte of resynchronisation finds the next STX, which is the
 * conservative choice and the one that keeps a stream of messages this app does
 * not know from derailing the ones it does.
 */
export class MavlinkDecoder {
  private buffer: number[] = [];
  private pending: MavlinkFrame[] = [];
  private problems: MavIssue[] = [];

  push(chunk: Uint8Array): MavDecodeResult {
    for (const byte of chunk) this.buffer.push(byte);
    this.scan();
    const frames = this.pending;
    const issues = this.problems;
    this.pending = [];
    this.problems = [];
    return { frames, issues };
  }

  /** Anything held half-received. A caller deciding "has this vehicle gone
   *  quiet" wants to know there are bytes stuck rather than none at all. */
  get held(): number {
    return this.buffer.length;
  }

  reset(): void {
    this.buffer.length = 0;
  }

  private scan(): void {
    for (;;) {
      let start = -1;
      for (let i = 0; i < this.buffer.length; i++) {
        const byte = this.buffer[i]!;
        if (byte === MAVLINK_STX_V1 || byte === MAVLINK_STX_V2) {
          start = i;
          break;
        }
      }
      if (start < 0) {
        this.buffer.length = 0;
        return;
      }
      if (start > 0) this.buffer.splice(0, start);

      const consumed = this.buffer[0] === MAVLINK_STX_V2 ? this.tryV2() : this.tryV1();
      if (consumed === 0) return; // need more bytes
      if (consumed < 0) this.buffer.shift(); // bad frame: resynchronise
    }
  }

  /** Returns bytes consumed, 0 for "need more", or -1 for "this one is bad". */
  private tryV1(): number {
    if (this.buffer.length < 2) return 0;
    const length = this.buffer[1]!;
    const total = 8 + length;
    if (this.buffer.length < total) return 0;

    const seq = this.buffer[2]!;
    const systemId = this.buffer[3]!;
    const componentId = this.buffer[4]!;
    const msgid = this.buffer[5]!;
    const extra = CRC_EXTRA.get(msgid);
    if (extra === undefined) {
      this.problems.push({ kind: 'unknown-message', msgid });
      return -1;
    }
    const payload = Uint8Array.from(this.buffer.slice(6, 6 + length));
    const body = Uint8Array.from(this.buffer.slice(1, 6 + length));
    const received = this.buffer[6 + length]! | (this.buffer[7 + length]! << 8);
    if (frameChecksum(body, extra) !== received) {
      this.problems.push({ kind: 'checksum', msgid });
      return -1;
    }
    this.buffer.splice(0, total);
    this.push_(1, msgid, systemId, componentId, seq, payload, false);
    return total;
  }

  private tryV2(): number {
    if (this.buffer.length < 3) return 0;
    const length = this.buffer[1]!;
    const incompat = this.buffer[2]!;
    const signed = (incompat & MAVLINK_IFLAG_SIGNED) !== 0;
    // A frame with an incompatibility flag this app does not implement cannot
    // be parsed at all: the flags mean the layout itself differs. Dropping one
    // byte is the only honest response, and it is reported.
    if ((incompat & ~MAVLINK_IFLAG_SIGNED) !== 0) {
      this.problems.push({ kind: 'truncated-frame' });
      return -1;
    }
    const total = 12 + length + (signed ? MAVLINK_SIGNATURE_LENGTH : 0);
    if (this.buffer.length < total) return 0;

    const seq = this.buffer[4]!;
    const systemId = this.buffer[5]!;
    const componentId = this.buffer[6]!;
    const msgid = this.buffer[7]! | (this.buffer[8]! << 8) | (this.buffer[9]! << 16);
    const extra = CRC_EXTRA.get(msgid);
    if (extra === undefined) {
      this.problems.push({ kind: 'unknown-message', msgid });
      return -1;
    }
    const payload = Uint8Array.from(this.buffer.slice(10, 10 + length));
    const body = Uint8Array.from(this.buffer.slice(1, 10 + length));
    const received = this.buffer[10 + length]! | (this.buffer[11 + length]! << 8);
    if (frameChecksum(body, extra) !== received) {
      this.problems.push({ kind: 'checksum', msgid });
      return -1;
    }
    this.buffer.splice(0, total);
    this.push_(2, msgid, systemId, componentId, seq, payload, signed);
    return total;
  }

  private push_(
    version: 1 | 2,
    msgid: number,
    systemId: number,
    componentId: number,
    seq: number,
    payload: Uint8Array,
    signed: boolean,
  ): void {
    const layout = LAYOUT_BY_ID.get(msgid)!;
    this.pending.push({
      version, msgid, name: layout.name, systemId, componentId, seq,
      payload, structLength: layout.length, signed,
    });
  }
}

// ---- the messages ----------------------------------------------------------

/** A decoded payload, keyed by the dialect's own field names.
 *
 * The names are the wire's, not this app's, and the values are raw: radians,
 * degrees×1e7, milliseconds — whatever the field is defined as. Unit conversion
 * belongs to whoever draws the number, and keeping it there is what lets the
 * fixture compare against these values byte for byte. */
export type DecodedMessage = Readonly<Record<string, number | string>>;

function readScalar(payload: Uint8Array, offset: number, type: MavFieldType): number {
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  switch (type) {
    case 'uint8_t': return view.getUint8(offset);
    case 'int8_t': return view.getInt8(offset);
    case 'uint16_t': return view.getUint16(offset, true);
    case 'int16_t': return view.getInt16(offset, true);
    case 'uint32_t': return view.getUint32(offset, true);
    case 'int32_t': return view.getInt32(offset, true);
    // A u64 is read through BigInt and narrowed. Every 64-bit field this app
    // decodes is a microsecond timestamp or an identifier, both far below
    // 2^53, so the narrowing is exact here — and it is stated rather than
    // assumed, because silently losing the low bits of a timestamp is the sort
    // of thing that produces a plausible wrong number.
    case 'uint64_t': return Number(view.getBigUint64(offset, true));
    case 'int64_t': return Number(view.getBigInt64(offset, true));
    case 'float': return view.getFloat32(offset, true);
    case 'double': return view.getFloat64(offset, true);
    case 'char': return view.getUint8(offset);
  }
}

export class MavProtocolError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'MavProtocolError';
  }
}

/**
 * One frame's payload as named values.
 *
 * The payload is zero-padded to the struct size first, so v2's truncated frames
 * decode exactly as the sender intended: a stripped trailing zero *is* a zero.
 * A payload longer than the struct is refused rather than truncated — that is
 * not a frame any version of this protocol produces, and quietly ignoring the
 * extra bytes would hide a layout mistake.
 */
export function decode(frame: MavlinkFrame): DecodedMessage {
  const layout = LAYOUT_BY_ID.get(frame.msgid);
  if (layout === undefined) {
    throw new MavProtocolError(`no definition for MAVLink message ${frame.msgid}`);
  }
  if (frame.payload.length > layout.length) {
    throw new MavProtocolError(
      `${layout.name}: a payload of ${frame.payload.length} bytes is longer than the ` +
        `${layout.length}-byte message`,
    );
  }
  const padded = new Uint8Array(layout.length);
  padded.set(frame.payload);

  const out: Record<string, number | string> = {};
  for (const field of layout.fields) {
    if (field.type === 'char') {
      if ((field.count ?? 1) > 1) {
        const slice = padded.subarray(field.offset, field.offset + field.size);
        const end = slice.indexOf(0);
        out[field.name] = new TextDecoder().decode(end < 0 ? slice : slice.subarray(0, end));
      } else {
        out[field.name] = String.fromCharCode(padded[field.offset]!);
      }
    } else if ((field.count ?? 1) > 1) {
      const values: number[] = [];
      for (let i = 0; i < field.count!; i++) {
        values.push(readScalar(padded, field.offset + i * SCALAR_SIZE[field.type], field.type));
      }
      out[field.name] = values.join(',');
    } else {
      out[field.name] = readScalar(padded, field.offset, field.type);
    }
  }
  return out;
}

/** Numbers out of a decoded message, for the fields that are numbers. */
export function num(message: DecodedMessage, name: string): number {
  const value = message[name];
  return typeof value === 'number' ? value : 0;
}

/** A string out of a decoded message. */
export function str(message: DecodedMessage, name: string): string {
  const value = message[name];
  return typeof value === 'string' ? value : '';
}

// ---- the commands this app may send ---------------------------------------

/** From `common.xml`. The only command this app will put on the wire. */
export const MAV_CMD_SET_MESSAGE_INTERVAL = 511;

/** `MAV_MODE_FLAG_SAFETY_ARMED`. The heartbeat's own word on whether the
 *  vehicle is armed, and the only one this app acts on. */
export const MAV_MODE_FLAG_SAFETY_ARMED = 128;
export const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;

export enum MavState {
  UNINIT = 0, BOOT = 1, CALIBRATING = 2, STANDBY = 3,
  ACTIVE = 4, CRITICAL = 5, EMERGENCY = 6, POWEROFF = 7, FLIGHT_TERMINATION = 8,
}

export enum MavResult {
  ACCEPTED = 0, TEMPORARILY_REJECTED = 1, DENIED = 2, UNSUPPORTED = 3,
  FAILED = 4, IN_PROGRESS = 5, CANCELLED = 6,
}

const AUTOPILOT_NAMES: Record<number, string> = {
  0: 'Generic autopilot',
  3: 'ArduPilot',
  4: 'OpenPilot',
  5: 'Generic autopilot (generic)',
  7: 'AutoQuad',
  8: 'Invalid',
  12: 'PX4',
  13: 'ARM',
  14: 'iNav',
  15: 'Paparazzi',
  16: 'MAVLink autopilot',
  17: 'MAVLink autopilot (generic)',
  18: 'Vertigo',
  19: 'ArduPilot (balloon)',
  20: 'ArduPilot (helicopter)',
  21: 'ArduPilot (plane)',
  22: 'ArduPilot (rover)',
  23: 'ArduPilot (sub)',
  24: 'ArduPilot (tracker)',
  25: 'ArduPilot (blimp)',
  26: 'ArduPilot (APPeriph)',
};

const VEHICLE_TYPE_NAMES: Record<number, string> = {
  0: 'generic', 1: 'fixed wing', 2: 'quadrotor', 3: 'coaxial helicopter',
  4: 'helicopter', 6: 'ground station', 7: 'airship', 10: 'ground rover',
  11: 'surface boat', 12: 'submarine', 13: 'hexarotor', 14: 'octorotor',
  15: 'tricopter', 16: 'flapping wing', 19: 'VTOL (two-rotor)',
  20: 'VTOL (quadrotor)', 21: 'VTOL (tiltrotor)', 22: 'VTOL (reserved)',
  29: 'drone (generic)', 30: 'drone (multirotor)',
};

export function autopilotName(code: number): string {
  return AUTOPILOT_NAMES[code] ?? `autopilot ${code}`;
}

export function vehicleTypeName(code: number): string {
  return VEHICLE_TYPE_NAMES[code] ?? `vehicle type ${code}`;
}

/**
 * What a heartbeat establishes, and what it does not.
 *
 * `armed` is the vehicle's own statement from `base_mode`. It is the only
 * armed state this app will show, and — as with the other two families — it
 * goes stale, because a heartbeat that stopped arriving is not a vehicle that
 * disarmed.
 */
export interface MavHeartbeat {
  readonly systemId: number;
  readonly componentId: number;
  readonly type: number;
  readonly typeName: string;
  readonly autopilot: number;
  readonly autopilotName: string;
  readonly baseMode: number;
  readonly customMode: number;
  /** True when `MAV_MODE_FLAG_SAFETY_ARMED` is set. */
  readonly armed: boolean;
  readonly systemStatus: number;
  /** The `mavlink_version` *field* of the message — which is 3 on every
   *  autopilot in service, because it is a protocol constant rather than a
   *  statement about the connection. Not to be confused with `framing`. */
  readonly mavlinkVersion: number;
  /** The framing that actually carried this heartbeat: 1 or 2. This is the
   *  version a person means when they say "the vehicle speaks MAVLink 2", and
   *  the one this app answers in. */
  readonly framing: 1 | 2;
}

export function heartbeatOf(frame: MavlinkFrame): MavHeartbeat {
  const message = decode(frame);
  const baseMode = num(message, 'base_mode');
  const type = num(message, 'type');
  const autopilot = num(message, 'autopilot');
  return {
    systemId: frame.systemId,
    componentId: frame.componentId,
    type,
    typeName: vehicleTypeName(type),
    autopilot,
    autopilotName: autopilotName(autopilot),
    baseMode,
    customMode: num(message, 'custom_mode'),
    armed: (baseMode & MAV_MODE_FLAG_SAFETY_ARMED) !== 0,
    systemStatus: num(message, 'system_status'),
    mavlinkVersion: num(message, 'mavlink_version'),
    framing: frame.version,
  };
}

// ---- typed messages --------------------------------------------------------
//
// The raw field names and units above are the wire's and stay that way, because
// that is what the fixture compares against. These are what the rest of the app
// uses: one place per message where the wire's units become the ones a person
// reads. Naming each field with its unit is not decoration — MAVLink mixes
// radians, degrees, degrees×1e7, centimetres, metres and centimetres per second
// across messages that sit next to each other on a screen, and the failure mode
// is a number that looks entirely plausible.

export interface MavAttitude {
  readonly timeBootMs: number;
  readonly rollDeg: number;
  readonly pitchDeg: number;
  /** Unwrapped by the autopilot, so this runs past ±180 rather than wrapping. */
  readonly yawDeg: number;
  readonly rollspeedDegS: number;
  readonly pitchspeedDegS: number;
  readonly yawspeedDegS: number;
}

const DEG = 180 / Math.PI;

export function attitudeOf(frame: MavlinkFrame): MavAttitude {
  const m = decode(frame);
  return {
    timeBootMs: num(m, 'time_boot_ms'),
    rollDeg: num(m, 'roll') * DEG,
    pitchDeg: num(m, 'pitch') * DEG,
    yawDeg: num(m, 'yaw') * DEG,
    rollspeedDegS: num(m, 'rollspeed') * DEG,
    pitchspeedDegS: num(m, 'pitchspeed') * DEG,
    yawspeedDegS: num(m, 'yawspeed') * DEG,
  };
}

export interface MavGlobalPosition {
  readonly timeBootMs: number;
  readonly latDeg: number;
  readonly lonDeg: number;
  /** Above mean sea level, metres. */
  readonly altM: number;
  /** Above the home position, metres. The one a pilot usually wants. */
  readonly relativeAltM: number;
  readonly vxCms: number;
  readonly vyCms: number;
  readonly vzCms: number;
  /** 0..359, or 65535 for "unknown" — which is not the same as north. */
  readonly hdgDeg: number | null;
}

export function globalPositionOf(frame: MavlinkFrame): MavGlobalPosition {
  const m = decode(frame);
  const hdg = num(m, 'hdg');
  return {
    timeBootMs: num(m, 'time_boot_ms'),
    latDeg: num(m, 'lat') / 1e7,
    lonDeg: num(m, 'lon') / 1e7,
    altM: num(m, 'alt') / 1000,
    relativeAltM: num(m, 'relative_alt') / 1000,
    vxCms: num(m, 'vx'),
    vyCms: num(m, 'vy'),
    vzCms: num(m, 'vz'),
    // UINT16_MAX is the protocol's "I do not know my heading", and it would
    // otherwise draw as 655.35 degrees.
    hdgDeg: hdg === 0xffff ? null : hdg / 100,
  };
}

const FIX_NAMES: Record<number, string> = {
  0: 'no fix', 1: 'no fix', 2: '2D fix', 3: '3D fix', 4: 'DGPS', 5: 'RTK float',
  6: 'RTK fixed', 7: 'static', 8: 'PPP',
};

export function fixTypeName(code: number): string {
  return FIX_NAMES[code] ?? `fix type ${code}`;
}

export interface MavGps {
  readonly timeUsec: number;
  readonly fixType: number;
  readonly fixName: string;
  readonly latDeg: number;
  readonly lonDeg: number;
  readonly altM: number;
  /** Horizontal and vertical dilution of precision, not accuracy in metres. */
  readonly eph: number;
  readonly epv: number;
  readonly velocityCms: number;
  readonly cogDeg: number;
  readonly satellitesVisible: number;
}

export function gpsOf(frame: MavlinkFrame): MavGps {
  const m = decode(frame);
  const fixType = num(m, 'fix_type');
  return {
    timeUsec: num(m, 'time_usec'),
    fixType,
    fixName: fixTypeName(fixType),
    latDeg: num(m, 'lat') / 1e7,
    lonDeg: num(m, 'lon') / 1e7,
    altM: num(m, 'alt') / 1000,
    eph: num(m, 'eph'),
    epv: num(m, 'epv'),
    velocityCms: num(m, 'vel'),
    cogDeg: num(m, 'cog') / 100,
    satellitesVisible: num(m, 'satellites_visible'),
  };
}

export interface MavSysStatus {
  readonly voltageV: number;
  /** Negative when the autopilot is not measuring it; -1 is the convention. */
  readonly currentA: number;
  /** -1 when the autopilot does not know. Not the same as empty. */
  readonly batteryRemainingPct: number | null;
  /** CPU load, per mille. */
  readonly loadPermille: number;
  readonly dropRateCommPermille: number;
  readonly errorsComm: number;
}

export function sysStatusOf(frame: MavlinkFrame): MavSysStatus {
  const m = decode(frame);
  const remaining = num(m, 'battery_remaining');
  return {
    voltageV: num(m, 'voltage_battery') / 1000,
    currentA: num(m, 'current_battery') / 100,
    batteryRemainingPct: remaining < 0 ? null : remaining,
    loadPermille: num(m, 'load'),
    dropRateCommPermille: num(m, 'drop_rate_comm'),
    errorsComm: num(m, 'errors_comm'),
  };
}

export interface MavVfrHud {
  readonly airspeedMs: number;
  readonly groundspeedMs: number;
  readonly altM: number;
  readonly climbMs: number;
  readonly headingDeg: number;
  readonly throttlePct: number;
}

export function vfrHudOf(frame: MavlinkFrame): MavVfrHud {
  const m = decode(frame);
  return {
    airspeedMs: num(m, 'airspeed'),
    groundspeedMs: num(m, 'groundspeed'),
    altM: num(m, 'alt'),
    climbMs: num(m, 'climb'),
    headingDeg: num(m, 'heading'),
    throttlePct: num(m, 'throttle'),
  };
}

export interface MavRcChannels {
  /** Only the channels `chancount` says exist, in order. The rest of the 18
   *  fields are zero because the vehicle does not have them, and drawing
   *  eighteen bars with fourteen at zero would be a lie about the airframe. */
  readonly channels: readonly number[];
  readonly chancount: number;
  readonly rssi: number;
}

export function rcChannelsOf(frame: MavlinkFrame): MavRcChannels {
  const m = decode(frame);
  const chancount = num(m, 'chancount');
  const channels: number[] = [];
  for (let i = 1; i <= 18; i++) channels.push(num(m, `chan${i}_raw`));
  return { channels: channels.slice(0, Math.max(0, Math.min(18, chancount))), chancount, rssi: num(m, 'rssi') };
}

export interface MavServoOutput {
  readonly servos: readonly number[];
  readonly port: number;
}

export function servoOutputOf(frame: MavlinkFrame): MavServoOutput {
  const m = decode(frame);
  const servos: number[] = [];
  for (let i = 1; i <= 16; i++) servos.push(num(m, `servo${i}_raw`));
  return { servos, port: num(m, 'port') };
}

export interface MavParamValue {
  readonly paramId: string;
  readonly paramValue: number;
  readonly paramType: number;
  /** How many the vehicle says there are in total. */
  readonly paramCount: number;
  readonly paramIndex: number;
}

export function paramValueOf(frame: MavlinkFrame): MavParamValue {
  const m = decode(frame);
  return {
    paramId: str(m, 'param_id'),
    paramValue: num(m, 'param_value'),
    paramType: num(m, 'param_type'),
    paramCount: num(m, 'param_count'),
    paramIndex: num(m, 'param_index'),
  };
}

const SEVERITY_NAMES: Record<number, string> = {
  0: 'emergency', 1: 'alert', 2: 'critical', 3: 'error',
  4: 'warning', 5: 'notice', 6: 'info', 7: 'debug',
};

export function severityName(code: number): string {
  return SEVERITY_NAMES[code] ?? `severity ${code}`;
}

export interface MavStatusText {
  readonly severity: number;
  readonly severityName: string;
  readonly text: string;
  readonly id: number;
  readonly chunkSeq: number;
}

export function statusTextOf(frame: MavlinkFrame): MavStatusText {
  const m = decode(frame);
  const severity = num(m, 'severity');
  return {
    severity,
    severityName: severityName(severity),
    text: str(m, 'text'),
    id: num(m, 'id'),
    chunkSeq: num(m, 'chunk_seq'),
  };
}

export interface MavCommandAck {
  readonly command: number;
  readonly result: number;
  readonly resultName: string;
  readonly progress: number;
  readonly resultParam2: number;
}

const RESULT_NAMES: Record<number, string> = {
  0: 'accepted', 1: 'temporarily rejected', 2: 'denied', 3: 'unsupported',
  4: 'failed', 5: 'in progress', 6: 'cancelled',
};

export function resultName(code: number): string {
  return RESULT_NAMES[code] ?? `result ${code}`;
}

export function commandAckOf(frame: MavlinkFrame): MavCommandAck {
  const m = decode(frame);
  const result = num(m, 'result');
  return {
    command: num(m, 'command'),
    result,
    resultName: resultName(result),
    progress: num(m, 'progress'),
    resultParam2: num(m, 'result_param2'),
  };
}

/** `flight_sw_version` and its siblings are packed: major, minor, patch, and a
 *  release type in the low byte. `0` in the low byte means a released build;
 *  anything else is a development build of that type. */
export function formatVersion(raw: number): string {
  if (raw === 0) return '';
  const major = (raw >>> 24) & 0xff;
  const minor = (raw >>> 16) & 0xff;
  const patch = (raw >>> 8) & 0xff;
  const type = raw & 0xff;
  const suffix = type === 0 ? '' : `-dev(${type})`;
  return `${major}.${minor}.${patch}${suffix}`;
}

export interface MavAutopilotVersion {
  readonly capabilities: number;
  readonly flightSwVersion: string;
  readonly middlewareSwVersion: string;
  readonly osSwVersion: string;
  readonly boardVersion: string;
  readonly vendorId: number;
  readonly productId: number;
  readonly uid: string;
}

function numericArray(text: string): readonly number[] {
  if (text === '') return [];
  return text.split(',').map((part) => Number(part));
}

export function autopilotVersionOf(frame: MavlinkFrame): MavAutopilotVersion {
  const m = decode(frame);
  const custom = (name: string): string => {
    const values = numericArray(str(m, name)).filter((value) => value !== 0);
    return values.length === 0 ? '' : values.join('.');
  };
  return {
    capabilities: num(m, 'capabilities'),
    // The `flight_sw_version` word is the authority; the eight-byte custom
    // version is a git hash on most builds and is shown only when it is not
    // empty, because eight zero bytes is not a version.
    flightSwVersion: formatVersion(num(m, 'flight_sw_version')) || custom('flight_custom_version'),
    middlewareSwVersion: formatVersion(num(m, 'middleware_sw_version')) || custom('middleware_custom_version'),
    osSwVersion: formatVersion(num(m, 'os_sw_version')) || custom('os_custom_version'),
    boardVersion: formatVersion(num(m, 'board_version')),
    vendorId: num(m, 'vendor_id'),
    productId: num(m, 'product_id'),
    uid: num(m, 'uid').toString(16),
  };
}

// ---- frame building --------------------------------------------------------

/**
 * Wraps a payload in a frame, in the dialect the vehicle was heard speaking.
 *
 * **Not exported.** Every message this app can put on the wire goes through the
 * client below, which offers exactly two of them; a general builder on the
 * module's surface would be the general `COMMAND_LONG` this file exists to not
 * have.
 *
 * Trailing zeros are stripped for v2, which is what a real implementation does
 * and what makes a `PARAM_REQUEST_LIST` nine bytes of frame rather than eleven.
 * A v1 frame cannot be truncated — v1 predates the feature — so it is padded.
 */
function buildFrame(
  msgid: number,
  payload: Uint8Array,
  version: 1 | 2,
  seq: number,
  systemId: number,
  componentId: number,
): Uint8Array {
  const extra = CRC_EXTRA.get(msgid);
  if (extra === undefined) {
    throw new MavProtocolError(`no definition for MAVLink message ${msgid}`);
  }
  if (payload.length > MAVLINK_MAX_PAYLOAD) {
    throw new RangeError(`a payload of ${payload.length} bytes does not fit a MAVLink frame`);
  }

  if (version === 1) {
    if (msgid > 0xff) throw new MavProtocolError(`message ${msgid} does not fit a v1 frame`);
    const body = new Uint8Array(5 + payload.length);
    body[0] = payload.length;
    body[1] = seq & 0xff;
    body[2] = systemId & 0xff;
    body[3] = componentId & 0xff;
    body[4] = msgid;
    body.set(payload, 5);
    return assemble(MAVLINK_STX_V1, body, extra);
  }

  // v2: strip the trailing zeros the receiver will put back. The length byte is
  // what the receiver trusts, so a shorter payload here is the same message.
  let end = payload.length;
  while (end > 0 && payload[end - 1] === 0) end--;
  const trimmed = payload.subarray(0, end);
  const body = new Uint8Array(9 + trimmed.length);
  body[0] = trimmed.length;
  body[1] = 0; // incompat
  body[2] = 0; // compat
  body[3] = seq & 0xff;
  body[4] = systemId & 0xff;
  body[5] = componentId & 0xff;
  body[6] = msgid & 0xff;
  body[7] = (msgid >> 8) & 0xff;
  body[8] = (msgid >> 16) & 0xff;
  body.set(trimmed, 9);
  return assemble(MAVLINK_STX_V2, body, extra);
}

function assemble(stx: number, body: Uint8Array, extra: number): Uint8Array {
  const out = new Uint8Array(body.length + 3);
  out[0] = stx;
  out.set(body, 1);
  const checksum = frameChecksum(body, extra);
  out[body.length + 1] = checksum & 0xff;
  out[body.length + 2] = (checksum >> 8) & 0xff;
  return out;
}

/** Writes into a struct-sized buffer at the given offsets. */
class PayloadBuilder {
  private readonly bytes: Uint8Array;
  private readonly view: DataView;

  constructor(readonly msgid: number) {
    const layout = LAYOUT_BY_ID.get(msgid);
    if (layout === undefined) throw new MavProtocolError(`no definition for MAVLink message ${msgid}`);
    this.bytes = new Uint8Array(layout.length);
    this.view = new DataView(this.bytes.buffer);
  }

  u8(name: string, value: number): this { return this.set(name, 'uint8_t', value); }
  u16(name: string, value: number): this { return this.set(name, 'uint16_t', value); }
  f32(name: string, value: number): this { return this.set(name, 'float', value); }

  private set(name: string, type: MavFieldType, value: number): this {
    const layout = LAYOUT_BY_ID.get(this.msgid)!;
    const field = layout.fields.find((item) => item.name === name);
    if (field === undefined) {
      throw new MavProtocolError(`${layout.name} has no field ${name}`);
    }
    if (field.type !== type) {
      throw new MavProtocolError(`${layout.name}.${name} is ${field.type}, not ${type}`);
    }
    switch (type) {
      case 'uint8_t': this.view.setUint8(field.offset, value); break;
      case 'uint16_t': this.view.setUint16(field.offset, value, true); break;
      case 'float': this.view.setFloat32(field.offset, value, true); break;
      default: throw new MavProtocolError(`cannot write a ${type} yet`);
    }
    return this;
  }

  done(): Uint8Array {
    return this.bytes;
  }
}

// ---- the client ------------------------------------------------------------

/** The byte pipe a MAVLink client needs. The same shape the other two take, so
 *  a transport never has to know which protocol is about to speak through it —
 *  which is what makes firmware detection possible at all. */
export interface MavLink {
  write(bytes: Uint8Array): void | Promise<void>;
  onData(handler: (chunk: Uint8Array) => void): () => void;
  onClose(handler: (reason: string) => void): () => void;
  onError(handler: (message: string) => void): () => void;
}

export interface MavClientOptions {
  /** This app's own system id. A ground station is 255 by convention. */
  readonly systemId?: number;
  readonly componentId?: number;
  /** How long to wait for a heartbeat before deciding nothing is there. A
   *  vehicle heartbeats at 1 Hz by default, so this is a second and a bit, not
   *  a probe deadline. */
  readonly heartbeatMs?: number;
}

/**
 * MAVLink, from the ground station's side.
 *
 * **Two writes, and no third.** The vocabulary here is `requestParameterList`
 * and `requestMessageInterval`, plus reads of what the vehicle streams. There is
 * no `command()` and no way to reach `COMMAND_LONG` with a command of the
 * caller's choosing, because the same message carries arm, takeoff and
 * `DO_SET_HOME`. That is a property of the type rather than of a flag a caller
 * could forget to check, and a test asserts it by recording what reaches the
 * wire.
 *
 * Unlike the other two clients there is no request queue and no correlation: a
 * MAVLink vehicle answers a command with a `COMMAND_ACK` that names the command
 * and a parameter request with `PARAM_VALUE` frames that name their index.
 * Nothing is tied to a request by arrival order, so requests are simply sent —
 * and `waitFor` is what a caller uses when it wants to know one arrived.
 */
export class MavlinkClient {
  private readonly decoder = new MavlinkDecoder();
  private readonly unsubscribes: Array<() => void> = [];
  private closed = false;
  private sequence = 0;
  private targetSystem = 0;
  private targetComponent = 0;
  /** The framing the vehicle was heard using. A v1 vehicle is answered in v1:
   *  replying in v2 to something that spoke v1 is assuming a capability that
   *  was not demonstrated. */
  private version: 1 | 2 = 2;
  private heardHeartbeat = false;
  private heartbeatWaiters: Array<(heartbeat: MavHeartbeat) => void> = [];

  /** Counted rather than guessed at, so a lossy cable and an unfamiliar vehicle
   *  are both numbers on a screen. */
  readonly stats = { frames: 0, issues: 0, signed: 0, unreadMessages: 0, sent: 0 };
  /** Message ids seen, by name where this app knows one — including ids it has
   *  no definition for, which are counted but never decoded. The honest shape of
   *  "what is this vehicle actually saying". */
  readonly seen = new Map<number, number>();

  private readonly options: Required<MavClientOptions>;
  private readonly listeners = new Set<(frame: MavlinkFrame) => void>();
  private readonly issueListeners = new Set<(issue: MavIssue) => void>();

  constructor(
    private readonly link: MavLink,
    options: MavClientOptions = {},
  ) {
    this.options = {
      systemId: options.systemId ?? 255,
      componentId: options.componentId ?? 190,
      heartbeatMs: options.heartbeatMs ?? 1500,
    };
    this.unsubscribes.push(link.onData((chunk) => this.onData(chunk)));
    this.unsubscribes.push(
      link.onClose(() => {
        this.closed = true;
      }),
    );
  }

  /** Every frame the vehicle sent. The vehicle decides when to talk; this app
   *  only decides what to keep. */
  onFrame(handler: (frame: MavlinkFrame) => void): () => void {
    this.listeners.add(handler);
    return () => this.listeners.delete(handler);
  }

  /**
   * Every frame the decoder refused, and why.
   *
   * Most of these arrive without a frame to carry them: an unknown message is
   * dropped rather than delivered, and a bad checksum means there is nothing
   * trustworthy to hand on. So a caller that only counted issues when a frame
   * arrived would freeze its counters exactly when the link went bad — the one
   * moment it is watching them for.
   */
  onIssue(handler: (issue: MavIssue) => void): () => void {
    this.issueListeners.add(handler);
    return () => this.issueListeners.delete(handler);
  }

  /**
   * Bytes heard before this client existed.
   *
   * Detection has to listen for a heartbeat before it can know a vehicle is on
   * the link at all, and those are the same bytes this client would otherwise
   * wait a whole heartbeat period to hear again. They are pushed through the
   * same path as live bytes — same decoder, same checksum, same counting — so a
   * replayed frame is a frame this client verified, not a summary it was asked
   * to believe.
   */
  replay(chunks: readonly Uint8Array[]): void {
    for (const chunk of chunks) this.onData(chunk);
  }

  /** Waits for the vehicle to announce itself.
   *
   * This is the only way a MAVLink conversation can start. A vehicle heartbeats
   * unasked; nothing this app could send would be safe to send before it knows
   * what is on the other end. */
  waitForHeartbeat(timeoutMs = this.options.heartbeatMs): Promise<MavHeartbeat> {
    if (this.lastHeartbeat !== null) return Promise.resolve(this.lastHeartbeat);
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.heartbeatWaiters = this.heartbeatWaiters.filter((item) => item !== onBeat);
        reject(new MavTimeoutError('no heartbeat', timeoutMs));
      }, timeoutMs);
      const onBeat = (heartbeat: MavHeartbeat): void => {
        clearTimeout(timer);
        resolve(heartbeat);
      };
      this.heartbeatWaiters.push(onBeat);
    });
  }

  private lastHeartbeat: MavHeartbeat | null = null;

  /** Asks the vehicle for its whole parameter table.
   *
   * The one request that *does* enumerate parameters, unlike MSP — a MAVLink
   * vehicle answers with a `PARAM_VALUE` per index, so this app reads a table
   * here where on a Betaflight board it can only ask after a name it already
   * knows.
   *
   * Nothing is sent until a heartbeat has been heard, because the target system
   * and component in the request are the vehicle's own ids and there is no way
   * to learn them except by listening. */
  requestParameterList(): void {
    this.requireTarget('PARAM_REQUEST_LIST');
    const payload = new PayloadBuilder(21)
      .u8('target_system', this.targetSystem)
      .u8('target_component', this.targetComponent)
      .done();
    this.send(21, payload);
  }

  /**
   * Asks the vehicle to stream one message at a given rate.
   *
   * This is a write — it changes what the vehicle sends — so it is stated as one
   * rather than dressed up as a read. It is not a flight action: it changes a
   * telemetry rate and nothing about how the aircraft flies, and every ground
   * station does it on connect. It is restricted to the messages this app can
   * decode, because asking a vehicle to stream something this app cannot read
   * would spend a serial link's bandwidth on nothing.
   */
  requestMessageInterval(msgid: number, hz: number): void {
    this.requireTarget('COMMAND_LONG');
    if (LAYOUT_BY_ID.get(msgid) === undefined) {
      throw new MavProtocolError(
        `refusing to request message ${msgid}: this app carries no definition for it, ` +
          'so it could not read what came back',
      );
    }
    if (!(hz > 0) || hz > 1000) {
      throw new RangeError(`a stream rate of ${hz} Hz is not a rate this app will ask for`);
    }
    const intervalUs = Math.round(1e6 / hz);
    const payload = new PayloadBuilder(76)
      .u8('target_system', this.targetSystem)
      .u8('target_component', this.targetComponent)
      .u16('command', MAV_CMD_SET_MESSAGE_INTERVAL)
      .u8('confirmation', 0)
      .f32('param1', msgid) // the message id
      .f32('param2', intervalUs) // microseconds between frames; -1 disables
      .f32('param3', 0)
      .f32('param4', 0)
      .f32('param5', 0)
      .f32('param6', 0)
      .f32('param7', 0)
      .done();
    this.send(76, payload);
  }

  close(): void {
    this.closed = true;
    for (const off of this.unsubscribes) off();
    this.unsubscribes.length = 0;
    this.listeners.clear();
    this.issueListeners.clear();
  }

  private requireTarget(what: string): void {
    if (this.closed) throw new MavLinkClosedError();
    if (!this.heardHeartbeat) {
      throw new MavProtocolError(
        `refusing to send ${what} before hearing a heartbeat: a MAVLink request is addressed ` +
          'to a system id that only the vehicle can tell us',
      );
    }
  }

  private send(msgid: number, payload: Uint8Array): void {
    const frame = buildFrame(
      msgid, payload, this.version, this.sequence++,
      this.options.systemId, this.options.componentId,
    );
    this.stats.sent++;
    void this.link.write(frame);
  }

  private onData(chunk: Uint8Array): void {
    const { frames, issues } = this.decoder.push(chunk);
    for (const issue of issues) {
      this.stats.issues++;
      if (issue.kind === 'unknown-message') {
        this.stats.unreadMessages++;
        // Counted here even though it was not decoded, because "what is this
        // vehicle saying" is a question about the wire and not about this app's
        // message table. A vehicle streaming something unreadable is the single
        // most useful thing a person can learn from this list.
        this.seen.set(issue.msgid, (this.seen.get(issue.msgid) ?? 0) + 1);
      }
    }
    if (issues.length > 0) {
      for (const listener of this.issueListeners) {
        for (const issue of issues) listener(issue);
      }
    }    for (const frame of frames) {
      this.stats.frames++;
      this.seen.set(frame.msgid, (this.seen.get(frame.msgid) ?? 0) + 1);
      if (frame.signed) this.stats.signed++;
      // The first frame heard fixes the dialect for everything this app sends
      // afterwards, and the heartbeat is what tells us who to address.
      if (!this.heardHeartbeat) this.version = frame.version;
      if (frame.msgid === 0) {
        try {
          const heartbeat = heartbeatOf(frame);
          this.lastHeartbeat = heartbeat;
          this.heardHeartbeat = true;
          this.targetSystem = heartbeat.systemId;
          this.targetComponent = heartbeat.componentId;
          const waiters = this.heartbeatWaiters;
          this.heartbeatWaiters = [];
          for (const waiter of waiters) waiter(heartbeat);
        } catch {
          // A heartbeat this app cannot parse is not a heartbeat.
          this.stats.issues++;
        }
      }
      for (const listener of this.listeners) listener(frame);
    }
  }
}

/** The vehicle did not say the thing that was waited for. An ordinary result of
 *  asking a vehicle something it does not answer, not an exception a caller has
 *  to guard against to detect firmware. */
export class MavTimeoutError extends Error {
  constructor(what: string, ms: number) {
    super(`the vehicle did not send ${what} within ${ms} ms`);
    this.name = 'MavTimeoutError';
  }
}

export class MavLinkClosedError extends Error {
  constructor() {
    super('the connection closed');
    this.name = 'MavLinkClosedError';
  }
}

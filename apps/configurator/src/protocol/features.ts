/**
 * The capability word `hello` carries, and the one rule for reading it.
 *
 * A board's problem is not "which protocol version are you" — that is one byte
 * and it is in every frame — it is "do you answer the command I am about to
 * send". Until this word existed the only way to find out was to send it and
 * wait out a timeout, on a link that carries someone's configuration.
 *
 * **The rule: absent is not zero.** `null` means the reply ended before the
 * field — this firmware predates it — and that is a different claim from a
 * board that reports a word with no bits set. The first says "this app cannot
 * tell what you support"; the second says "you support none of it". Rendering
 * them the same way would be this app inventing a capability reading out of a
 * missing byte, which is the exact class of thing it exists not to do.
 */

/** The bits, from `AK_PROTO_FEATURE_*` in `aerialkit/src/core/ak_proto.h`. */
export const enum Feature {
  PARAM_INFO = 1 << 0,
  PARAM_DEFAULT = 1 << 1,
  /** A successful set makes the board re-apply its configuration. */
  APPLIES_ON_WRITE = 1 << 2,
  /** Every write path refuses while the aircraft is armed. */
  GATES_ON_ARMED = 1 << 3,
  RC_CHANNELS = 1 << 4,
  SENSOR_INFO = 1 << 5,
  OUTPUT_INFO = 1 << 6,
  OUTPUT_TEST = 1 << 7,
  LOG_STREAM = 1 << 8,
  PREFLIGHT = 1 << 9,
  CALIBRATE = 1 << 10,
  MISSION = 1 << 11,
}

/**
 * A board's capability word, or `null` when its `hello` ended before the
 * field. Not `number | null` spelled inline everywhere, because the difference
 * between the two is the whole point of the type and it is worth a name.
 */
export type Capabilities = number | null;

/** Whether the board claims this capability. False for an absent word, and the
 *  caller that needs to tell the two apart asks `capabilities === null`. */
export function has(capabilities: Capabilities, feature: Feature): boolean {
  return capabilities !== null && (capabilities & feature) !== 0;
}

/** The `AK_PROTO_FEATURE_*` names, for the diagnostics list. */
export const FEATURE_NAMES: ReadonlyArray<{ readonly bit: Feature; readonly name: string }> = [
  { bit: Feature.PARAM_INFO, name: 'PARAM_INFO' },
  { bit: Feature.PARAM_DEFAULT, name: 'PARAM_DEFAULT' },
  { bit: Feature.APPLIES_ON_WRITE, name: 'APPLIES_ON_WRITE' },
  { bit: Feature.GATES_ON_ARMED, name: 'GATES_ON_ARMED' },
  { bit: Feature.RC_CHANNELS, name: 'RC_CHANNELS' },
  { bit: Feature.SENSOR_INFO, name: 'SENSOR_INFO' },
  { bit: Feature.OUTPUT_INFO, name: 'OUTPUT_INFO' },
  { bit: Feature.OUTPUT_TEST, name: 'OUTPUT_TEST' },
  { bit: Feature.LOG_STREAM, name: 'LOG_STREAM' },
  { bit: Feature.PREFLIGHT, name: 'PREFLIGHT' },
  { bit: Feature.CALIBRATE, name: 'CALIBRATE' },
  { bit: Feature.MISSION, name: 'MISSION' },
];

export function featureNames(capabilities: Capabilities): readonly string[] {
  if (capabilities === null) return [];
  return FEATURE_NAMES.filter((entry) => (capabilities & entry.bit) !== 0).map(
    (entry) => entry.name,
  );
}

/**
 * Why this board cannot do the thing a capability names — or `null` when it
 * can. One function, so every tab, panel and limitation that gates on a
 * capability says it the same way and names the opcode rather than the bit:
 * a person reading "does not answer `rc channels`" knows what to search for in
 * the firmware, and a person reading "missing bit 4" does not.
 */
export function reasonFor(
  capabilities: Capabilities,
  feature: Feature,
  opcode: string,
): string | null {
  if (capabilities === null) {
    return (
      `this board's firmware answered \`hello\` without a capability word, so this app ` +
      `cannot tell whether it answers \`${opcode}\` — it predates the field`
    );
  }
  if ((capabilities & feature) === 0) {
    return `this board's firmware does not answer \`${opcode}\``;
  }
  return null;
}

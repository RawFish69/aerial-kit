import { AerialKitClient, type ByteLink } from './client';
import {
  MavlinkDecoder,
  autopilotName,
  heartbeatOf,
  vehicleTypeName,
  type MavHeartbeat,
} from './mavlink';
import type { Hello } from './messages';
import { MspClient, MspCommand } from './msp';

/**
 * Which firmware is on the other end.
 *
 * This app has one protocol of its own and has to share a serial port with
 * boards that speak other ones. That makes "what is this?" the first question,
 * and it has to be asked in the right order.
 *
 * **The order is not cosmetic.** A flight controller's serial port carries the
 * console *and* the protocol, and a board's console parser reads whatever
 * arrives. A window that greeted every board with somebody else's protocol
 * would be typing at its own console — and on a board whose console takes
 * commands, typing is doing. So:
 *
 *   1. **AerialKit first.** Our own hello, which is the only greeting whose
 *      reply is unambiguous and the only board this app may write to.
 *   2. **MSP second**, and only because it was silent to the first. A board that
 *      answered our hello is ours, and is not asked again.
 *   3. **MAVLink last, and not probed at all.** It is *passive*: a vehicle
 *      heartbeats on its own, unasked. There is nothing to send and nothing
 *      this app could send safely — a MAVLink request is addressed to a system
 *      id that only the vehicle can tell us. So this step listens, and a
 *      heartbeat that arrives is both the identification and the address.
 *
 * Every probe is a **read**. Nothing here writes a parameter, and nothing here
 * can: a board this app cannot identify is a board whose parameter semantics it
 * does not know, and the design's rule is that parameter writes to somebody
 * else's firmware are how a tool crashes an aircraft. The one place this app
 * speaks first on a foreign link is step 3, after the vehicle has spoken.
 */

export type FirmwareFamily = 'aerialkit' | 'msp' | 'mavlink' | 'unrecognised' | 'silent';

/** What the passive MAVLink step heard.
 *
 *  `heard` is the raw bytes, kept so the board that is opened next does not have
 *  to wait for the vehicle to come round again. Same bytes, decoded once more by
 *  a decoder that starts empty — not a summary that could disagree with them. */
export interface MavDetection {
  /** The vehicle's own announcement. Null when CRC-valid frames arrived but no
   *  heartbeat did within the window — unusual, and reported rather than
   *  rounded to either "recognised" or "nothing there". */
  readonly heartbeat: MavHeartbeat | null;
  /** How many frames passed their checksum while listening. */
  readonly frames: number;
  readonly heard: readonly Uint8Array[];
}

export interface Detection {
  readonly family: FirmwareFamily;
  /** What was asked and what came back, for the interface and the log. */
  readonly detail: string;
  /** For `unrecognised`: the first byte nobody claimed, so the screen can say
   *  what it saw instead of only that it saw something. */
  readonly firstByte: number | null;
  /** For `mavlink`: what was heard before the app knew what it was hearing. */
  readonly mavlink?: MavDetection;
}

export interface DetectOptions {
  /**
   * How long each probe waits for an answer.
   *
   * Short by default. A board that does not speak a protocol answers with
   * silence, not with a refusal, so `timeout` is the *normal* outcome of a
   * probe that misses — and two misses in a row is what a person waits through
   * before the app tells them what it found.
   */
  readonly probeMs?: number;
  /** How long to listen for a vehicle that talks on its own. */
  readonly listenMs?: number;
}

export const DEFAULT_PROBE_MS = 600;

/**
 * Longer than the probes, and deliberately.
 *
 * A vehicle heartbeats on a period of its own — ArduPilot's default is 1 Hz —
 * and this step can only start listening at an arbitrary moment, so a window
 * shorter than one period would miss the vehicle most of the time and report a
 * working flight controller as a dead cable. 1500 ms covers a 1 Hz stream from
 * any phase, and the cost of being wrong in the other direction is half a second
 * of waiting on a board that was never going to answer anyway.
 */
export const DEFAULT_LISTEN_MS = 1500;

export async function detectFirmware(
  link: ByteLink,
  options: DetectOptions = {},
): Promise<Detection> {
  const probeMs = options.probeMs ?? DEFAULT_PROBE_MS;

  // Asked twice before moving on. The first hello can land on a console that is
  // mid-line - a half-typed command, or a modem prober's AT that arrived just
  // before the port was ours - and a second costs one more deadline on a board
  // that is not ours, which is cheap against calling ours silent.
  const ours = (await probeAerialKit(link, probeMs)) ?? (await probeAerialKit(link, probeMs));
  if (ours !== null) {
    return {
      family: 'aerialkit',
      detail: `${ours.product} answered the AerialKit hello on protocol version ${ours.protocolVersion}`,
      firstByte: null,
    };
  }

  const theirs = await probeMsp(link, probeMs);
  if (theirs !== null) {
    const name = theirs.variant.known ? theirs.variant.variant : `${theirs.variant.variant} (unrecognised)`;
    return {
      family: 'msp',
      detail: `${name} answered MSP ${theirs.api.apiMajor}.${theirs.api.apiMinor} — read-only`,
      firstByte: null,
    };
  }

  // Nothing answered, so this is not a board that answers. Whether anything was
  // *heard* is a different question, and it is the one that tells a person
  // their cable works.
  const heard = await listen(link, options.listenMs ?? DEFAULT_LISTEN_MS);
  if (heard.firstByte === null) {
    return {
      family: 'silent',
      detail: `nothing answered the AerialKit hello or MSP within ${probeMs} ms, and nothing was heard afterwards`,
      firstByte: null,
    };
  }

  if (heard.mavlink !== null) {
    const beat = heard.mavlink.heartbeat;
    const what =
      beat === null
        ? `${heard.mavlink.frames} MAVLink frames passed their checksum but none of them was a heartbeat`
        : `${autopilotName(beat.autopilot)} on a ${vehicleTypeName(beat.type)}, system ${beat.systemId}, ` +
          `MAVLink v${beat.framing}, ${beat.armed ? 'ARMED' : 'disarmed'}`;
    return {
      family: 'mavlink',
      detail: `${what} — read-only`,
      firstByte: null,
      mavlink: heard.mavlink,
    };
  }

  return {
    family: 'unrecognised',
    detail:
      'this link is talking, but not in a protocol this app implements: not the AerialKit ' +
      'protocol, not MSP, and no MAVLink frame on it passed its checksum',
    firstByte: heard.firstByte,
  };
}

async function probeAerialKit(
  link: ByteLink,
  probeMs: number,
): Promise<Hello | null> {
  const client = new AerialKitClient(link, { timeoutMs: probeMs });
  try {
    return await client.hello();
  } catch {
    // Silence, a timeout, or a frame that was not a hello. All three mean the
    // same thing here: this is not our board. The reason is not reported
    // because at this point it is not yet known to be a fault.
    return null;
  } finally {
    // Closed either way. Two clients listening to one link would each parse the
    // other's replies, and the probe that lost would keep counting frames it
    // has no business seeing.
    client.close();
  }
}

async function probeMsp(
  link: ByteLink,
  probeMs: number,
): Promise<Awaited<ReturnType<MspClient['identify']>> | null> {
  const client = new MspClient(link, { timeoutMs: probeMs });
  try {
    // One question first, and only then the rest of the identity.
    //
    // `identify()` asks four things in a row, and each one that goes unanswered
    // costs a whole deadline — a silent link would take four times as long to
    // rule out as it needs to, which is the difference between a probe and a
    // hang. One miss costs one deadline, and a board that answered the first
    // question answers the others.
    await client.request(MspCommand.API_VERSION);
    return await client.identify();
  } catch {
    return null;
  } finally {
    client.close();
  }
}

interface ListenResult {
  /** The first byte nobody claimed, or null if the link was silent throughout. */
  readonly firstByte: number | null;
  readonly mavlink: MavDetection | null;
}

/**
 * Waits for the vehicle to say something, and works out whether what it said
 * was MAVLink.
 *
 * The decode here is a second, independent reader: the frames it accepts are
 * checked against MAVLink's own CRC, which needs a per-message constant this app
 * carries rather than one that arrives on the wire. A stray `0xfd` in a stream
 * of console noise therefore does not become a "PX4" — the enormous majority of
 * arbitrary bytes fail that check, and only a whole, correctly-checksummed frame
 * counts as having been heard.
 *
 * It stops early on a heartbeat, which is the vehicle announcing itself and the
 * only frame that carries the system id a reply would have to be addressed to.
 */
function listen(link: ByteLink, listenMs: number): Promise<ListenResult> {
  return new Promise((resolve) => {
    const decoder = new MavlinkDecoder();
    const heard: Uint8Array[] = [];
    let firstByte: number | null = null;
    let frames = 0;
    let heartbeat: MavHeartbeat | null = null;
    let settled = false;

    const finish = (): void => {
      if (settled) return;
      settled = true;
      off();
      clearTimeout(timer);
      resolve({
        firstByte,
        mavlink:
          frames === 0
            ? null
            : { heartbeat, frames, heard: heard.map((chunk) => chunk.slice()) },
      });
    };

    const off = link.onData((chunk) => {
      if (chunk.length === 0) return;
      if (firstByte === null) firstByte = chunk[0]!;
      heard.push(chunk);
      // Bytes are kept as they arrived and handed on whole. A window that
      // reassembled them into frames first would be handing the board a
      // reconstruction to trust, rather than the wire.
      const result = decoder.push(chunk);
      frames += result.frames.length;
      for (const frame of result.frames) {
        if (frame.msgid !== 0 || heartbeat !== null) continue;
        try {
          heartbeat = heartbeatOf(frame);
        } catch {
          // A heartbeat this app cannot read is not one. The frame still
          // counted, so the link is still MAVLink.
        }
      }
      if (heartbeat !== null) finish();
    });
    const timer = setTimeout(finish, listenMs);
  });
}


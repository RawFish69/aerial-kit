import type { ByteLink } from '../protocol/client';

/**
 * How bytes got here, which is a thing the interface has to keep saying. A
 * person looking at a screen full of plausible numbers is entitled to know
 * whether they came from a board on a cable, the firmware's own simulator, or
 * nothing at all.
 */
export type TransportKind = 'serial' | 'bridge' | 'demo';

/**
 * What the simulated board calls itself, on the wire.
 *
 * This is a constant here rather than a literal in the demo board because three
 * places need the same string and they must not be able to disagree: the board
 * that answers it, the transport that builds that board, and the session that
 * decides whether the identity it just read came from hardware.
 *
 * The session reads the *product string*, not `TransportInfo.kind`, and that
 * ordering is deliberate. The demo's one job is to never pass for hardware, and
 * a check against the transport would be satisfied by a board that was
 * simulated while claiming to be an F405 — which is precisely the mistake worth
 * catching. What the board says about itself is the thing that has to carry the
 * word "demo", so the identity is taken from the bytes.
 */
export const DEMO_PRODUCT = 'aerialkit-demo';

export interface TransportInfo {
  readonly kind: TransportKind;
  /** Short, for the identity strip: "USB serial", "local bridge", "demo". */
  readonly label: string;
  /** The detail worth showing on hover or in the diagnostics pane. */
  readonly detail: string;
}

export interface Transport extends ByteLink {
  readonly info: TransportInfo;
  /** Opens the link. For serial this must be reached from a user gesture —
   *  browsers require it — so callers open from a click handler, not on mount. */
  open(): Promise<void>;
  close(): Promise<void>;
}

/** Web Serial is the only transport that reaches a board with no companion
 *  installed, and it exists on a minority of browsers. Everything that offers
 *  it has to be able to say no. */
export function serialSupported(): boolean {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

export class TransportError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'TransportError';
  }
}

import { useCallback, useEffect, useSyncExternalStore } from 'react';
import type { Session, SessionSnapshot } from '../session/types';

/**
 * The session, as React state.
 *
 * `useSyncExternalStore` rather than `useState` plus an effect, because the
 * session already is an external store with a stable snapshot object: it
 * replaces `snapshot` on publish and leaves it alone in between. Subscribing to
 * it directly means a render is caused by a frame arriving and by nothing else,
 * which for a page whose job is to show what the board last said is the whole
 * correctness argument.
 */
export function useSession(session: Session | null): SessionSnapshot | null {
  const subscribe = useCallback(
    (listener: () => void) => (session === null ? () => {} : session.subscribe(listener)),
    [session],
  );
  const getSnapshot = useCallback(
    () => (session === null ? null : session.snapshot),
    [session],
  );
  return useSyncExternalStore(subscribe, getSnapshot, getSnapshot);
}

/**
 * Drives the staleness clock.
 *
 * Nothing on the wire says "the armed state is now old" — that is a fact about
 * time, so something has to notice it. This is that something, and it is
 * deliberately the view: a session left open in a background tab stops being
 * aged, and a background tab is not where anyone is about to press a button
 * that matters.
 */
export function useAging(session: Session | null, everyMs = 250): void {
  useEffect(() => {
    if (session === null) return;
    const timer = setInterval(() => session.age(), everyMs);
    return () => clearInterval(timer);
  }, [session, everyMs]);
}

/**
 * The same clock, for a vehicle that is not ours.
 *
 * A MAVLink board's armed state comes from its heartbeat and expires the same
 * way ours does, so it needs the same nudge — otherwise a page left open after
 * a vehicle is switched off would keep showing "Armed" or "Disarmed" as though
 * it were current.
 */
export function useMavAging(board: { age(): void } | null, everyMs = 250): void {
  useEffect(() => {
    if (board === null) return;
    const timer = setInterval(() => board.age(), everyMs);
    return () => clearInterval(timer);
  }, [board, everyMs]);
}

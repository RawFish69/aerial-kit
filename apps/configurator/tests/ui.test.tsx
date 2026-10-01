import { render, screen, waitFor, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it, vi } from 'vitest';
import { App } from '../src/ui/App';
import { ArmedBanner, Limitations, ParametersPanel } from '../src/ui/panels';
import type {
  BoardMeta,
  Limitation,
  LiveView,
  ParameterRow,
  Permission,
  SessionSnapshot,
} from '../src/session/types';
import { ALL_LIMITATIONS } from '../src/session/limitations';
import { DiagnosticsPanel } from '../src/ui/panels';
import { TabRail } from '../src/ui/TabRail';
import { MspBoard } from '../src/session/foreign';
import { ForeignWorkspace } from '../src/ui/foreign';
import { MavWorkspace } from '../src/ui/mavlink';
import { MavBoard } from '../src/session/mavlink';
import { demoParameters } from '../src/transport/demo';
import { FixtureTransport } from './msp-link';
import { MavFixtureTransport, MavFixtureVehicle } from './mavlink-link';

/**
 * The interface's contract with the person using it.
 *
 * These are not tests that the components render. They are tests that the
 * interface cannot say a reassuring thing it does not know — which is the only
 * failure mode of this app that matters, because every other one is visible on
 * the next screen and this one is not.
 */

const ALLOWED: Permission = { allowed: true, reason: 'the board reports disarmed, read 40 ms ago' };
const ARMED: Permission = {
  allowed: false,
  reason: 'the board says it is armed. This app does not write to an armed aircraft; disarm it and the write will be enabled.',
};
const STALE: Permission = {
  allowed: false,
  reason:
    'the armed state is 3200 ms old and this app stops trusting it at 2000 ms — the aircraft may have armed since.',
};

function live(over: Partial<LiveView> = {}): LiveView {
  return { status: null, telemetry: null, armed: 'disarmed', lastSeenMs: 40, stale: false, ...over };
}

/** A row whose description looks like one a board served over `param info`.
 *
 *  That is now the only way a row gets a description at all: there is no file
 *  fallback, so every field here is one the board would have had to send. The
 *  bounds are *text*, as they are on the wire — the protocol has no typed fields
 *  and the app parses them, so a test that handed over numbers would be testing
 *  a shape that cannot arrive. */
function meta(over: Partial<BoardMeta> = {}): BoardMeta {
  return {
    name: 'roll_kp',
    type: 0,
    typeName: 'float',
    group: 1,
    groupName: 'rates',
    decimals: 3,
    flags: 0,
    secret: false,
    min: '0.000',
    max: '3.000',
    maxLen: null,
    default: '0.250',
    help: 'rate loop P, roll',
    ...over,
  };
}

function row(over: Partial<ParameterRow> = {}): ParameterRow {
  return {
    index: 0,
    name: 'roll_kp',
    boardValue: '0.250',
    edited: null,
    meta: meta(),
    metaUnavailable: null,
    write: null,
    changedFromBoot: false,
    ...over,
  };
}

function actions() {
  return {
    edit: vi.fn(),
    revert: vi.fn(),
    write: vi.fn(),
    reread: vi.fn(),
    writeAll: vi.fn(),
    reset: vi.fn(),
  };
}

describe('the armed banner', () => {
  it('says the word, not only the colour', () => {
    // Colour never travels alone. Someone reading this on a washed-out bench
    // monitor, or who cannot distinguish the two hues, must still get it.
    const { container } = render(<ArmedBanner live={live()} permission={ALLOWED} />);
    expect(screen.getByText('Disarmed')).toBeInTheDocument();
    expect(container.querySelector('.armed')).toHaveClass('is-disarmed');
  });

  it('marks an armed aircraft unmistakably', () => {
    const { container } = render(
      <ArmedBanner live={live({ armed: 'armed' })} permission={ARMED} />,
    );
    expect(screen.getByText('Armed')).toBeInTheDocument();
    expect(screen.getByText('no write will be sent')).toBeInTheDocument();
    expect(container.querySelector('.armed')).toHaveClass('is-armed');
  });

  it('makes unknown look unknown, and never like disarmed', () => {
    // The one failure this app exists to avoid. A stale armed state is not a
    // disarmed aircraft: the board may have armed since the last frame, and a
    // grey that reads as "probably fine" is how someone writes to a live model.
    const { container } = render(
      <ArmedBanner
        live={live({ armed: 'unknown', stale: true, lastSeenMs: 3200 })}
        permission={STALE}
      />,
    );
    expect(screen.getByText('Armed state unknown')).toBeInTheDocument();
    expect(screen.queryByText('Disarmed')).not.toBeInTheDocument();
    expect(container.querySelector('.armed')).toHaveClass('is-unknown');
    expect(container.querySelector('.armed')).not.toHaveClass('is-disarmed');
    // And the reason states the bound and the arithmetic, not a vibe.
    expect(screen.getByText(/2000 ms/)).toBeInTheDocument();
  });

  it('gives the reason a write was allowed, not only the reason it was not', () => {
    render(<ArmedBanner live={live()} permission={ALLOWED} />);
    expect(screen.getByText(/read 40 ms ago/)).toBeInTheDocument();
  });
});

describe('the parameter table', () => {
  it('disables every write when the gate is shut, and says why on the control', async () => {
    const acts = actions();
    render(
      <ParametersPanel
        rows={[row({ edited: '1.000' })]}
        permission={ARMED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={acts}
      />,
    );
    // Send is refused for a staged row while armed, and the refusal is on the
    // button itself so it is found by hovering as well as by reading around.
    expect(screen.getByRole('button', { name: 'Send' })).toBeDisabled();
    expect(screen.getByRole('button', { name: 'Send' })).toHaveAttribute('title', ARMED.reason);
    expect(screen.getByRole('button', { name: /Send all staged/ })).toBeDisabled();

    // Clicking it anyway must not reach the session.
    await userEvent.click(screen.getByRole('button', { name: 'Send' }));
    expect(acts.write).not.toHaveBeenCalled();
  });

  it('allows a write when the gate is open', async () => {
    const acts = actions();
    render(
      <ParametersPanel
        rows={[row({ edited: '1.000' })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={acts}
      />,
    );
    await userEvent.click(screen.getByRole('button', { name: 'Send' }));
    expect(acts.write).toHaveBeenCalledWith(0);
  });

  it('keeps a staged row to two verbs so the write column cannot overflow', () => {
    // jsdom performs no layout, so "this header text is wider than its column"
    // is untestable here — clientWidth and scrollWidth are both zero. The
    // column widths are verified against the rendered page in the screenshot
    // pass instead. What *can* be pinned is the behaviour that keeps the
    // widest cell state small: a staged row offers exactly Discard and Send,
    // never Read beside them, because three verbs side by side measured 175px
    // and overflowed the column. Read returns once the value is sent or
    // discarded.
    const acts = actions();
    render(
      <ParametersPanel
        rows={[row({ edited: '1.000' })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={acts}
      />,
    );
    const cell = screen.getByRole('button', { name: 'Send' }).parentElement!;
    expect(within(cell).getAllByRole('button').map((button) => button.textContent)).toEqual([
      'Discard',
      'Send',
    ]);
    expect(within(cell).queryByRole('button', { name: 'Read' })).toBeNull();
  });

  it('offers Read and Reset on a clean row, and stops at two', () => {
    // This assertion read `['Read']` alone until milestone 4, and it was right
    // then: the reset had no wire call behind it. It is the same rule as the
    // staged row's — the write column holds at two verbs — so it stays a list
    // equality rather than a "contains", and a third control added here will
    // fail it exactly as it was meant to.
    const acts = actions();
    render(
      <ParametersPanel
        rows={[row()]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={acts}
      />,
    );
    const cell = screen.getByRole('button', { name: 'Read' }).parentElement!;
    expect(within(cell).getAllByRole('button').map((button) => button.textContent)).toEqual([
      'Read',
      'Reset',
    ]);
  });

  it('disables Reset, with the reason on it, on a board that predates param default', () => {
    // The reset is a capability, not a control the app assumes. A board whose
    // HELLO carried no PARAM_DEFAULT bit falls through to the firmware's
    // unknown-command case, so the button must be present-but-refused with the
    // sentence that says why — the alternative, hiding it, leaves a person
    // wondering whether their firmware is old or their click missed.
    const acts = actions();
    const reason = 'this board does not answer `param default`';
    render(
      <ParametersPanel
        rows={[row()]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={reason}
        actions={acts}
      />,
    );
    const button = screen.getByRole('button', { name: 'Reset' });
    expect(button).toBeDisabled();
    expect(button).toHaveAttribute('title', reason);
    const all = screen.getByRole('button', { name: /Reset all to build defaults/ });
    expect(all).toBeDisabled();
    expect(all).toHaveAttribute('title', reason);
  });

  it('sends the row index for one row, and null for the whole table', () => {
    // The two are different requests on the wire — mode 1 with an index, mode 2
    // with none — and the toolbar button must not send row 0 by accident.
    const acts = actions();
    render(
      <ParametersPanel
        rows={[row(), row({ index: 1 })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={acts}
      />,
    );
    return userEvent
      .click(screen.getAllByRole('button', { name: 'Reset' })[1]!)
      .then(() => {
        expect(acts.reset).toHaveBeenCalledWith(1);
        return userEvent.click(screen.getByRole('button', { name: /Reset all to build defaults/ }));
      })
      .then(() => {
        expect(acts.reset).toHaveBeenLastCalledWith(null);
      });
  });

  it('separates requested from echoed from applied, in words', () => {
    render(
      <ParametersPanel
        rows={[
          row({
            write: {
              requested: '1.250',
              echoed: true,
              // The common case: the board took the value, and whether its
              // configuration was rebuilt for it is not something the reply
              // carries.
              applied: null,
              status: 0,
              message: '',
              atMs: 0,
              notAppliedBecause:
                'the board put it in its parameter table, which is all this reply can confirm.',
            },
          }),
        ]}
        permission={ALLOWED}
        unsaved={1}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText('requested')).toBeInTheDocument();
    expect(screen.getByText('1.250')).toBeInTheDocument();
    expect(screen.getByText(/the board put it in its table/)).toBeInTheDocument();
    // The sentence that stops "ok" from being read as "done" — and that does
    // not go the other way either. `null` is not "no".
    expect(screen.getByText('applied')).toBeInTheDocument();
    expect(screen.getByText('not established')).toBeInTheDocument();
    expect(screen.queryByText(/^no —/)).not.toBeInTheDocument();
  });

  it('says "no" for applied only when the write did not land', () => {
    render(
      <ParametersPanel
        rows={[
          row({
            write: {
              requested: '99',
              echoed: false,
              // Knowable, unlike the accepted case: the table did not take the
              // value, so nothing rebuilt from it.
              applied: false,
              status: 4,
              message: 'airframe must be 0..6',
              atMs: 0,
              notAppliedBecause: 'the board refused it',
            },
          }),
        ]}
        permission={ALLOWED}
        unsaved={1}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText('no — the board refused it')).toBeInTheDocument();
    expect(screen.queryByText('not established')).not.toBeInTheDocument();
  });

  it('shows a range when both bounds arrived, and only then', () => {
    render(
      <ParametersPanel
        rows={[row({ meta: meta({ decimals: 3, min: '0.000', max: '3.000' }) })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText('0.000 … 3.000')).toBeInTheDocument();
    expect(screen.queryByText(/no range stated/)).not.toBeInTheDocument();
  });

  it('shows no range rather than a guessed one when a bound will not evaluate', () => {
    // The board spelled both bounds — this is not a missing-metadata case — and
    // one of them is text this parser will not turn into a number. The row says
    // so and quotes what the board actually sent, because "no range" alone reads
    // as "any value".
    render(
      <ParametersPanel
        rows={[row({ meta: meta({ type: 1, typeName: 'u32', decimals: 0, min: '0', max: 'auto' }) })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText(/no range stated/)).toBeInTheDocument();
    expect(screen.getByText(/will not evaluate/)).toBeInTheDocument();
    expect(screen.getByText(/max “auto”/)).toBeInTheDocument();
  });

  it('says a text parameter is bounded by length, not by a range', () => {
    // `maxLen` is set, so the row has a bound the board did state — it is just
    // not a numeric one. Rendering it as "no range" would hide a real limit.
    render(
      <ParametersPanel
        rows={[row({ meta: meta({ type: 2, typeName: 'text', min: '', max: '', maxLen: 16 }) })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText(/bounded by 16 characters/)).toBeInTheDocument();
  });

  it('shows that a board it has no metadata for gets no invented help', () => {
    render(
      <ParametersPanel
        rows={[
          row({
            meta: null,
            metaUnavailable: 'this board answered `param info` with 0x7f, so it predates it',
          }),
        ]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText('no description')).toBeInTheDocument();
    expect(screen.getByText(/not described by the board/i)).toBeInTheDocument();
    // And the reason is the board's own, not this app's guess at one.
    expect(screen.getByText(/predates it/)).toBeInTheDocument();
  });

  it('distinguishes "the board did not describe it" from "the help has not arrived"', () => {
    // Both render an empty help cell, and they are different facts: one is a
    // board that never sent a description, the other is a row whose sentence is
    // still on its way. Collapsing them would call a slow walk a missing one.
    const { unmount } = render(
      <ParametersPanel
        rows={[row({ meta: null })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText('no description')).toBeInTheDocument();
    expect(screen.queryByText('help not read')).not.toBeInTheDocument();
    unmount();

    render(
      <ParametersPanel
        rows={[row({ meta: meta({ help: null }) })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    // Described, so the row is not "no description" — only its prose is pending.
    expect(screen.getByText('help not read')).toBeInTheDocument();
    expect(screen.queryByText('no description')).not.toBeInTheDocument();
  });

  it('shows a described row with no bounds as described, not as missing metadata', () => {
    // The board answered for this row and stated no numeric bound. That is not
    // the same as a board that never answered, and the heading says which.
    render(
      <ParametersPanel
        rows={[row({ meta: meta({ min: '', max: '' }) })]}
        permission={ALLOWED}
        unsaved={0}
        busy={false}
        resetReason={null}
        actions={actions()}
      />,
    );
    expect(screen.getByText('no range stated')).toBeInTheDocument();
    expect(screen.getByText(/stated no numeric bound for it/)).toBeInTheDocument();
    expect(screen.queryByText('no description')).not.toBeInTheDocument();
    expect(screen.queryByText(/not described by the board/i)).not.toBeInTheDocument();
  });
});

describe('the limitations list', () => {
  it('shows the normal write-safety rule as a note when the write gate is open', () => {
    const items = Object.values(ALL_LIMITATIONS).filter((item) => item.id === 'armed-state-gates-writes');
    const { container, rerender } = render(<Limitations items={items} permission={ALLOWED} />);
    expect(container.querySelector('.bp5-intent-danger')).toBeNull();
    expect(screen.getByText(/writes need a disarmed aircraft/)).toBeInTheDocument();
    rerender(<Limitations items={items} permission={ARMED} />);
    expect(container.querySelector('.bp5-intent-danger')).toBeInTheDocument();
  });

  it('states every claim with the file it came from', () => {
    const items = Object.values(ALL_LIMITATIONS) as readonly Limitation[];
    render(<Limitations items={items} />);
    for (const item of items) {
      // `getAllBy`, not `getBy`: two entries share an id and a summary and
      // differ only in the detail — the "it says no" and the "it cannot say"
      // wordings of the same limitation. Rendering both at once is not a bug,
      // it is this test exercising every wording there is.
      expect(screen.getAllByText(item.summary).length, item.detail).toBeGreaterThan(0);
      expect(screen.getAllByText(item.detail).length, item.detail).toBe(1);
    }
    // Every cited file is a real source path, not a gesture at one.
    const text = document.body.textContent ?? '';
    expect(text).toMatch(/ak_proto\.c/);
  });

  it('says so plainly when there is nothing to report', () => {
    render(<Limitations items={[]} />);
    expect(screen.getByText(/No limitations recorded/)).toBeInTheDocument();
  });
});

describe('diagnostics', () => {
  it('counts refused frames separately from received ones', () => {
    const snapshot = { counts: { frames: 41, writes: 3, telemetry: 12, issues: 2 } } as SessionSnapshot;
    render(<DiagnosticsPanel snapshot={snapshot} events={[]} />);
    expect(screen.getByText('41')).toBeInTheDocument();
    expect(screen.getByText('pushed')).toBeInTheDocument();
    expect(screen.getByText('2')).toBeInTheDocument();
  });
});

describe('the rail', () => {
  it('keeps an unbackable tab in the list, disabled, saying which half is missing', () => {
    // A board that answers `hello` without a capability word: every tab that
    // needs one must say *that*, not "unavailable" and not nothing at all.
    const snapshot = {
      phase: 'ready',
      identity: { product: 'aerialkit-f405', protocolVersion: 1, parameterCount: 92,
        changedSinceSaved: '0', features: null, configHash: null, isDemo: false },
      parameters: [], unsaved: 0,
      permission: ALLOWED, limitations: [], events: [], readError: null, readProgress: null,
      openedAtMs: null, telemetry: { agreedHz: null, received: 0, lastFrameMs: null },
      live: live(),
      rc: { state: null, error: null, atMs: null, watching: false, polls: 0, failed: 0 },
      sensors: { answers: {}, error: null, atMs: null, watching: false, rounds: 0, failed: 0 },
      transport: { label: 'demo', detail: 'in this page' },
      counts: { frames: 0, telemetry: 0, issues: 0, writes: 0 },
    } as unknown as SessionSnapshot;

    render(
      <TabRail
        snapshot={snapshot}
        actions={actions()}
        busy={false}
        streamHz={10}
        setStreamHz={() => {}}
        onStream={() => {}}
        nowMs={0}
        onRefresh={() => {}}
        onSave={() => {}}
        onWatchRc={() => {}}
        onReadRc={() => {}}
        onWatchSensors={() => {}}
        onReadSensors={() => {}}
        onWatchAttitude={() => {}}
      />,
    );

    // The written tabs are openable; the ones with no view are present and
    // disabled. Preflight is one that needs a capability word and has no view.
    // (Motors used to be the example; it now draws the outputs from STATUS.)
    expect(screen.getByRole('button', { name: 'Live' })).toBeEnabled();
    expect(screen.getByRole('button', { name: 'Attitude' })).toBeEnabled();
    expect(screen.getByRole('button', { name: 'Motors' })).toBeEnabled();
    const preflight = screen.getByRole('button', { name: /^Preflight/ });
    expect(preflight).toBeDisabled();
    // The reason names the *opcode* and the reason it is missing, because the
    // useful thing is to know what to look for. It is in the title, and the
    // body says it too for anyone not hovering.
    expect(preflight.getAttribute('title')).toMatch(/preflight/);
    expect(preflight.getAttribute('title')).toMatch(/capability word/);

    // Receiver and Sensors are the exceptions, and deliberately so: both are
    // written, and both open for a board with no capability word at all. The
    // board is still asked — a firmware that predates the word answers `rc
    // channels` and `sensor info` with 0x7F — and each panel says so in the
    // board's own terms rather than this rail guessing on its behalf. A tab
    // that stayed disabled here would hide the answer the board is perfectly
    // able to give.
    expect(screen.getByRole('button', { name: 'Receiver' })).toBeEnabled();
    expect(screen.getByRole('button', { name: 'Sensors' })).toBeEnabled();
  });
});

describe('the whole page, against the demo board', () => {
  it('connects, reads the table, and shows a real parameter with the demo badge', async () => {
    const user = userEvent.setup();
    render(<App />);

    // Nothing is open until someone clicks. This app never claims a device
    // because a page loaded.
    expect(screen.getByRole('button', { name: 'Explore demo' })).toBeInTheDocument();

    await user.click(screen.getByRole('button', { name: 'Explore demo' }));

    // The demo board answers to a name the capture did not record, so the page
    // gets the badge without any view having to decide it is a demo.
    expect(await screen.findByText('simulated — not hardware', {}, { timeout: 15_000 })).toBeInTheDocument();
    expect(screen.getByText('aerialkit-demo')).toBeInTheDocument();

    // A real parameter, from the firmware's own table, read over the protocol.
    // Behind the Parameters tab now — the rail opened on Live, which is the
    // first question a person has, and the table is the second.
    await user.click(screen.getByRole('button', { name: /^Parameters/ }));
    expect(await screen.findByText('rate_kp_roll', {}, { timeout: 15_000 })).toBeInTheDocument();

    // And the page states what it cannot establish, unprompted — the limitation
    // strip is on screen for every connection, not tucked behind a disclosure.
    //
    // These two sentences used to be the F4 finding: "a write reaches the
    // parameter table, not the running aircraft" and "ak_proto.c:211-247 has no
    // equivalent". Both were false — the protocol does fire the post-change
    // callback — and this test was pinning them to the screen.
    //
    // Now the demo board reports APPLIES_ON_WRITE, so the first of those
    // sentences has no business on the screen at all: the board has answered
    // the question and the answer is yes. Pinning its *absence* is the stronger
    // assertion, and it is the one this milestone is for.
    expect(screen.getByText('aerialkit-demo')).toBeInTheDocument();
    expect(
      screen.queryByText(
        'the board confirms the value is in its table, not that the aircraft rebuilt for it',
      ),
    ).toBeNull();
    // And the second of the pair, which milestone 4 retired. This used to be a
    // `getByText` pinning the armed-set note to the screen, and it was right to
    // while the firmware left GATES_ON_ARMED clear — a set really was unchecked
    // against the flight state. The firmware consults `ak_proto_io_t.writable`
    // on every write route now, so the demo board reports the bit and the app
    // has nothing left to warn about. Flipping this assertion to its absence is
    // the assertion; a test that simply deleted it would have thrown away the
    // record that the note was once owed.
    expect(
      screen.queryByText(
        /the board would take a parameter value while armed; this app will not send one/,
      ),
    ).toBeNull();
  }, 30_000);

  it('re-reads the table without growing it, on a board whose groups repeat', async () => {
    // The board's own `group` bytes are not monotonic. This build's run
    // 1, 1, … 1, 3, 1, 2, 2, … 4, 12, 8, 8, 8, 5, 4, 4, 4, 4, so three of the
    // seven groups — 1, 3 and 4 — each begin a second chapter somewhere further
    // down the table. `groupRows` used to key a chapter by its group number
    // alone, which made two siblings share a React key. React cannot tell two
    // children with one key apart, and what it does instead is keep the
    // previous render's rows alongside the new ones.
    //
    // Measured with the old key, on this page: the table went 43 tbody rows to
    // 61 to 79 to 97 — 18 rows and 3 chapters more on every re-read, three being
    // exactly the number of groups the table revisits. Production React strips
    // the duplicate-key warning, so nothing said a word about it.
    //
    // Why this test is at the page level and not on `ParametersPanel`: a
    // synthetic three-row table with groups 1, 3, 1 does **not** reproduce it —
    // that test passed against the broken key. The failure needs the whole
    // table's shape, so the fixture is the real one, read through the same
    // transport a person uses.
    const user = userEvent.setup();
    render(<App />);
    await user.click(screen.getByRole('button', { name: 'Explore demo' }));
    await screen.findByText('simulated — not hardware', {}, { timeout: 15_000 });
    await user.click(screen.getByRole('button', { name: /^Parameters/ }));
    await screen.findByText('rate_kp_roll', {}, { timeout: 15_000 });

    // What the table is supposed to draw, from the source of the demo board's
    // own table rather than from a number written down here.
    const expected = demoParameters().length;
    const drawn = () => {
      const tbody = document.querySelector('tbody');
      return {
        rows: tbody?.querySelectorAll('tr:not(.group)').length ?? -1,
        inputs: tbody?.querySelectorAll('input').length ?? -1,
      };
    };

    await waitFor(() => expect(drawn()).toEqual({ rows: expected, inputs: expected }), { timeout: 15_000 });

    // Two re-reads, because the bug compounded: the second is worse than the
    // first, and one would not distinguish "grew" from "drew one extra". The
    // wait is on the count rather than on a name, because under the bug the name
    // is the thing that appears twice and a `findByText` fails on the duplicate
    // before the count is ever reported — which hides the number that says what
    // went wrong.
    for (let round = 0; round < 2; round += 1) {
      await user.click(screen.getByRole('button', { name: 'Re-read table' }));
      await waitFor(() => expect(drawn()).toEqual({ rows: expected, inputs: expected }), { timeout: 15_000 });
    }

    // And the row on screen is the table that was just read, not a leftover.
    // The value shown here is the demo table's, and it is shown once: under the
    // old key the older copy stayed in the DOM next to the new one.
    const shown = [...document.querySelectorAll('tbody tr:not(.group)')].filter((tr) =>
      tr.textContent?.includes('rate_kp_roll'),
    );
    expect(shown).toHaveLength(1);
  }, 60_000);

  it('offers Web Serial as unavailable rather than hiding it, when the browser has none', async () => {
    render(<App />);
    const choices = screen.getByRole('radiogroup', { name: 'Talk to' });
    const serial = within(choices).getByRole('radio', { name: /USB serial/ });
    // jsdom has no navigator.serial, which is every browser without Web Serial.
    expect(serial).toBeDisabled();
    expect(serial.closest('label')?.textContent).toContain('not available in this browser');
  });
});

describe('the read-only page', () => {
  it('shows a foreign board without offering it a single write', async () => {
    const board = new MspBoard(new FixtureTransport());
    await board.open();
    render(<ForeignWorkspace board={board} />);

    // What it is, from the board's own answer.
    expect(screen.getByText('BTFL')).toBeInTheDocument();
    // Twice: once as a fact and once in the trace, which is what a trace is for.
    expect(screen.getAllByText(/AERIALKIT-SIM \(STM32F405\)/).length).toBeGreaterThan(0);

    // The armed state, from the status frame, with the same three states as
    // our own page — a board whose arming state is unknown must look unknown
    // here too.
    expect(screen.getByText('Disarmed')).toBeInTheDocument();

    // The limitations, including the one that matters most.
    expect(screen.getByText(/Read-only\./)).toBeInTheDocument();
    expect(screen.getByText(/no "list the settings" command/)).toBeInTheDocument();
    expect(screen.getByText(/Nothing here has been run against a physical board/)).toBeInTheDocument();

    // And the thing itself: no control anywhere on this page writes, saves or
    // streams. Asserted rather than described, because a button added later by
    // someone in a hurry would otherwise go unnoticed.
    for (const label of [/save/i, /write/i, /flash/i, /stream/i]) {
      expect(screen.queryByRole('button', { name: label })).toBeNull();
    }
    // One button, and it is a read.
    expect(screen.getAllByRole('button').map((button) => button.textContent)).toEqual(['Read']);
  });
});

describe('the page for a vehicle that is not ours', () => {
  async function vehiclePage(
    paint: (vehicle: MavFixtureVehicle) => void,
  ): Promise<{ board: MavBoard; transport: MavFixtureTransport }> {
    const transport = new MavFixtureTransport();
    const vehicle = transport.vehicle;
    const board = new MavBoard(transport, { linkAlreadyOpen: true });
    vehicle.sayArmed(true);
    paint(vehicle);
    await board.open();
    await Promise.resolve();
    return { board, transport };
  }

  it('shows what the vehicle said, and offers it no way to be flown', async () => {
    const { board } = await vehiclePage((vehicle) => {
      vehicle.say('ATTITUDE');
      vehicle.say('GLOBAL_POSITION_INT');
      vehicle.say('SYS_STATUS');
      vehicle.parameters = [{ id: 'RATE_RLL_P', value: 0.5, type: 9 }];
    });
    await board.readParameters(2000);
    render(<MavWorkspace board={board} />);

    // What it is, from the vehicle's own heartbeat.
    expect(screen.getAllByText('ArduPilot').length).toBeGreaterThan(0);
    expect(screen.getAllByText('quadrotor').length).toBeGreaterThan(0);
    // Armed, and drawn armed — the same three states as our own page.
    expect(screen.getByText('Armed')).toBeInTheDocument();
    expect(screen.getByText(/^from a heartbeat/)).toBeInTheDocument();

    // The parameter table, read rather than guessed at.
    expect(screen.getByText('RATE_RLL_P')).toBeInTheDocument();

    // The limitations, including the one that matters most on this protocol.
    expect(screen.getByText(/^Read-only\./)).toBeInTheDocument();
    expect(screen.getByText(/MAVLink has PARAM_SET/)).toBeInTheDocument();

    // Nothing on this page can arm, fly, upload or write a parameter. The one
    // write it does offer is a stream rate, and it is labelled as a write.
    for (const label of [/arm/i, /^disarm/i, /takeoff/i, /mission/i, /^set /i, /parameter$/i, /mode/i]) {
      expect(screen.queryByRole('button', { name: label })).toBeNull();
    }
    expect(screen.getByText(/This is a write, and it is the only one on this page/)).toBeInTheDocument();
  });

  it('says a heading the vehicle does not have, instead of drawing north', async () => {
    const { board } = await vehiclePage((vehicle) => {
      // `0xffff` in GLOBAL_POSITION_INT's `hdg` field: no heading known.
      vehicle.say('GLOBAL_POSITION_INT', { hdg: 0xffff });
    });
    render(<MavWorkspace board={board} />);

    expect(screen.getByText('the vehicle did not say')).toBeInTheDocument();
    // 0° would be a confident lie about which way the aircraft is pointing.
    expect(screen.queryByText('0°')).toBeNull();
  });

  it('counts a message it cannot read rather than showing a blank row', async () => {
    const { board } = await vehiclePage((vehicle) => {
      vehicle.say('COMMAND_ACK');
      const mangled = Uint8Array.from(vehicle.lastSaid!);
      mangled[7] = 248;
      vehicle.sayBytes(mangled);
    });
    render(<MavWorkspace board={board} />);

    expect(screen.getByText('message 248')).toBeInTheDocument();
    expect(screen.getByText(/no definition in this app/)).toBeInTheDocument();
  });
});

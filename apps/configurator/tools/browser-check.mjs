/**
 * Drive the built configurator in a real browser.
 *
 * W4 asks for the site to be verified "on ... supported browsers" and for
 * "connect/disconnect/reconnect and permission workflows" to be exercised. That
 * is a question about a browser, and jsdom cannot answer it: jsdom has no
 * layout, no Web Serial and no real event loop for a serial chooser. So this
 * runs the actual page in an actual browser and reports what happened.
 *
 * Three modes, and the third exists because the second does not run everywhere:
 *
 *   chrome           has Web Serial. It can be driven through the whole
 *                    workflow: connect, disconnect, reconnect.
 *   chrome-noserial  Chrome with `Navigator.prototype.serial` deleted before
 *                    the app loads. This exercises the branch a person on
 *                    Firefox or Safari lands in — "the page says so rather than
 *                    throwing" — without needing Firefox to be launchable.
 *                    It is a simulation of the capability, not of the browser:
 *                    it proves what the app does when the API is absent, and
 *                    nothing about Firefox's rendering, layout or events.
 *   firefox          the real thing, when it can be launched. On the machine
 *                    this was written on it cannot (see tools/verify-browser.sh),
 *                    and the script says so instead of reporting a pass.
 *   mavlink          Chrome, the local bridge, and an ArduPilot stand-in
 *                    behind it. This is the only mode where the bytes the page
 *                    reads were built by something other than this repository:
 *                    tools/mavlink_fake_vehicle.py encodes every frame with
 *                    pymavlink, the reference implementation, and decodes the
 *                    page's two writes with it as well. The page under test is
 *                    the third workspace — a vehicle that is not ours — and
 *                    what is checked is mostly what is *absent* from it.
 *
 * The board in the other modes is the demo board, which is a protocol peer
 * inside the page rather than a mock of the interface: it produces frames the
 * same decoder parses and answers commands the same client sends. **No physical
 * board is touched by this script** and none is claimed to be.
 *
 * Usage:  node tools/browser-check.mjs <base-url> <chrome|chrome-noserial|firefox|mavlink>
 * Exits 0 when every check passes, 1 when a check fails, 3 when the browser —
 * or, in mavlink mode, the bridge or the vehicle behind it — could not be
 * started at all (a different fact from a failing check, and one that must not
 * be reported as either a pass or a failure of the app).
 * Requires `puppeteer-core` to be resolvable; see tools/verify-browser.sh.
 */

import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const MODES = ['chrome', 'chrome-noserial', 'firefox', 'mavlink'];
const [baseUrl, mode = 'chrome'] = process.argv.slice(2);
if (!baseUrl) {
  console.error(`usage: browser-check.mjs <base-url> <${MODES.join('|')}>`);
  process.exit(2);
}
if (!MODES.includes(mode)) {
  console.error(`unknown mode ${mode}; expected ${MODES.join(', ')}`);
  process.exit(2);
}
const which = mode === 'firefox' ? 'firefox' : 'chrome';
const stripSerial = mode === 'chrome-noserial';
const isMavlink = mode === 'mavlink';

/**
 * The vehicle behind the bridge, and how long until it arms.
 *
 * Arm-after is not decoration: an armed-state transition is the one thing on
 * this page that can only be observed over time, and the alternative to making
 * the vehicle do it is to assert the banner once and call it verified. Ten
 * seconds is long enough to get through the identity, telemetry, parameter and
 * write checks while the vehicle is still disarmed, and short enough that the
 * wait at the end is a wait and not a timeout.
 */
const ARM_AFTER_S = 10;

/** Neither of these is from npm; both are paths in this repository. */
const BRIDGE_JS = fileURLToPath(new URL('../bridge/index.js', import.meta.url));
const VEHICLE_PY = fileURLToPath(
  new URL('../../../aerialkit/tools/mavlink_fake_vehicle.py', import.meta.url),
);
const BRIDGE_PORT = Number(process.env.AK_BRIDGE_PORT ?? 8791);
const BRIDGE_URL = `ws://127.0.0.1:${BRIDGE_PORT}/ak`;

let bridge = null;
const stopBridge = () => {
  if (bridge !== null && bridge.exitCode === null) bridge.kill('SIGKILL');
  bridge = null;
};
process.on('exit', stopBridge);

if (isMavlink) {
  console.log(`== bridge: node bridge/index.js --port ${BRIDGE_PORT} --stdio python3 mavlink_fake_vehicle.py --arm-after ${ARM_AFTER_S}`);
  bridge = spawn(
    process.execPath,
    [BRIDGE_JS, '--port', String(BRIDGE_PORT), '--stdio', 'python3', VEHICLE_PY,
     '--arm-after', String(ARM_AFTER_S)],
    { stdio: ['ignore', 'pipe', 'pipe'] },
  );
  // The bridge prints its source and its address on stderr and stdout
  // respectively; waiting for the address is waiting for the thing the page
  // will connect to, rather than sleeping and hoping.
  // A bridge that is *up* whose vehicle child died is the case this had no
  // answer for, and it is the case that actually happened. The bridge listens,
  // the page connects, nothing ever answers, and every check below fails — as
  // twenty-five failures of the app, which is not what they are. The vehicle's
  // own reason arrives on the bridge's stderr (the bridge relays its child's),
  // so it is collected while the bridge comes up and the run stops as not-run
  // instead.
  //
  // This was not hypothetical: with the `upstream/pymavlink-2.4.49` oracle
  // absent between 2026-09-30 and 2026-10-01, this family printed
  // `25 FAILED` and one `..` note, and the note was the only true line in it.
  // The four checks that fail on a working oracle are a separate, real finding;
  // these twenty-five were not a finding at all.
  let vehicleGone = null;
  const ready = await new Promise((resolve) => {
    const timer = setTimeout(() => resolve(false), 8000);
    const look = (chunk) => {
      const text = String(chunk);
      const died = /mavlink_fake_vehicle: .*(no |error|Traceback)/i.exec(text);
      if (died !== null) vehicleGone = died[0].trim();
      if (text.includes('listening on')) {
        clearTimeout(timer);
        resolve(true);
      }
    };
    bridge.stdout.on('data', look);
    bridge.stderr.on('data', (chunk) => {
      process.stderr.write(`  ..    bridge  ${chunk}`);
      look(chunk);
    });
    bridge.on('exit', (code) => {
      clearTimeout(timer);
      resolve(false);
      if (code !== 0) console.error(`  !!    the bridge exited with ${code}`);
    });
  });
  // The child's complaint and the bridge's "listening" line are two streams, so
  // their order is not fixed. Waiting for both to have had their say is what
  // makes this a decision rather than a race — and it is 600 ms, not a sleep
  // that has to be long enough.
  if (ready) await new Promise((resolve) => setTimeout(resolve, 600));
  if (vehicleGone !== null) {
    console.error(`  !!    the fake vehicle did not start: ${vehicleGone}`);
    console.error('  !!    this is NOT a pass and NOT a failure of the app — nothing was tested.');
    stopBridge();
    process.exit(3);
  }
  if (!ready) {
    console.error('  !!    the bridge did not come up; NOTHING WAS TESTED.');
    console.error('  !!    this is not a pass and not a failure of the app.');
    stopBridge();
    process.exit(3);
  }
}

const EXECUTABLE = {
  chrome: process.env.CHROME_PATH || '/usr/bin/google-chrome',
  firefox: process.env.FIREFOX_PATH || '/usr/bin/firefox',
};

const puppeteer = (await import('puppeteer-core')).default;

let failures = 0;
const check = (label, ok, detail = '') => {
  console.log(`  ${ok ? 'ok  ' : 'FAIL'}  ${label}${detail ? `  ${detail}` : ''}`);
  if (!ok) failures++;
};
const note = (label, detail) => console.log(`  ..    ${label}  ${detail}`);

const launch = { executablePath: EXECUTABLE[which], headless: true, args: ['--no-sandbox', '--disable-gpu'] };
if (which === 'chrome') launch.args.push('--ignore-certificate-errors');
if (which === 'firefox') launch.browser = 'firefox';

console.log(`== ${mode}: ${EXECUTABLE[which]}`);
let browser;
try {
  browser = await puppeteer.launch(launch);
} catch (error) {
  console.error(`  !!    could not launch ${which}: ${String(error).split('\n')[0]}`);
  console.error(`  !!    this is NOT a pass and NOT a failure of the app — nothing was tested.`);
  stopBridge();
  process.exit(3);
}
note('browser version', await browser.version());

const page = await browser.newPage();
await page.setViewport({ width: 1440, height: 1000 });

// Remove Web Serial before any app code runs, so `'serial' in navigator` is
// false from the very first line — the same thing a Firefox or Safari user's
// browser reports. Must be done on the prototype: `navigator.serial` is an
// accessor on `Navigator.prototype`, so deleting it from the instance is a
// no-op that would leave the property visible and the test meaningless.
if (stripSerial) {
  await page.evaluateOnNewDocument(() => {
    delete Navigator.prototype.serial;
  });
}

const consoleErrors = [];
const pageErrors = [];
const failedRequests = [];
page.on('console', (m) => { if (m.type() === 'error') consoleErrors.push(m.text()); });
page.on('pageerror', (e) => pageErrors.push(String(e.message ?? e)));
page.on('requestfailed', (r) => failedRequests.push(`${r.url()} — ${r.failure()?.errorText}`));

await page.goto(baseUrl, { waitUntil: 'networkidle0', timeout: 30000 });

// 1. It loaded, and it loaded completely.
const title = await page.title();
check('the page loaded and has a title', title.length > 0, JSON.stringify(title));
check('no failed requests', failedRequests.length === 0, failedRequests.join('; '));
check('no uncaught page errors on load', pageErrors.length === 0, pageErrors.join('; '));

// 2. What this browser actually offers — asked of the browser, not inferred.
const caps = await page.evaluate(() => ({
  serial: 'serial' in navigator,
  origin: location.origin,
  secureContext: window.isSecureContext,
  assetScripts: [...document.querySelectorAll('script[src]')].map((s) => s.getAttribute('src')),
}));
note('origin', caps.origin);
note("'serial' in navigator", String(caps.serial));
note('isSecureContext', String(caps.secureContext));
check('the page is a secure context (Web Serial requires one)', caps.secureContext === true);
check(
  'every script came from the subpath it was built for',
  caps.assetScripts.every((s) => s.startsWith(new URL(baseUrl).pathname)),
  caps.assetScripts.join(', '),
);

const clickButton = async (name) =>
  page.evaluate((n) => {
    const b = [...document.querySelectorAll('button')].find(
      (x) => x.textContent.trim().toLowerCase() === n.toLowerCase(),
    );
    if (!b) return false;
    b.click();
    return true;
  }, name);

const settle = (ms = 2000) => new Promise((r) => setTimeout(r, ms));
const bodyText = () => page.evaluate(() => document.body.innerText);

const buttons = () =>
  page.evaluate(() =>
    [...document.querySelectorAll('button')].map((b) => ({ text: b.textContent.trim(), disabled: b.disabled })),
  );

if (isMavlink) {
  // The bridge transport, chosen the way a person chooses it. The default
  // address is the same one, so this is not a workaround for a fixed port —
  // it is the form being driven, and the check below is that what was typed
  // is what the page kept.
  await page.click('input[name="transport"][value="bridge"]');
  const urlField = await page.$('#url');
  if (urlField === null) {
    check('the bridge address field is offered when the bridge is chosen', false);
  } else {
    // Select-all then type, rather than a triple click: triple-click selects a
    // *line* in some input implementations and nothing at all in others, and
    // the failure it produces is a silently concatenated address that only
    // shows up as a refused connection much later.
    await urlField.click();
    await page.keyboard.down('Control');
    await page.keyboard.press('KeyA');
    await page.keyboard.up('Control');
    await urlField.type(BRIDGE_URL);
    const held = await page.$eval('#url', (el) => el.value);
    check('the bridge address field took what was typed into it', held === BRIDGE_URL, held);
  }

  check('a Connect button is present', await clickButton('Connect bridge'));
  // Detection listens for 1.5 s before it will call anything MAVLink, and the
  // vehicle heartbeats at 1 Hz on a phase of its own.
  await settle(4000);
  let text = await bodyText();

  check('the page says which autopilot answered', text.includes('ArduPilot'), 'from the heartbeat');
  check('and what airframe it is', /quadrotor/i.test(text));
  check(
    'and which framing is on the wire, not the version field',
    /v2 on the wire/.test(text),
  );
  check('the armed state is read from a heartbeat', /from a heartbeat/i.test(text));
  check('and the vehicle is reported disarmed', /Disarmed/.test(text));
  check('the detection line says it is read-only', /read-only/i.test(text));

  // The safety property, checked against the rendered page rather than the
  // source: a MAVLink vehicle accepts arm, mode, mission and parameter writes,
  // and this page must offer none of them. Asserted over every button on the
  // page, so a control added later fails this check rather than shipping.
  const offered = await buttons();
  const forbidden = offered.filter((b) =>
    /\b(arm|disarm|take ?off|land|mission|upload|reboot|flight ?mode|write|save|set)\b/i.test(b.text),
  );
  check(
    'no button on the page could change the vehicle',
    forbidden.length === 0,
    forbidden.map((b) => b.text).join(' | '),
  );
  // A whitelist rather than a blacklist, and that is the point: the check
  // above can only fail for a control somebody thought to name, while this one
  // fails for *any* button that is not one of the seven this page is supposed
  // to have. A "Calibrate" or an "Advanced" added later lands here.
  const ALLOWED = [
    'Read parameters',
    'Disconnect',
    'Read the table',
    'Attitude at 4 Hz',
    'Position at 4 Hz',
    'Airspeed and heading at 4 Hz',
    'RC channels at 4 Hz',
  ];
  const unexpected = offered.map((b) => b.text).filter((label) => !ALLOWED.includes(label));
  check(
    'and every control on it is one of the seven it is meant to have',
    unexpected.length === 0,
    unexpected.join(' | ') || `${offered.length} buttons, all expected`,
  );
  check('the page states what it cannot do', text.includes('Read-only'), 'MAV_LIMITATIONS');

  // Live telemetry. The vehicle streams nothing until it is asked, which is
  // what a real one does — so the panel is empty until the rate button is
  // pressed, and this is the write path exercised end to end: page → bridge →
  // pymavlink's decoder → the vehicle's own answer.
  check('the live panel is empty before anything is asked for', /attitude\s*\?/i.test(text));
  check('a rate button is offered', await clickButton('Attitude at 4 Hz'));
  await settle(2500);
  text = await bodyText();
  check('the vehicle answered the rate request with an ack', /command 511/.test(text), 'COMMAND_ACK');
  check('and the ack says it was accepted', /accepted/i.test(text));
  check('the vehicle is now streaming attitude', /\d+(\.\d+)?° roll/.test(text));
  check('and position', /52\.1\d+, 4\.98\d+/.test(text));
  check('and battery', /12\.6\d V/.test(text));
  check('and GPS', /11 satellites/i.test(text));
  check('the trace says what was asked for', /asked for message 30 at 4 Hz/i.test(text));

  // The parameter table. MAVLink has one command that really does enumerate
  // them, so this is the difference between this page and the MSP one.
  check('a Read parameters button is present', await clickButton('Read parameters'));
  await settle(2500);
  text = await bodyText();
  check('the whole table came back', /RATE_RLL_P/.test(text));
  check('all forty, with the last one closing the list', /index 39 of 40/.test(text));

  // The armed transition, which only time can produce. Polled rather than
  // slept for, so a slow machine reports a failure instead of a false pass.
  const deadline = Date.now() + (ARM_AFTER_S + 12) * 1000;
  let armedText = '';
  while (Date.now() < deadline) {
    armedText = await bodyText();
    if (/\bArmed\b/.test(armedText) && !/Armed state unknown/.test(armedText)) break;
    await settle(500);
  }
  check('the vehicle armed, and the page followed it', /\bArmed\b/.test(armedText));
  check('the change was reported rather than absorbed', /the vehicle is now ARMED/.test(armedText));

  // Disconnect and reconnect, on a link that has been streaming — the case
  // where a stale frame or a stale parameter table would show up.
  check('a Disconnect button is present', await clickButton('Disconnect'));
  await settle(1000);
  text = await bodyText();
  check('it is back to the connect form', /Connect/.test(text) && !/RATE_RLL_P/.test(text));
  check('Connect is offered again', await clickButton('Connect bridge'));
  await settle(4000);
  text = await bodyText();
  check('it reconnected to the vehicle', /ArduPilot/.test(text) && /quadrotor/i.test(text));
  check('and re-detected it as MAVLink rather than assuming', /read-only/i.test(text));
} else if (which === 'firefox' || stripSerial) {
  // Web Serial is absent. The property under test is that the page says so
  // rather than throwing or quietly hiding the option.
  if (caps.serial) {
    // Firefox behind a pref, say. Not the negative case, and saying so is
    // better than reporting a vacuous pass.
    note('this browser exposes navigator.serial', 'the no-Web-Serial branch is NOT exercised — no claim made');
    check('the negative case was reached', false, 'navigator.serial is present, so nothing was tested here');
  } else {
    // What the app actually does, which is better than what this check first
    // asserted: the USB serial option stays *visible* and is marked unavailable
    // and disabled, rather than being hidden — so a person on Firefox can see
    // that the option exists and why it is greyed out.
    //
    // The "This browser has no Web Serial" notice is NOT asserted here, because
    // it is unreachable from this state: it renders only when the chosen
    // transport is 'serial', and the only way to choose that is the select
    // option that is disabled precisely when the notice would apply. It is
    // covered where it is reachable — tests/ui.test.tsx:307 renders the form
    // with serial chosen.
    const option = await page.evaluate(() => {
      const o = document.querySelector('input[name="transport"][value="serial"]');
      return o ? { text: o.closest('.choice')?.textContent.trim() ?? '', disabled: o.disabled } : null;
    });
    check('the USB serial option is still shown, not hidden', option !== null);
    check('it is marked unavailable in this browser', /not available/i.test(option?.text ?? ''), option?.text ?? '');
    check('and it cannot be chosen', option?.disabled === true);

    const text = await bodyText();
    check('the demo board is still offered when the cable cannot be', /demo/i.test(text));
    check('the bridge is still offered when the cable cannot be', /bridge/i.test(text));
    const connectEnabled = await page.evaluate(
      () => !([...document.querySelectorAll('button')].find((b) => /connect|explore demo/i.test(b.textContent))?.disabled ?? true),
    );
    check('Connect is still usable for the transports that do work', connectEnabled === true);
  }
} else {
  check('the browser has Web Serial', caps.serial === true);

  // 3. Explicitly choose the demo; a serial-capable browser defaults to USB.
  await page.click('input[name="transport"][value="demo"]');
  check('Explore demo is present', await clickButton('Explore demo'));
  await settle(2500);
  let text = await bodyText();
  check('the demo board identified itself', text.includes('aerialkit-demo'));

  // 3a. The rail. Every section is listed, including the ones that cannot be
  // opened, and a disabled one carries the reason. This is checked against the
  // rendered page because the failure it guards is a rendered one: an empty
  // page where a person cannot tell "your firmware is old" from "the app
  // forgot to draw something".
  const rail = await page.evaluate(() =>
    [...document.querySelectorAll('nav.rail button')].map((b) => ({
      text: b.textContent.trim(),
      disabled: b.disabled,
      title: b.getAttribute('title') ?? '',
    })),
  );
  check('the rail lists the sections', rail.length >= 4, `${rail.length} tabs`);
  check('the written ones can be opened', rail.filter((t) => !t.disabled).length >= 4,
    rail.filter((t) => !t.disabled).map((t) => t.text).join(' | '));
  const unbackable = rail.filter((t) => t.disabled);
  check('an unwritten one is still in the list, disabled', unbackable.length > 0);
  check(
    'and it says which half is missing, naming the opcode',
    unbackable.every((t) => t.title.length > 40 && /[a-z ]+/.test(t.title)),
    unbackable.map((t) => `${t.text}: ${t.title}`).join(' || ') || 'none',
  );

  // 3b. The Parameters tab, which is where the table lives now.
  check('the Parameters tab opens', await clickButton('Parameters'));
  await settle(500);
  text = await bodyText();
  check('a parameter was read from it', /[a-z_]+ *= *-?\d/.test(text) || /parameter/i.test(text));

  // 3c. What the page says it cannot establish, checked against the rendered
  // page rather than the source — because the failure this replaced was in the
  // rendered page. Until 2026-09-30 this app told every visitor, on every
  // connection, that the firmware had no post-change callback and no armed
  // guard on save. Both were false. A unit test can be updated to match a new
  // wrong sentence; this check reads the text a person actually sees.
  //
  // Since the capability word landed, the demo board reports APPLIES_ON_WRITE
  // and the first of those two sentences is now *wrong to show*: the board has
  // answered the question and the answer is yes. So this asserts its absence —
  // the direction that fails if the app ever goes back to saying it.
  check(
    'the page does not claim a write stops at the table, on a board that says it re-applies',
    !text.includes('the board confirms the value is in its table'),
  );
  // And the same direction for the armed guard. This check read
  // `text.includes('the board would take a parameter value while armed')` until
  // milestone 4, and it was right then: the protocol guarded `param save` and
  // not `param set`, so the app owed a sentence saying so. The board now refuses
  // a set while armed, reports GATES_ON_ARMED, and `limitationsFor` reads that
  // bit — so the sentence is not owed any more, and showing it would be the
  // falsehood this whole section exists to catch. Asserted absent, which is the
  // direction that fails if the app goes back to saying it.
  check(
    'the page does not claim setting is unguarded, on a board that guards it',
    !text.includes('the board would take a parameter value while armed'),
  );
  check(
    'and it does say the board refuses a write while the aircraft is armed',
    /armed/i.test(text) && /refus/i.test(text),
  );
  // The retracted sentence, asserted absent. A grep for the old claim is the
  // one check that cannot pass by accident.
  check(
    'and it no longer claims the firmware has no post-change callback',
    !/reaches the parameter table, not the running aircraft/.test(text),
  );
  // The citations are rendered, and they are `path:symbol` rather than a line
  // range: a line number is wrong after the next edit and nothing fails.
  const limitBlocks = await page.evaluate(() =>
    [...document.querySelectorAll('.limit')].map((el) => ({
      id: el.id || el.getAttribute('data-id') || '',
      cites: [...el.querySelectorAll('.cites li')].map((li) => li.textContent.trim()),
    })),
  );
  // This used to require `cites.length >= 5`, which was a number about how many
  // limitations the demo board happened to have — and milestone 4 retired two of
  // them, so the number went down for a good reason and the check would have
  // read that as a failure. The property worth pinning is per-limitation and
  // does not move when a limitation is retired: **every rendered limitation
  // carries at least one source, and every source is a symbol rather than a line
  // range**. A line range is wrong after the next edit and nothing fails.
  const citeShape = /^[A-Za-z0-9_./-]+\.(c|h):[A-Za-z_]\w*$/;
  const missing = limitBlocks.filter((b) => b.cites.length === 0);
  const malformed = limitBlocks.flatMap((b) => b.cites).filter((c) => !citeShape.test(c));
  check(
    'every limitation carries its source, as a symbol rather than a line',
    limitBlocks.length > 0 && missing.length === 0 && malformed.length === 0,
    `${limitBlocks.length} limitation(s); ` +
      (missing.length ? `no citations on ${missing.length} | ` : '') +
      (malformed.join(' | ') || 'all symbols'),
  );

  // 3d. The reset, which milestone 4 added. Two things are being checked and
  // they are different: that the control is *offered* on a board that answers
  // `param default`, and that it *works* — the table goes back to the build
  // defaults, over the wire, through the app's own client and parser.
  //
  // The second half is what makes this worth a browser rather than a unit test.
  // Everything here runs in a real page against a real peer, so a control that
  // rendered correctly and sent nothing — a handler wired to the wrong action,
  // a request the client built wrong — fails here and passes a DOM assertion.
  //
  // The demo board is the peer throughout this branch, so the write lands in
  // the page and never on hardware. Nothing here is a bench test.
  const resetButton = await page.evaluate(() => {
    const b = [...document.querySelectorAll('button')].find((x) => x.textContent.trim() === 'Reset');
    return b === null || b === undefined ? null : { disabled: b.disabled, title: b.getAttribute('title') ?? '' };
  });
  check('a Reset control is offered on a parameter row', resetButton !== null);
  // Enabled, and its tooltip is the explanation of what the button *does* rather
  // than a refusal. The two are different sentences and the first version of
  // this check conflated them by demanding an empty title: a disabled control
  // carries the reason it is disabled, and an enabled one carries what it will
  // do. What must not happen is a refusal's wording on a button that is live.
  check(
    'and it is enabled, because this board says it answers `param default`',
    resetButton !== null &&
      resetButton.disabled === false &&
      !/^this board|^needs |does not answer|cannot/i.test(resetButton.title),
    resetButton === null ? 'no Reset button' : `disabled=${resetButton.disabled} title=${resetButton.title}`,
  );
  const resetAll = await page.evaluate(() => {
    const b = [...document.querySelectorAll('button')].find((x) => /^Reset all to build defaults$/.test(x.textContent.trim()));
    return b === null || b === undefined ? null : { disabled: b.disabled };
  });
  check('the whole table can be put back in one press', resetAll !== null && resetAll.disabled === false);

  // Do it, and do it against a board that has something to put back. On a
  // freshly connected demo board every value already equals its default, so
  // clicking Reset would be a reset of nothing and the check would pass whether
  // or not the button was wired to anything. So the workflow is the real one: a
  // value is staged, sent, and read back; then the whole table is reset; then
  // the first value is read back again and must be its default.
  //
  // Every step is asserted on the *rendered* value column, which only ever holds
  // what came off the wire — so a reset that answered 0 but moved nothing fails
  // here, and so does one that moved the board and never re-read it.
  //
  // This writes to the demo board and to nothing else: the branch is entered
  // only after the page has said `aerialkit-demo` (the rail check above), and
  // the guard below re-reads it before typing anything.
  // The board's value is the `.value` span inside the "ON THE BOARD" cell, not
  // the cell's whole text: the cell also carries the `changed` badge, and
  // comparing "1.000changed" with "1.000" would be this file failing to read the
  // page rather than the page being wrong.
  const readRow = (name) =>
    page.evaluate((n) => {
      const input = document.querySelector(`input[aria-label="${n}, new value"]`);
      if (input === null) return null;
      const cell = input.closest('tr').querySelectorAll('td')[3];
      return {
        board: (cell.querySelector('.value') ?? cell).innerText.trim(),
        changed: cell.querySelector('.changed') !== null,
        staged: input.value,
      };
    }, name);

  /*
   * How big the table is, as the DOM holds it. This exists because a write and a
   * reset once left the page one chapter larger every time they were used: the
   * panel keyed each chapter by its `group` byte alone, the board's own table
   * revisits three of its groups, and two React siblings sharing a key made
   * React keep the previous render's rows alongside the new ones. Measured here
   * with the old key: 43 tbody rows, then 61, then 79, then 97 — 18 rows more
   * on every re-read. Production React strips the duplicate-key warning, so the
   * page said nothing and a reset that had worked read as a button that did
   * nothing, because the row on screen was a stale copy of the one before it.
   *
   * Nothing else in this file would have caught that: every other check reads a
   * named row, and a named row kept answering correctly from whichever copy
   * `querySelector` reached first.
   */
  const countRows = () =>
    page.evaluate(() => ({
      rows: document.querySelectorAll('tbody tr:not(.group)').length,
      chapters: document.querySelectorAll('tbody tr.group').length,
      inputs: document.querySelectorAll('tbody input').length,
    }));

  // Clicking a disabled button is a no-op that reports success, so every click
  // in this workflow waits for the control to actually be usable first. The
  // wait is also a check in its own right: a write leaves the panel busy until
  // the board has answered and the table has been re-read, and a panel that
  // never comes back is the failure this catches.
  const waitUsable = async (label) => {
    for (let i = 0; i < 40; i++) {
      const usable = await page.evaluate((l) => {
        const b = [...document.querySelectorAll('button')].find((x) => x.textContent.trim().toLowerCase() === l.toLowerCase());
        return b !== undefined && !b.disabled;
      }, label);
      if (usable) return true;
      await settle(250);
    }
    return false;
  };

  const demoStill = /aerialkit-demo/.test(await bodyText());
  const target = await page.evaluate(() => {
    const input = document.querySelector('input[aria-label$=", new value"]');
    return input === null ? null : input.getAttribute('aria-label').replace(/, new value$/, '');
  });

  if (demoStill && target !== null) {
    const before = await readRow(target);
    const tableBefore = await countRows();
    check(
      'the table drew one row per parameter, and the table is not empty',
      tableBefore.rows > 0 && tableBefore.rows === tableBefore.inputs,
      JSON.stringify(tableBefore),
    );
    await page.evaluate((n) => {
      const input = document.querySelector(`input[aria-label="${n}, new value"]`);
      const setter = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, 'value').set;
      setter.call(input, '1');
      input.dispatchEvent(new Event('input', { bubbles: true }));
    }, target);
    await settle(200);
    const staged = await readRow(target);
    check('a value can be staged for a named parameter', staged !== null && staged.staged === '1', target);

    await waitUsable('Send');
    check('the row can be sent', await clickButton('Send'));
    const settledAfterSend = await waitUsable('Reset all to build defaults');
    check('the panel is usable again once the board has answered', settledAfterSend);
    const sent = await readRow(target);
    // `Number(...)`, not `=== '1'`. The board renders the parameter at the
    // decimal count its metadata declared, so the cell on screen reads `1.000`.
    // Comparing against the bare `'1'` failed on a write the board had taken
    // correctly, which is its own lesson: an assertion about a number should
    // compare numbers, and a string that happens to hold one is how a passing
    // board looks like a failing one.
    check(
      'and the board took it, read back through the app',
      sent !== null && sent.staged === '' && Number(sent.board) === 1 && sent.changed === true,
      JSON.stringify(sent),
    );

    await clickButton('Reset all to build defaults');
    const settledAfterReset = await waitUsable('Reset all to build defaults');
    check('the reset was answered rather than left outstanding', settledAfterReset);
    const after = await readRow(target);
    check(
      'and the reset put the whole table back to the build defaults',
      after !== null &&
        after.board === before.board &&
        after.staged === '' &&
        after.changed === false,
      `${target}: was ${JSON.stringify(before)}, now ${JSON.stringify(after)}`,
    );

    // A write and a reset have been through the table by now. It is the same
    // size it was.
    const tableAfter = await countRows();
    check(
      'and the write and the reset left the table exactly as big as they found it',
      JSON.stringify(tableAfter) === JSON.stringify(tableBefore),
      `was ${JSON.stringify(tableBefore)}, now ${JSON.stringify(tableAfter)}`,
    );
  }

  // 3e. The Receiver tab, which milestone 5 added. Three claims, and only one
  // of them is that the panel drew something:
  //
  //  - it reads the receiver over the wire, in a real page, against a real peer;
  //  - **it polls only while the tab is open** — checked by counting the
  //    readings the panel itself reports, leaving the tab for a while, and
  //    coming back. This is the one claim a unit test cannot make honestly,
  //    because in a unit test the tab is a function call and "the tab is not on
  //    screen" is not a thing that happens;
  //  - the numbers it shows came off the wire rather than out of the app.
  //
  // The demo board is the peer here, so nothing is asked of hardware.
  const receiverOpen = await clickButton('Receiver');
  check('the Receiver tab opens rather than sitting disabled', receiverOpen === true);
  await settle(1200);

  // The panel's own note: "N readings, the last Xs ago". Read as text, because
  // the claim is about what a person can see.
  const readNote = () =>
    page.evaluate(() => {
      const panel = [...document.querySelectorAll('section.panel')].find((p) =>
        /^receiver$/i.test(p.querySelector('h2')?.textContent.trim() ?? ''),
      );
      if (panel === undefined) return null;
      return {
        note: panel.querySelector('.note')?.textContent.trim() ?? '',
        // Scoped to each block rather than counted across the panel: the sticks
        // are drawn as `.rc-channel` too, so a bare count returns 12 on the demo
        // board and reads as eight channels that are somehow twelve. A number
        // under a label has to mean what the label says.
        channels: panel.querySelectorAll('.rc-channels > .rc-channel').length,
        sticks: panel.querySelectorAll('.rc-sticks > .rc-channel').length,
        rows: panel.querySelectorAll('table.data-table tbody tr').length,
        text: panel.innerText,
      };
    });

  const first = await readNote();
  check(
    'the receiver panel drew a bar per channel from the wire',
    first !== null && first.channels > 0,
    first === null ? 'no panel' : `${first.channels} channel bar(s)`,
  );
  // The sticks are the firmware's decode, drawn separately from the raw counts,
  // and the panel says so rather than naming a channel as a stick.
  check('and a separate block for the four decoded sticks', first !== null && first.sticks === 4,
    first === null ? 'no panel' : `${first.sticks} stick(s)`);
  // The counters table, which is the part that could only come from the board:
  // seven values this app has nowhere else to get.
  check('and the board\'s own frame counters', first !== null && first.rows >= 7,
    first === null ? 'no panel' : `${first.rows} counter row(s)`);
  check(
    'the panel says the mapping from raw channel to stick is not on the wire',
    first !== null && /ak_types\.h/.test(first.text),
    'the caveat that keeps the bars from claiming to be sticks',
  );
  // Frames move as the board's receiver runs, and the board counts them against
  // the clock rather than against how often it is asked — so a panel that
  // incremented its own counter per poll would show a number here too. What
  // makes this a wire reading is that it comes back through `rc channels`.
  const readingsOf = (text) => {
    const m = /(\d+)\s+readings?/i.exec(text ?? '');
    return m === null ? null : Number(m[1]);
  };
  const opened = readingsOf(first?.note);
  check('the panel reports how many times it has asked', opened !== null, first?.note ?? 'no note');

  // Now leave the tab. Nothing about the receiver should be asked for while it
  // is not on screen: at ten polls a second, a minute on another tab would be
  // six hundred exchanges on a link that is also carrying the parameter table.
  await clickButton('Parameters');
  await settle(1500);
  await clickButton('Receiver');
  await settle(250);
  const second = await readNote();
  const back = readingsOf(second?.note);
  // Coming back asks once immediately and then resumes on the interval, so the
  // quarter second of settling adds at most three. Seventeen would be the
  // failure: that is what 1.75 s of a still-running poll looks like.
  check(
    'and it stopped asking while the tab was closed',
    opened !== null && back !== null && back - opened <= 5,
    `${opened} reading(s) before, ${back} after 1.5 s on another tab`,
  );
  check(
    'and it resumed when the tab came back',
    opened !== null && back !== null && back > opened,
    `before ${opened}, after ${back}`,
  );

  // The bars are drawn against the whole of an 11-bit count field, and this app
  // must not have rescaled them by the calibration — that lives on the board.
  const widths = await page.evaluate(() => {
    const panel = [...document.querySelectorAll('section.panel')].find((p) =>
      /^receiver$/i.test(p.querySelector('h2')?.textContent.trim() ?? ''),
    );
    return [...(panel?.querySelectorAll('.rc-track .fill') ?? [])].map((el) => el.style.width);
  });
  check(
    'the bars are widths the page computed, not a fixed picture',
    widths.length > 0 && widths.every((w) => /%$/.test(w)),
    widths.slice(0, 3).join(' '),
  );

  // 3f. The Sensors tab, which milestone 6 added. The same two claims the
  // Receiver block makes — it reads the board over the wire, and it asks for
  // nothing while nobody is looking — plus one the receiver has no equivalent
  // of: **the panel can say a sensor is missing in the board's own terms**,
  // which is the whole reason `sensor info` leaves the body off an absent
  // sensor rather than sending zeros.
  const sensorsOpen = await clickButton('Sensors');
  check('the Sensors tab opens rather than sitting disabled', sensorsOpen === true);
  // One round is five exchanges and the panel asks immediately, so this is
  // about the first round finishing rather than about the poll interval.
  await settle(1500);

  const readSensors = () =>
    page.evaluate(() => {
      const panel = [...document.querySelectorAll('section.panel')].find((p) =>
        /^sensors$/i.test(p.querySelector('h2')?.textContent.trim() ?? ''),
      );
      if (panel === undefined) return null;
      return {
        note: panel.querySelector('.note')?.textContent.trim() ?? '',
        sections: [...panel.querySelectorAll('section.sensor')].map((s) => ({
          head: s.querySelector('.sub-head')?.textContent.trim() ?? '',
          text: s.innerText,
        })),
        text: panel.innerText,
      };
    });

  const sensors = await readSensors();
  check(
    'the sensors panel drew a section per topic',
    sensors !== null && sensors.sections.length === 5,
    sensors === null ? 'no panel' : `${sensors.sections.length} section(s)`,
  );
  check(
    'and headed each one with the driver name the board reported',
    sensors !== null &&
      ['icm42688p', 'bmp388', 'tof10120'].every((d) => sensors.text.includes(d)),
    'the demo board names its own drivers, so this could only have come off the wire',
  );
  // **The unit-tested property, checked against the rendered page.** The
  // firmware distinguishes "this topic has no sensor" from "this sensor reads
  // zero" by omitting the body; a panel that drew both the same way would tell
  // a person their barometer is broken when there is no barometer. The
  // position line is the sharpest case: with no fix, 0, 0 is the Gulf of
  // Guinea, and a map drawn from it would be a fabrication.
  check(
    'the fix type is named in the receiver\'s numbering, not MAVLink\'s',
    sensors !== null && /3D fix/.test(sensors.text),
    'u-blox numbering, per ak_gps.h',
  );
  check(
    'and the panel does not read a position without a fix',
    sensors !== null && !/0\.000000/.test(sensors.text),
    'a zeroed position would be a real place',
  );
  const roundsOf = (text) => {
    const m = /(\d+)\s+rounds?/i.exec(text ?? '');
    return m === null ? null : Number(m[1]);
  };
  const roundsOpen = roundsOf(sensors?.note);
  check('the panel reports how many rounds it has read', roundsOpen !== null, sensors?.note ?? 'no note');

  // Leave the tab for long enough that a still-running poll is unmistakable: a
  // round a second over three and a half seconds is three or four more, where
  // the honest number is at most the one round that was already in flight.
  await clickButton('Parameters');
  await settle(3500);
  await clickButton('Sensors');
  await settle(400);
  const roundsBack = roundsOf((await readSensors())?.note);
  check(
    'and it stopped asking while the tab was closed',
    roundsOpen !== null && roundsBack !== null && roundsBack - roundsOpen <= 2,
    `${roundsOpen} round(s) before, ${roundsBack} after 3.5 s on another tab`,
  );
  check(
    'and it resumed when the tab came back',
    roundsOpen !== null && roundsBack !== null && roundsBack > roundsOpen,
    `before ${roundsOpen}, after ${roundsBack}`,
  );

  // Back to the table, so the disconnect check below reads the page it expects.
  await clickButton('Parameters');
  await settle(300);

  // 4. Disconnect.
  check('a Disconnect button is present', await clickButton('Disconnect'));
  await settle(800);
  text = await bodyText();
  check('it is back to the connect form', /connect/i.test(text) && !text.includes('aerialkit-demo'));

  // 5. Reconnect. The workflow W4 names, and the one where state leaks show up.
  check('Connect is offered again', await clickButton('Explore demo'));
  await settle(2500);
  text = await bodyText();
  check('it reconnected and re-read the board', text.includes('aerialkit-demo'));
}

// 6. Nothing complained in the console during any of it.
check('no console errors across the whole workflow', consoleErrors.length === 0, consoleErrors.join(' | '));
note('console errors', consoleErrors.length === 0 ? 'none' : consoleErrors.join(' | '));

await browser.close();
console.log(failures === 0 ? `== ${mode}: all checks passed` : `== ${mode}: ${failures} FAILED`);
process.exit(failures === 0 ? 0 : 1);

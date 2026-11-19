import { afterEach, describe, expect, it } from 'vitest';
import { Address, CommandStep } from '../src/protocol/crsf';
import { bindingUid, parseUid } from '../src/protocol/elrs-binding';
import { RadioSession, RadioWriteRefused } from '../src/session/radio';
import { DEMO_RADIO_NAME, DemoRadioTransport } from '../src/transport/demo-radio';

const open: DemoRadioTransport[] = [];
async function connect(options = {}) {
  const t = new DemoRadioTransport({ linkStatsMs: 20, ...options });
  await t.open();
  open.push(t);
  return { t, s: new RadioSession(t, 300) };
}
afterEach(async () => {
  for (const t of open.splice(0)) await t.close();
});

describe('radio session against the demo ExpressLRS module', () => {
  it('identifies the module and reads its whole menu, chunked entries included', async () => {
    const { t, s } = await connect({ chunkMax: 12 });
    const info = await s.identify();
    expect(info.name).toBe(DEMO_RADIO_NAME);
    expect(info.parameterCount).toBe(12);
    const menu = await s.readAll();
    expect(menu.size).toBe(12);
    expect(menu.get(1)).toMatchObject({ kind: 'selection', name: 'Packet Rate', value: 2,
      options: ['50Hz', '100Hz', '150Hz', '250Hz', '333Hz Full', '500Hz'] });
    expect(menu.get(4)).toMatchObject({ kind: 'folder', name: 'TX Power', children: [5, 6] });
    expect(menu.get(5)).toMatchObject({ kind: 'selection', parent: 4, units: 'mW' });
    expect(menu.get(8)).toMatchObject({ kind: 'command', name: 'Bind', step: CommandStep.IDLE });
    expect(menu.get(9)).toMatchObject({ kind: 'info', value: '4.1.0 AK-A3' });
    expect(menu.get(11)).toMatchObject({ kind: 'command', name: 'Enable WiFi' });
    expect(s.snapshot().loading).toBeNull();
    expect(s.snapshot().role).toBe('tx');
    expect(await s.readUid()).toEqual(t.radio.uid);
  });

  it('refuses writes until changes are allowed, then writes and reads back', async () => {
    const { t, s } = await connect();
    await s.readAll();
    await expect(s.write(1, 4)).rejects.toBeInstanceOf(RadioWriteRefused);
    expect(t.radio.menu[0]).toMatchObject({ value: 2 });          // untouched
    s.allowWrites = true;
    await expect(s.write(1, 99)).rejects.toBeInstanceOf(RangeError);
    const after = await s.write(1, 4);
    expect(after).toMatchObject({ value: 4 });
    expect(t.radio.menu[0]).toMatchObject({ value: 4 });
  });

  it('runs Bind through its steps', async () => {
    const { t, s } = await connect();
    await s.readAll();
    await expect(s.runCommand(8)).rejects.toBeInstanceOf(RadioWriteRefused);
    s.allowWrites = true;
    const done = await s.runCommand(8);
    expect(done).toMatchObject({ kind: 'command', step: CommandStep.IDLE, info: 'Bound' });
    expect(t.radio.receiverConnected).toBe(true);
  });

  it('sends the direct CRSF bind command only after writes are allowed', async () => {
    const { t, s } = await connect();
    await s.identify();
    await expect(s.bind()).rejects.toBeInstanceOf(RadioWriteRefused);
    expect(t.radio.directBindRequests).toBe(0);

    s.allowWrites = true;
    await s.bind();
    expect(t.radio.directBindRequests).toBe(1);
    expect(t.radio.directBindDestinations).toEqual([Address.CRSF_TRANSMITTER]);
    expect(t.radio.receiverConnected).toBe(false);
  });

  it('confirms the TX Wi-Fi command after the person confirms it', async () => {
    const { t, s } = await connect();
    await s.readAll();
    s.allowWrites = true;
    const result = await s.enableModuleWifi();
    expect(t.radio.moduleWifiRequested).toBe(true);
    expect(result).toMatchObject({
      kind: 'command',
      step: CommandStep.EXECUTING,
      info: 'WiFi Running...',
    });
  });

  it('detects the RX tree and addresses direct bind commands to the receiver', async () => {
    const { t, s } = await connect({ role: 'rx' });
    await s.identify();
    const menu = await s.readAll();
    expect(s.snapshot().role).toBe('rx');
    expect(menu.get(1)).toMatchObject({ kind: 'selection', name: 'Bind Storage' });
    expect(menu.get(2)).toMatchObject({ kind: 'command', name: 'Enter Bind Mode' });

    s.allowWrites = true;
    await s.bind();
    expect(t.radio.directBindRequests).toBe(1);
    expect(t.radio.directBindDestinations).toEqual([Address.CRSF_RECEIVER]);
  });

  it('reads and writes the UID separately from explicit bind mode', async () => {
    const { t, s } = await connect();
    await s.readAll();
    s.allowWrites = true;
    await s.setBindingPhrase('test');
    expect(t.radio.uid).toEqual(bindingUid('test'));
    expect(await s.readUid()).toEqual(bindingUid('test'));
    await s.setUid(parseUid('aa:bb:cc:dd:ee:ff'));
    expect(t.radio.uid).toEqual(parseUid('aa:bb:cc:dd:ee:ff'));
    expect(s.snapshot().uid).toBeNull();
  });

  it('shows link statistics as they arrive, and an age when they stop', async () => {
    const { t, s } = await connect();
    await new Promise((r) => setTimeout(r, 80));
    const snap = s.snapshot();
    expect(snap.link).toMatchObject({ uplinkLinkQuality: 100, downlinkLinkQuality: 100 });
    expect(snap.linkAgeMs).toBeLessThan(80);
    t.radio.receiverConnected = false;
    await new Promise((r) => setTimeout(r, 120));
    expect(s.snapshot().linkAgeMs).toBeGreaterThanOrEqual(80);
  });

  it('says so when nothing answers', async () => {
    const { t, s } = await connect();
    await t.close();
    await expect(s.identify(1)).rejects.toThrow(/did not answer a device ping/);
  });
});

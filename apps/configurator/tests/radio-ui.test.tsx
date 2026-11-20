import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it } from 'vitest';
import { DEMO_RADIO_NAME, DEMO_RADIO_RX_NAME } from '../src/transport/demo-radio';
import { App } from '../src/ui/App';

describe('the radio module page', () => {
  it('is its own device: connect the demo radio, read its menu, change a setting only when allowed', async () => {
    const user = userEvent.setup();
    render(<App />);
    await user.selectOptions(screen.getByRole('combobox', { name: 'Device' }), 'radio');
    // The flight controller's connection controls are gone; the radio has its own.
    expect(screen.queryByRole('button', { name: /^Connect$/ })).toBeNull();
    await user.selectOptions(screen.getByRole('combobox', { name: 'Radio connection' }), 'demo');
    await user.click(screen.getByRole('button', { name: /Connect radio/ }));
    await screen.findByText(DEMO_RADIO_NAME, {}, { timeout: 4000 });
    const rate = await screen.findByLabelText('Packet Rate', {}, { timeout: 4000 });
    expect((rate as HTMLSelectElement).value).toBe('2');
    expect(rate).toBeDisabled();                                     // read-only until allowed
    expect(screen.getByText('TX Power')).toBeInTheDocument();       // a folder
    expect(screen.getByText('4.1.0 AK-A3')).toBeInTheDocument();    // an info field
    expect(screen.getByText('Binding and Wi-Fi')).toBeInTheDocument();
    expect(screen.getByText('Bench')).toBeInTheDocument();
    expect(screen.getByText(/TX transmitter/)).toBeInTheDocument();
    expect(screen.getByRole('button', { name: 'Enable module Wi-Fi' })).toBeInTheDocument();
    expect(screen.getByRole('button', { name: 'Enable receiver Wi-Fi' })).toBeInTheDocument();
    expect(screen.queryByRole('button', { name: 'Run' })).not.toBeInTheDocument();
    await user.click(screen.getByRole('checkbox', { name: 'Allow changes' }));
    await waitFor(() => expect(rate).toBeEnabled());
    await user.selectOptions(rate, '4');
    await waitFor(() => expect((screen.getByLabelText('Packet Rate') as HTMLSelectElement).value).toBe('4'));
    await user.type(screen.getByLabelText('Binding phrase'), 'test');
    expect(screen.getByText('4f:04:fd:82:21:55')).toBeInTheDocument();
    await user.click(screen.getByRole('button', { name: 'Set phrase' }));
    expect(screen.getByText(/Confirm radio identity change/)).toBeInTheDocument();
    await user.click(screen.getByRole('button', { name: 'Confirm' }));
    await screen.findByText(/Verified TX phrase readback/);
    await user.click(screen.getByRole('button', { name: 'Enable module Wi-Fi' }));
    expect(screen.getByText(/Enable Wi-Fi now/)).toBeInTheDocument();
    await user.click(screen.getByRole('button', { name: 'Cancel' }));
    await user.click(screen.getByRole('button', { name: 'Start binding mode' }));
    expect(screen.getByText('Confirm bind request')).toBeInTheDocument();
    expect(screen.getByText(/Enter explicit binding mode/)).toBeInTheDocument();
    await user.click(screen.getByRole('button', { name: 'Cancel' }));
    await user.click(screen.getByRole('button', { name: 'Disconnect' }));
    await screen.findByRole('button', { name: /Connect radio/ });

    // Phrase verification is per endpoint. Keep the TX evidence, reconnect the
    // RX demo with the same phrase, and only then report the pair as eligible.
    await user.selectOptions(screen.getByRole('combobox', { name: 'Radio connection' }), 'demo-rx');
    await user.click(screen.getByRole('button', { name: /Connect radio/ }));
    await screen.findByText(DEMO_RADIO_RX_NAME, {}, { timeout: 4000 });
    await user.click(screen.getByRole('checkbox', { name: 'Allow changes' }));
    await user.type(screen.getByLabelText('Binding phrase'), 'test');
    await user.click(screen.getByRole('button', { name: 'Set phrase' }));
    await user.click(screen.getByRole('button', { name: 'Confirm' }));
    await screen.findByText(/Verified RX phrase readback/);
    expect(screen.getByText(/TX and RX phrase-derived UIDs match/)).toBeInTheDocument();
  }, 10_000);

  it('drives the TX from the Bench page with explicit arm confirmation and safe stop', async () => {
    const user = userEvent.setup();
    render(<App />);
    await user.selectOptions(screen.getByRole('combobox', { name: 'Device' }), 'radio');
    await user.selectOptions(screen.getByRole('combobox', { name: 'Radio connection' }), 'demo');
    await user.click(screen.getByRole('button', { name: /Connect radio/ }));
    await screen.findByText(DEMO_RADIO_NAME, {}, { timeout: 4000 });

    expect(screen.getByText('Bench')).toBeInTheDocument();
    const start = screen.getByRole('button', { name: 'Start continuous drive' });
    expect(start).toBeDisabled();
    await user.click(screen.getByRole('checkbox', { name: 'Allow changes' }));
    await waitFor(() => expect(start).toBeEnabled());
    await user.selectOptions(screen.getByLabelText('Pattern'), 'arm-test');
    expect(start).toBeDisabled();
    await user.click(screen.getByRole('checkbox', {
      name: 'I confirm the controlled arm sequence and safe area',
    }));
    expect(start).toBeEnabled();
    await user.click(start);
    await screen.findByText(/Continuous drive is running|TX drive/);
    await user.click(screen.getByRole('button', { name: 'Stop and disarm' }));
    await screen.findByText(/safe disarm confirmed/, {}, { timeout: 3000 });
    expect(screen.getAllByText(/RFC? link remains NOT TESTED here|rf_link:/i).length).toBeGreaterThan(0);
    expect(screen.getAllByText(/^rate /).length).toBeGreaterThan(0);
    expect(screen.getByText('CRC 0')).toBeInTheDocument();
    expect(screen.getByText('resyncs 0')).toBeInTheDocument();
  });

  it('captures live RX channels and telemetry injection evidence without claiming an RF link', async () => {
    const user = userEvent.setup();
    render(<App />);
    await user.selectOptions(screen.getByRole('combobox', { name: 'Device' }), 'radio');
    await user.selectOptions(screen.getByRole('combobox', { name: 'Radio connection' }), 'demo-rx');
    await user.click(screen.getByRole('button', { name: /Connect radio/ }));
    await screen.findByText(DEMO_RADIO_RX_NAME, {}, { timeout: 4000 });
    await user.click(screen.getByRole('checkbox', { name: 'Allow changes' }));
    await user.click(screen.getByRole('button', { name: 'Start capture' }));
    await screen.findByLabelText('Captured channels', {}, { timeout: 3000 });
    expect(screen.getByText('CH1 992')).toBeInTheDocument();
    await screen.findByText(/telemetry_injection:/, {}, { timeout: 3000 });
    await user.click(screen.getByRole('button', { name: 'Stop capture' }));
    await screen.findByText(/rf_link:/, {}, { timeout: 3000 });
  });
});

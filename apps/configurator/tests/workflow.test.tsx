import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it, vi } from 'vitest';
import { App } from '../src/ui/App';
import { ParametersView } from '../src/ui/views/Parameters';
import { AerialKitSession } from '../src/session/aerialkit';
import { DemoTransport } from '../src/transport/demo';

describe('configuration workflow', () => {
  it('keeps edits separate from saving and offers a review across tabs', async () => {
    const user = userEvent.setup();
    render(<App />);
    await user.click(screen.getByRole('button', { name: 'Open demo' }));
    await screen.findByText('aerialkit-demo');
    await user.click(screen.getByRole('button', { name: 'Parameters' }));
    await user.type(await screen.findByRole('textbox', { name: 'rate_kp_roll, new value' }), '0.500');
    expect(screen.getByRole('button', { name: /Save to flash/ })).toBeDisabled();
    expect(screen.getByText('1 staged edit')).toBeInTheDocument();
    await user.click(screen.getByRole('button', { name: 'Attitude' }));
    await user.click(screen.getByRole('button', { name: 'Review edits (1)' }));
    expect(screen.getByRole('textbox', { name: 'rate_kp_roll, new value' })).toHaveValue('0.500');
    expect(screen.queryByRole('textbox', { name: 'rate_kp_pitch, new value' })).toBeNull();
    await user.click(screen.getByRole('button', { name: 'Discard all staged' }));
    await waitFor(() => expect(screen.getByRole('button', { name: /Save to flash/ })).toBeEnabled());
    await user.click(screen.getByRole('button', { name: 'Show all' }));
    await user.click(screen.getByRole('button', { name: 'Attitude' }));
    await user.click(screen.getByRole('button', { name: 'Parameters' }));
    expect(screen.getByRole('textbox', { name: 'rate_kp_pitch, new value' })).toBeInTheDocument();
    await user.type(screen.getByRole('textbox', { name: 'rate_kp_roll, new value' }), '0.500');
    await user.click(screen.getByRole('button', { name: 'Review edits (1)' }));
    expect(screen.queryByRole('textbox', { name: 'rate_kp_pitch, new value' })).toBeNull();
    await user.click(screen.getByRole('button', { name: 'Disconnect' }));
    await user.click(await screen.findByRole('button', { name: 'Open demo' }));
    await screen.findByText('aerialkit-demo');
    expect(screen.getByRole('heading', { level: 1, name: 'Attitude' })).toBeInTheDocument();
    await user.click(screen.getByRole('button', { name: 'Parameters' }));
    expect(await screen.findByRole('textbox', { name: 'rate_kp_pitch, new value' })).toBeInTheDocument();
  }, 15000);

  it('filters by the board group without losing staged edits outside that group', async () => {
    const session = new AerialKitSession(new DemoTransport());
    await session.open();
    await session.refresh();
    session.edit(0, '0.500');
    const actions = { edit: vi.fn(), revert: vi.fn(), reread: vi.fn(), write: vi.fn(), writeAll: vi.fn(), reset: vi.fn() };
    const user = userEvent.setup();
    try {
      render(<ParametersView snapshot={session.snapshot} actions={actions} busy={false} />);
      await user.selectOptions(screen.getByRole('combobox', { name: 'Parameter group' }), 'arming');
      expect(screen.queryByRole('textbox', { name: 'rate_kp_roll, new value' })).toBeNull();
      expect(screen.getByRole('textbox', { name: 'arm_hold_ms, new value' })).toBeInTheDocument();
      await user.click(screen.getByRole('button', { name: 'Discard all staged' }));
      expect(actions.revert).toHaveBeenCalledTimes(1);
      expect(actions.revert).toHaveBeenCalledWith(0);
    } finally { await session.close(); }
  });
});

import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it, vi } from 'vitest';
import { CHIP_ID, type FlashSegment, type Loader } from '../src/firmware/flash';
import { FlashApp } from '../src/ui/FlashApp';

function image(size: number, chip: number = CHIP_ID.ESP32_S3): Uint8Array {
  const b = new Uint8Array(size);
  b[0] = 0xe9;
  b[12] = chip;
  return b;
}
function table(): Uint8Array {
  const b = new Uint8Array(96).fill(0xff);
  const v = new DataView(b.buffer);
  [['otadata', 1, 0, 0xe000, 0x2000], ['app0', 0, 0x10, 0x10000, 0x1e0000]].forEach(([n, t, st, o, sz], i) => {
    b.set([0xaa, 0x50, t as number, st as number], i * 32);
    v.setUint32(i * 32 + 4, o as number, true);
    v.setUint32(i * 32 + 8, sz as number, true);
    b.fill(0, i * 32 + 12, i * 32 + 28);
    b.set(new TextEncoder().encode(n as string), i * 32 + 12);
  });
  return b;
}
const file = (name: string, bytes: Uint8Array) => new File([bytes.slice().buffer as ArrayBuffer], name);

describe('the flash firmware page', () => {
  it('checks the files, shows where they go, and flashes only after the note is acknowledged', async () => {
    const user = userEvent.setup();
    const written: FlashSegment[][] = [];
    const loader: Loader = {
      connect: vi.fn(async () => 'ESP32-S3'),
      write: vi.fn(async (s: readonly FlashSegment[], p) => { written.push([...s]); p(0, 1, 1); }),
      finish: vi.fn(async () => {}),
    };
    render(<FlashApp serialAvailable makeLoader={async () => loader} />);
    await user.selectOptions(screen.getByLabelText('Firmware'), 'radio-rx');
    await user.upload(screen.getByLabelText('Image files'), [
      file('bootloader.bin', image(4096)), file('partitions.bin', table()),
      file('boot_app0.bin', new Uint8Array(8192)), file('firmware.bin', image(100000))]);
    const plan = await screen.findByRole('table', { name: 'Flash plan' });
    expect(plan.textContent).toContain('0x10000');
    expect(plan.textContent).toContain('0x0e000');
    const go = screen.getByRole('button', { name: /Flash Radio receiver/ });
    expect(go).toBeDisabled();                                       // the note first
    await user.click(screen.getByLabelText('I have read this and the board is ready'));
    await user.click(go);
    await screen.findByText(/written and the board reset/);
    expect(written[0]?.map((s) => s.offset)).toEqual([0x0, 0x8000, 0xe000, 0x10000]);
  });

  it('refuses images for the wrong chip without touching the board', async () => {
    const user = userEvent.setup();
    const loader: Loader = { connect: vi.fn(async () => 'ESP32-S3'), write: vi.fn(), finish: vi.fn(async () => {}) };
    render(<FlashApp serialAvailable makeLoader={async () => loader} />);
    await user.upload(screen.getByLabelText('Image files'), [
      file('bootloader.bin', image(4096, CHIP_ID.ESP32)), file('partitions.bin', table()), file('firmware.bin', image(1000))]);
    await screen.findByText(/built for ESP32, not ESP32-S3/);
    await user.click(screen.getByLabelText('I have read this and the board is ready'));
    expect(screen.getByRole('button', { name: /Flash Board diagnostic/ })).toBeDisabled();
    await waitFor(() => expect(loader.connect).not.toHaveBeenCalled());
  });
});

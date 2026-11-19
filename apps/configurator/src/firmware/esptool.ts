import type { FlashSegment, Loader } from './flash';

/**
 * The flasher's ROM loader in the browser: Espressif's esptool-js over Web
 * Serial. Loaded on demand so the configurator's main bundle does not carry it.
 *
 * The port is chosen by the person (Web Serial's chooser) and opened by
 * esptool-js at the ROM loader's rate; the board must be in its ROM
 * bootloader (hold BOOT while plugging in, or let esptool reset it).
 */
export async function esptoolLoader(baudrate = 921600): Promise<Loader> {
  const serial = (navigator as unknown as { serial?: { requestPort(): Promise<SerialPort> } }).serial;
  if (serial === undefined) throw new Error('this browser has no Web Serial: use Chrome, Edge or Opera on a desktop');
  const port = await serial.requestPort();
  const { ESPLoader, Transport } = await import('esptool-js');
  const transport = new Transport(port, false);
  const loader = new ESPLoader({ transport, baudrate, romBaudrate: 115200, terminal: undefined });
  const binary = (bytes: Uint8Array) => {
    let s = '';
    for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
    return s;
  };
  return {
    async connect() {
      await loader.main();
      return loader.chip.CHIP_NAME;
    },
    async write(segments: readonly FlashSegment[], progress) {
      await loader.writeFlash({
        fileArray: segments.map((s) => ({ data: binary(s.bytes), address: s.offset })),
        flashSize: 'keep', flashMode: 'keep', flashFreq: 'keep',
        eraseAll: false, compress: true, reportProgress: progress,
      });
    },
    async finish() {
      try {
        await loader.after('hard_reset');
      } finally {
        await transport.disconnect();
      }
    },
  };
}

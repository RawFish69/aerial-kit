import { describe, expect, it, vi } from 'vitest';
import {
  CHIP_ID,
  FLASH_TARGETS,
  FlashRefused,
  checkImage,
  flash,
  parsePartitions,
  planFlash,
  type FlashSegment,
  type Loader,
} from '../src/firmware/flash';

/** An ESP image header: magic 0xE9, segment count, flash mode/size, entry, then chip id at 12-13. */
function image(size: number, chipId: number = CHIP_ID.ESP32_S3): Uint8Array {
  const b = new Uint8Array(size);
  b[0] = 0xe9;
  b[1] = 3;
  b[2] = 0x02;
  b[3] = 0x3f;
  b[12] = chipId & 0xff;
  b[13] = chipId >> 8;
  return b;
}

/** The partition table the Rev A3 ExpressLRS build writes (read from a real partitions.bin). */
function partitionTable(): Uint8Array {
  const rows: Array<[string, number, number, number, number]> = [
    ['nvs', 1, 2, 0x9000, 0x5000], ['otadata', 1, 0, 0xe000, 0x2000], ['app0', 0, 0x10, 0x10000, 0x1e0000],
    ['app1', 0, 0x11, 0x1f0000, 0x1e0000], ['spiffs', 1, 0x82, 0x3d0000, 0x20000], ['coredump', 1, 3, 0x3f0000, 0x10000],
  ];
  const b = new Uint8Array(3072).fill(0xff);
  const v = new DataView(b.buffer);
  rows.forEach(([name, type, sub, off, size], i) => {
    const at = i * 32;
    b.set([0xaa, 0x50, type, sub], at);
    v.setUint32(at + 4, off, true);
    v.setUint32(at + 8, size, true);
    b.fill(0, at + 12, at + 28);
    b.set(new TextEncoder().encode(name), at + 12);
    v.setUint32(at + 28, 0, true);
  });
  return b;
}

const goodFiles = () => [
  { name: 'bootloader.bin', bytes: image(15104) },
  { name: 'partitions.bin', bytes: partitionTable() },
  { name: 'boot_app0.bin', bytes: new Uint8Array(8192).fill(1) },
  { name: 'firmware.bin', bytes: image(1123888) },
];

describe('the flash plan', () => {
  it('reads the partition table the build wrote', () => {
    const parts = parsePartitions(partitionTable());
    expect(parts.map((p) => [p.name, p.offset])).toEqual([
      ['nvs', 0x9000], ['otadata', 0xe000], ['app0', 0x10000], ['app1', 0x1f0000], ['spiffs', 0x3d0000], ['coredump', 0x3f0000]]);
  });

  it('places each image where the partition table says', () => {
    const plan = planFlash(goodFiles());
    expect(plan.problems).toEqual([]);
    expect(plan.segments.map((s) => [s.role, s.offset])).toEqual([
      ['bootloader', 0x0], ['partitions', 0x8000], ['boot_app0', 0xe000], ['firmware', 0x10000]]);
  });

  it('refuses images built for another chip, non-images, missing files and an app that does not fit', () => {
    expect(checkImage('firmware.bin', image(64, CHIP_ID.ESP32))).toMatch(/built for ESP32, not ESP32-S3/);
    expect(checkImage('firmware.bin', new Uint8Array(64))).toMatch(/not an ESP image/);
    const files = goodFiles();
    files[3] = { name: 'firmware.bin', bytes: image(0x1e0001) };
    expect(planFlash(files).problems.join()).toMatch(/does not fit the app0 partition/);
    expect(planFlash(goodFiles().slice(1)).problems.join()).toMatch(/bootloader.bin is missing/);
    expect(planFlash([...goodFiles(), { name: 'notes.txt', bytes: new Uint8Array(4) }]).problems.join())
      .toMatch(/notes.txt is not one of/);
  });
});

function fakeLoader(chip = 'ESP32-S3'): Loader & { written: FlashSegment[] | null; finished: boolean } {
  const l = {
    written: null as FlashSegment[] | null,
    finished: false,
    connect: vi.fn(async () => chip),
    write: vi.fn(async (segments: readonly FlashSegment[], progress: (s: number, w: number, t: number) => void) => {
      l.written = [...segments];
      segments.forEach((s, i) => progress(i, s.bytes.length, s.bytes.length));
    }),
    finish: vi.fn(async () => { l.finished = true; }),
  };
  return l;
}

describe('flashing', () => {
  const tx = FLASH_TARGETS.find((t) => t.id === 'radio-tx')!;

  it('writes the plan to an ESP32-S3 and resets it', async () => {
    const loader = fakeLoader();
    const seen: number[] = [];
    await flash(tx, planFlash(goodFiles()), loader, (p) => seen.push(p.segment));
    expect(loader.written?.map((s) => s.offset)).toEqual([0x0, 0x8000, 0xe000, 0x10000]);
    expect(seen).toEqual([0, 1, 2, 3]);
    expect(loader.finished).toBe(true);
  });

  it('never connects for a plan with problems', async () => {
    const loader = fakeLoader();
    await expect(flash(tx, planFlash(goodFiles().slice(1)), loader)).rejects.toBeInstanceOf(FlashRefused);
    expect(loader.connect).not.toHaveBeenCalled();
  });

  it('never writes to a board that is not the expected chip', async () => {
    const loader = fakeLoader('ESP32-C3');
    await expect(flash(tx, planFlash(goodFiles()), loader)).rejects.toThrow(/says it is ESP32-C3/);
    expect(loader.write).not.toHaveBeenCalled();
    expect(loader.finished).toBe(true);                     // the port is released
  });

  it('every target names its board, chip, build and a safety note', () => {
    expect(FLASH_TARGETS.map((t) => t.id)).toEqual(['radio-tx', 'radio-rx', 'radio-diagnostic']);
    for (const t of FLASH_TARGETS) {
      expect(t.chip).toBe('ESP32-S3');
      expect(t.note.length).toBeGreaterThan(20);
    }
  });
});

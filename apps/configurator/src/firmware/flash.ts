/**
 * Choosing and flashing firmware onto ESP32-S3 boards: the Aerial Kit radio
 * (ELRS USB TX Rev A3, as an ExpressLRS transmitter or receiver, or its
 * RF-off diagnostic image).
 *
 * The images are the files the repository's own build writes
 * (`firmware/elrs-usb-tx-rev-a3/scripts/build.sh`, PlatformIO's
 * `.pio/build/<env>/`): bootloader.bin, partitions.bin, boot_app0.bin and
 * firmware.bin. Where they go is read from the partition table that comes
 * with them, not assumed; every image is checked before anything is written:
 * the ESP image magic, that it was built for an ESP32-S3, and that the app
 * fits its partition. A board that is not an ESP32-S3 is refused.
 */

export const ESP_IMAGE_MAGIC = 0xe9;
/** `esp_chip_id_t` in the image header (bytes 12-13). */
export const CHIP_ID = { ESP32: 0, ESP32_S2: 2, ESP32_C3: 5, ESP32_S3: 9 } as const;
export const BOOTLOADER_OFFSET_S3 = 0x0;
export const PARTITION_TABLE_OFFSET = 0x8000;

export type FlashTargetId = 'radio-tx' | 'radio-rx' | 'radio-diagnostic';

export interface FlashTarget {
  readonly id: FlashTargetId;
  readonly label: string;
  readonly board: string;
  readonly chip: 'ESP32-S3';
  /** The PlatformIO environment whose build output this is. */
  readonly buildEnv: string;
  readonly note: string;
}

export const FLASH_TARGETS: readonly FlashTarget[] = [
  { id: 'radio-tx', label: 'Radio transmitter (ExpressLRS TX)', board: 'Aerial Kit ELRS USB TX Rev A3', chip: 'ESP32-S3',
    buildEnv: 'AERIALKIT_ELRS_USB_TX_REV_A3_via_UART',
    note: 'Fit the antenna before the first power-up with this image: a transmitter without a load can damage its amplifier.' },
  { id: 'radio-rx', label: 'Radio receiver (ExpressLRS RX)', board: 'Aerial Kit ELRS USB TX Rev A3', chip: 'ESP32-S3',
    buildEnv: 'AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART',
    note: 'Set SW2 to match the image and check it by continuity: the first prototype read the opposite of its label.' },
  { id: 'radio-diagnostic', label: 'Board diagnostic (RF off)', board: 'Aerial Kit ELRS USB TX Rev A3', chip: 'ESP32-S3',
    buildEnv: 'aerialkit_rev_a3_diagnostic',
    note: 'Flash this first on a new board, antenna disconnected: it checks boot, USB, UART, the radio chip and the power stage with RF off.' },
];

export type ImageRole = 'bootloader' | 'partitions' | 'boot_app0' | 'firmware';

export interface ImageFile {
  readonly name: string;
  readonly bytes: Uint8Array;
}

export interface Partition {
  readonly name: string;
  readonly type: number;
  readonly subtype: number;
  readonly offset: number;
  readonly size: number;
}

export interface FlashSegment {
  readonly role: ImageRole;
  readonly name: string;
  readonly offset: number;
  readonly bytes: Uint8Array;
}

export interface FlashPlan {
  readonly segments: readonly FlashSegment[];
  readonly partitions: readonly Partition[];
  readonly problems: readonly string[];
}

/** Which image a file is, by the name the build gives it. */
export function roleOf(name: string): ImageRole | null {
  const base = name.toLowerCase().replace(/^.*[\\/]/, '');
  if (base === 'bootloader.bin') return 'bootloader';
  if (base === 'partitions.bin') return 'partitions';
  if (base === 'boot_app0.bin') return 'boot_app0';
  if (base === 'firmware.bin') return 'firmware';
  return null;
}

/** The ESP-IDF partition table: 32-byte entries starting 0xAA 0x50, little-endian. */
export function parsePartitions(bytes: Uint8Array): Partition[] {
  const out: Partition[] = [];
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  for (let i = 0; i + 32 <= bytes.length; i += 32) {
    if (bytes[i] !== 0xaa || bytes[i + 1] !== 0x50) break;
    let end = i + 12;
    while (end < i + 28 && bytes[end] !== 0) end++;
    out.push({ name: new TextDecoder().decode(bytes.subarray(i + 12, end)), type: bytes[i + 2] ?? 0,
      subtype: bytes[i + 3] ?? 0, offset: v.getUint32(i + 4, true), size: v.getUint32(i + 8, true) });
  }
  return out;
}

/** What is wrong with an ESP image for this chip, or null. */
export function checkImage(name: string, bytes: Uint8Array, chipId: number = CHIP_ID.ESP32_S3): string | null {
  if (bytes.length < 24) return `${name} is too short to be an ESP image (${bytes.length} bytes)`;
  if (bytes[0] !== ESP_IMAGE_MAGIC) return `${name} is not an ESP image (first byte 0x${(bytes[0] ?? 0).toString(16)}, expected 0xe9)`;
  const chip = (bytes[12] ?? 0) | ((bytes[13] ?? 0) << 8);
  if (chip !== chipId) {
    const known = Object.entries(CHIP_ID).find(([, id]) => id === chip)?.[0] ?? `chip id ${chip}`;
    return `${name} was built for ${known.replace('_', '-')}, not ESP32-S3`;
  }
  return null;
}

/**
 * Where each file goes, read from the partition table that came with them,
 * and every reason not to write. A plan with problems is never flashed.
 */
export function planFlash(files: readonly ImageFile[]): FlashPlan {
  const problems: string[] = [];
  const byRole = new Map<ImageRole, ImageFile>();
  for (const f of files) {
    const role = roleOf(f.name);
    if (role === null) problems.push(`${f.name} is not one of bootloader.bin, partitions.bin, boot_app0.bin, firmware.bin`);
    else if (byRole.has(role)) problems.push(`two files for ${role}`);
    else byRole.set(role, f);
  }
  for (const role of ['bootloader', 'partitions', 'firmware'] as const) {
    if (!byRole.has(role)) problems.push(`${role}.bin is missing`);
  }
  const partitions = byRole.has('partitions') ? parsePartitions(byRole.get('partitions')!.bytes) : [];
  if (byRole.has('partitions') && partitions.length === 0) problems.push('partitions.bin holds no partition table');
  const app = partitions.find((p) => p.type === 0);
  const otadata = partitions.find((p) => p.type === 1 && p.subtype === 0);
  if (partitions.length > 0 && app === undefined) problems.push('the partition table has no app partition');

  for (const role of ['bootloader', 'firmware'] as const) {
    const f = byRole.get(role);
    if (f !== undefined) {
      const problem = checkImage(f.name, f.bytes);
      if (problem !== null) problems.push(problem);
    }
  }
  const fw = byRole.get('firmware');
  if (fw !== undefined && app !== undefined && fw.bytes.length > app.size) {
    problems.push(`firmware.bin (${fw.bytes.length} bytes) does not fit the ${app.name} partition (${app.size} bytes)`);
  }
  const boot = byRole.get('bootloader');
  if (boot !== undefined && boot.bytes.length > PARTITION_TABLE_OFFSET - BOOTLOADER_OFFSET_S3) {
    problems.push('bootloader.bin would overlap the partition table');
  }
  if (byRole.has('boot_app0') && otadata === undefined && partitions.length > 0) {
    problems.push('boot_app0.bin was given but the partition table has no otadata partition for it');
  }

  const segments: FlashSegment[] = [];
  if (boot) segments.push({ role: 'bootloader', name: boot.name, offset: BOOTLOADER_OFFSET_S3, bytes: boot.bytes });
  const pt = byRole.get('partitions');
  if (pt) segments.push({ role: 'partitions', name: pt.name, offset: PARTITION_TABLE_OFFSET, bytes: pt.bytes });
  const ba = byRole.get('boot_app0');
  if (ba && otadata) segments.push({ role: 'boot_app0', name: ba.name, offset: otadata.offset, bytes: ba.bytes });
  if (fw && app) segments.push({ role: 'firmware', name: fw.name, offset: app.offset, bytes: fw.bytes });
  return { segments, partitions, problems };
}

/** What the flasher needs from a ROM loader (esptool-js in the browser, a fake in tests). */
export interface Loader {
  /** Connects, syncs with the ROM and returns the chip's name, e.g. "ESP32-S3". */
  connect(): Promise<string>;
  write(segments: readonly FlashSegment[], progress: (segment: number, written: number, total: number) => void): Promise<void>;
  /** Hard-resets into the new firmware and releases the port. */
  finish(): Promise<void>;
}

export class FlashRefused extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'FlashRefused';
  }
}

export interface FlashProgress {
  readonly segment: number;
  readonly name: string;
  readonly fraction: number;
}

/**
 * Flash a plan. Refuses before connecting if the plan has problems, and
 * before writing if the board is not the chip the target expects.
 */
export async function flash(target: FlashTarget, plan: FlashPlan, loader: Loader,
  onProgress: (p: FlashProgress) => void = () => {}): Promise<void> {
  if (plan.problems.length > 0) throw new FlashRefused(`not flashing: ${plan.problems.join('; ')}`);
  const chip = await loader.connect();
  if (!chip.toUpperCase().startsWith(target.chip)) {
    await loader.finish();
    throw new FlashRefused(`the board says it is ${chip}; ${target.label} is built for ${target.chip}`);
  }
  await loader.write(plan.segments, (segment, written, total) => onProgress({
    segment, name: plan.segments[segment]?.name ?? '', fraction: total > 0 ? written / total : 0 }));
  await loader.finish();
}

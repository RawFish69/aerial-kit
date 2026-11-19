/**
 * ExpressLRS binding identity helpers.
 *
 * A binding phrase is never stored on the module. ExpressLRS hashes
 * `-DMY_BINDING_PHRASE="<phrase>"` with MD5 and keeps the first six bytes as
 * the UID (`src/lib/CONFIG/config.cpp`, `BindphraseConfigurable::SetBindPhrase`).
 * An empty phrase is the unbound all-zero UID.
 */

const S = [
  7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
  5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
  4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
  6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
] as const;

const K = Array.from({ length: 64 }, (_, i) =>
  Math.floor(Math.abs(Math.sin(i + 1)) * 2 ** 32) >>> 0,
);

function rotateLeft(value: number, bits: number): number {
  return ((value << bits) | (value >>> (32 - bits))) >>> 0;
}

export function md5(bytes: Uint8Array): Uint8Array {
  const bitLength = BigInt(bytes.length) * 8n;
  const total = Math.ceil((bytes.length + 9) / 64) * 64;
  const input = new Uint8Array(total);
  input.set(bytes);
  input[bytes.length] = 0x80;
  for (let i = 0; i < 8; i++) {
    input[total - 8 + i] = Number((bitLength >> BigInt(8 * i)) & 0xffn);
  }

  let h0 = 0x67452301;
  let h1 = 0xefcdab89;
  let h2 = 0x98badcfe;
  let h3 = 0x10325476;
  const words = new Uint32Array(16);
  const view = new DataView(input.buffer);

  for (let offset = 0; offset < input.length; offset += 64) {
    for (let i = 0; i < 16; i++) words[i] = view.getUint32(offset + i * 4, true);
    let a = h0;
    let b = h1;
    let c = h2;
    let d = h3;
    for (let i = 0; i < 64; i++) {
      let f: number;
      let g: number;
      if (i < 16) {
        f = (b & c) | (~b & d);
        g = i;
      } else if (i < 32) {
        f = (d & b) | (~d & c);
        g = (5 * i + 1) % 16;
      } else if (i < 48) {
        f = b ^ c ^ d;
        g = (3 * i + 5) % 16;
      } else {
        f = c ^ (b | ~d);
        g = (7 * i) % 16;
      }
      const next = (a + f + K[i]! + words[g]!) >>> 0;
      const rotated = rotateLeft(next, S[i]!);
      a = d;
      d = c;
      c = b;
      b = (b + rotated) >>> 0;
    }
    h0 = (h0 + a) >>> 0;
    h1 = (h1 + b) >>> 0;
    h2 = (h2 + c) >>> 0;
    h3 = (h3 + d) >>> 0;
  }

  const out = new Uint8Array(16);
  const outView = new DataView(out.buffer);
  [h0, h1, h2, h3].forEach((word, index) => outView.setUint32(index * 4, word, true));
  return out;
}

export function bindingUid(phrase: string): Uint8Array {
  if (phrase.length === 0) return new Uint8Array(6);
  const encoder = new TextEncoder();
  const input = new Uint8Array(encoder.encode('-DMY_BINDING_PHRASE="').length +
    encoder.encode(phrase).length + 1);
  const prefix = encoder.encode('-DMY_BINDING_PHRASE="');
  const phraseBytes = encoder.encode(phrase);
  input.set(prefix);
  input.set(phraseBytes, prefix.length);
  input[input.length - 1] = 0x22; // closing "
  return md5(input).slice(0, 6);
}

export function uidText(uid: Uint8Array): string {
  return Array.from(uid, (byte) => byte.toString(16).padStart(2, '0')).join(':');
}

export function parseUid(text: string): Uint8Array {
  const compact = text.replace(/[\s:]/g, '');
  if (!/^[0-9a-fA-F]{12}$/.test(compact)) {
    throw new Error('UID must contain exactly six hexadecimal bytes');
  }
  return Uint8Array.from(
    { length: 6 },
    (_, index) => parseInt(compact.slice(index * 2, index * 2 + 2), 16),
  );
}

export function isUnboundUid(uid: Uint8Array): boolean {
  return uid.length === 6 && uid.every((byte) => byte === 0);
}

import { describe, expect, it } from 'vitest';
import {
  bindingUid,
  isUnboundUid,
  parseUid,
  uidText,
} from '../src/protocol/elrs-binding';

describe('ExpressLRS binding identity', () => {
  it('derives the same six-byte UID as SetBindPhrase', () => {
    expect(uidText(bindingUid('test'))).toBe('4f:04:fd:82:21:55');
    expect(uidText(bindingUid('my phrase'))).toBe('3b:93:3b:37:b1:e4');
  });

  it('treats an empty phrase as the unbound all-zero UID', () => {
    expect(bindingUid('')).toEqual(new Uint8Array(6));
    expect(isUnboundUid(bindingUid(''))).toBe(true);
  });

  it('parses direct UID text and rejects malformed values', () => {
    const uid = parseUid('12:34:56:78:9A:BC');
    expect(uidText(uid)).toBe('12:34:56:78:9a:bc');
    expect(() => parseUid('12:34')).toThrow(/six hexadecimal/);
  });
});

# Rev A3 LED state machine — 2026-11-19

The dual-role applications no longer use the upstream rainbow startup or
packet-rate hue animation. Role is encoded only by color and state by pattern.

| State | TX | RX |
| --- | --- | --- |
| Boot identity | One 350 ms blue pulse | Two 350 ms green pulses |
| Bind | Blue breathing, 3 s cycle | Green breathing, 3 s cycle |
| Searching / no RF peer | Blue breathing, 1 s cycle | Green breathing, 1 s cycle |
| Host/FC present, RF disconnected | Two 100 ms blue pulses/s | Two 100 ms green pulses/s |
| RF connected | Steady blue | Steady green |
| No sufficient serial input | Blue pulse every 2 s | Green pulse every 2 s |
| Radio/SX1280 fault | Fast red, 100 ms on/off | Fast red, 100 ms on/off |
| Role or model mismatch | Three amber pulses, 1 s pause | Three amber pulses, 1 s pause |

The operational state priority is:

1. role/model mismatch;
2. radio/SX1280 fault;
3. boot identity;
4. binding mode;
5. RF connection or negotiation, including a connected link or a present host
   with RF still disconnected;
6. searching before a host has been seen;
7. no serial input after a host was previously seen.

Bind indication remains active for at least three seconds after the first bind
state is observed, so a short command still produces one complete breathing
cycle. Breathing updates every 20 ms and fades the one role channel smoothly
from 51/255 to 255/255 before the existing global 26/255 cap, which is
approximately 2% to 10% final LED brightness.

Native USB presence is read from `HWCDC::isConnected()`. Valid CRSF input from
the native USB or UART host/FC also marks the serial peer present. An external
UART peer therefore needs to send at least one valid CRSF frame before the
firmware can distinguish it from an absent peer.

## Verification

- TX and RX ExpressLRS applications build with the state machine.
- Native tests cover state priority, role colors, breathing range and periods,
  pulse timing, red radio fault, amber mismatch and serial-pointer safety.
- The QIO bootloader, role selector, native USB bridge, LittleFS image,
  partition layout and FEM safety paths are unchanged.

## Factory artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 5fbc25c9d23b32b33859a7ae02fce5f69aee718cb60c217850d241fc45317838
flash offset: 0x0
```

Hardware verification must still confirm the observed colors, brightness and
timing on the assembled WS2812.

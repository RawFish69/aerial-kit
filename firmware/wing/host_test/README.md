# host_test

A host test for the wing mixer's arithmetic. Run it with:

```bash
firmware/wing/host_test/run.sh
```

It compiles `src/mix_math.h` — the header `src/mixer.cpp` itself includes — and
checks the shape of the mix: which way each surface moves, that the two elevons
are mirror images in roll, and that nothing escapes its limits. It does not check
the tuning; the gains are arguments, and the assertions hold for any positive
gain.

## Why this is not `pio test`

PlatformIO's unit testing runs on a `native` platform environment, and this
project does not have one: it has `espressif32` and `ststm32` and nothing else,
so `pio test` here would need a platform downloaded first. A host test that only
runs when a network is available is a test that quietly stops running, so this
one is a `g++` invocation and a shell script instead — no board, no PlatformIO,
no network — and it deliberately does not sit in `test/`, where the PlatformIO
convention would imply `pio test` picks it up.

## Where the arithmetic lives, and why

The mix math sits in `src/mix_math.h` rather than inline in `src/mixer.cpp` for
one reason: so this test compiles the same code the firmware flashes. Copy the
formulas into the test instead and it would be checking a restatement of the
arithmetic, which passes when the arithmetic changes — the failure mode it exists
to catch.

`src/mixer.cpp` keeps everything that needs the board: the profile's pins, the
PWM calls, and which of the two thrust paths is compiled.

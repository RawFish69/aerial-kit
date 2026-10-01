> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - the flight pack

How much battery is left, from two resistors and an ADC pin:

```text
pack +  ---[ 10k ]---+--- PC0            ADC1 input 10
                     |
                    [ 1k ]
                     |
                    GND

counts -> volts at the pin -> pack volts -> cells -> volts a cell -> a level
         ak_battery_pin_volts()  x divider  ceil()   / cells      thresholds
```

The board measures volts at its own pin; everything to the right of the pin is
`src/core/flight/ak_battery.c`, which is portable, host-tested and identical on
both targets. That split is the same one the sensors use, and it is what makes
a second board with a different divider a pin and two resistors rather than a
second copy of the arithmetic.

## Wiring

| | |
| --- | --- |
| Pin | PC0, ADC1 input 10 |
| Divider | 10k from the pack's positive terminal to PC0, 1k from PC0 to ground, ratio **11.0** |
| Reference | 3.3 V nominal (the analog supply), 4095 counts full scale |
| Sample time | 480 ADC cycles - the longest the part has, chosen for a source impedance near a kilo-ohm |
| ADC rate | APB2 / 4 = 21 MHz, inside the part's 36 MHz limit |

PC0 is the pin because it is the first one left. It is not a timer channel the
outputs need, not a UART, not the SPI the sensors are on, and not the LED. The
divider puts even a 6S at 2.3 volts, comfortably under the 3.3 volt reference,
and the bottom resistor is also what makes an unplugged pack read zero rather
than whatever the pin was holding.

**Nothing is fitted on the bench board.** Until the two resistors are soldered
in, PC0 floats and whatever `battery` prints is a number about a pad. That is
the first item on the bench list, and it is the reason this page exists rather
than a line in [07-outputs.md](07-outputs.md).

It is written down where the code can see it, too: `AK_BOARD_VBAT_FITTED` in
`src/boards/AERIALKIT_F405/board.h` is 0. Until it is 1 the ADC still converts,
`battery` still prints the counts, and the pack arithmetic is switched off -
because a floating pin can read a perfectly plausible 1.5 volts, and the
firmware would otherwise multiply that by eleven and report a healthy 4S. The
only step when the resistors go in is that one line.

### And the same thing on the ESP32, where the converter works differently

The arithmetic above is the core's and is identical on both chips; what is not
identical is what a converter hands back. The F405's ADC returns counts and the
core converts them with a reference and a full scale it was told. The ESP32's
calibration is *line fitting* - the original part measures two points into
eFuse and fits a line through them, where the newer ones carry a curve - so the
only honest conversion is the one IDF does, and what comes out of
`src/arch/esp32/adc.c` is already millivolts. The board turns that into volts
at its pin and the core does the rest, which is the port working as intended:
the core's contract is "volts at your pin", and how a chip produces that is the
chip's business.

The pin is **GPIO34, ADC1 channel 6** - input-only, with a converter on it, and
the one left over: not a motor or a servo, not a UART, not the SPI or the I2C
pair, not the LED and not the console. The divider is the same 10k/1k the F405
is drawn with (11 volts of pack per volt at the pin), read at 12 dB
attenuation, which is the widest range this part has and about 3.1 volts of
span - a 6S lands at 2.3 volts, comfortably inside it. Nothing is fitted on a
bare devkit, `AK_BOARD_VBAT_FITTED` is 0, and the console says so; the emulated
ESP32 checks that it says so rather than inventing a voltage
(`tools/esp32_proto_check.py`).

What is *not* verified here is the converter itself: QEMU does not model this
peripheral, so no reading has ever been taken on this chip - and the
calibration's own numbers are the eFuse's business on a real part.

## The arithmetic, and the four ways it can be quietly wrong

**Counts to volts.** `counts * vref / full_scale`. 1252 counts of 4095 at 3.3
volts is 1.0086 volts. Nothing interesting, and it is a checked function rather
than three lines inside a driver, because it is the one part of the chain with
an exact answer.

**Volts at the pin to pack volts.** Multiply by the divider's ratio, which is
`vbat_ratio` and defaults to 11.0. This is the number to correct with a
multimeter, and **the firmware does the arithmetic**: put a meter across the
pack, type what it says, and the command reads the pin and works out the ratio.

```text
ak> calibrate vbat 12.60
calibrate vbat: 12.60 V at the pack over 1.091 V at the pin is a ratio of 11.550
                'save' keeps it, and the pack now reads 12.60 V
```

It refuses while armed, on a board with no way to measure a pack, and when the
pin reads almost nothing - a divider with no pack on it, which would otherwise
produce a ratio made of noise. The number it writes goes in through the
parameter table, so the range check, the "changed since the last save" count and
`save` all behave the way they do for anything else.

**Pack volts to a cell count.** Divide by what one cell can hold, and take the
ceiling, because a pack is one cell more than the whole number of cells that
fit. The dividing voltage is 4.30, not the 4.20 a cell is nominally full to,
and that is the trap this file is mostly about: a 3S that came off a charger
hot reads 12.65 volts, which over 4.20 is 3.01 - a ceiling that says *four*
cells, and four cells at 3.16 volts a cell is a pack that is fine. It is not
fine; it is a full 3S. Against 4.30 the answer is three, with a tenth of a volt
a cell of headroom on every pack a person actually uses.

**The count is a high-water mark.** Once a pack is counted, it does not shrink
while it stays plugged in. A 3S that sags to 8.4 volts under load is 1.95 packs
of 4.30, which a fresh count would call two cells - and two cells at 4.2 volts
is healthy, on the last minute of a flight. Latched at three it is 2.8 volts a
cell, which is what it is. Unplugging the pack is the only thing that clears
the count.

## Levels, and where the numbers come from

| Setting | AerialKit | Betaflight 2026.6.1 | INAV 9.1.0 |
| --- | --- | --- | --- |
| warn a cell | 3.50 V | 3.50 V | 3.50 V |
| critical a cell | 3.30 V | 3.30 V | 3.30 V |
| cell detect | 4.30 V | 4.30 V (the max cell voltage) | 4.25 V |
| max believable cells | 8 | 8 (`MAX_AUTO_DETECT_CELL_COUNT`) | - |

All four are parameters (`vbat_warn_cell`, `vbat_min_cell`, `vbat_ratio`,
`vbat_cells`), and `vbat_cells` at 0 - the default - counts them from the
voltage. A number overrides the count for a pack the arithmetic gets wrong,
because the pilot is the one holding it.

Exactly *on* a threshold is not past it: 3.50 volts a cell is `ok` and 3.49 is
`low`, because the firmware warns *under* a threshold - and the console says
"under" rather than "below 3.300" so that the words and the code agree. That is
a choice, it is written down here, and the tests pin it.

## The two readings that are not a battery

**Zero is not empty.** An ADC that was never configured reads zero, a pin with
no divider on it reads zero, and a disconnected pack reads zero. Zero volts a
cell is also the most alarming number the firmware could report, and reporting
it would be a lie with a red flag on it. So a reading below 2 volts of pack is
*nothing connected*, not a flat battery - and it takes three readings in a row
to say so, because one reading of nothing is a wire that moved. The cell count
is not cleared by a single bad reading for the same reason.

**A negative reading is not a voltage.** A board whose ADC does not answer
returns a negative pin voltage, which the core treats as "no reading" and
leaves the last good estimate alone. A sensor that stops answering is not a
battery that emptied itself.

**Above 34.4 volts (8 cells at 4.30) is not a pack** this firmware knows about,
so the reading is counted and dropped rather than turned into nine confident
cells. A pin stuck at the 3.3 volt rail - which is what a missing divider
sometimes does - reads as 36.3 volts of pack and lands here.

## What is there, and what is not

There is a filtered pack voltage at ten readings a second and a level on it,
which is what `battery` and the preflight report print. There is no *current*
sensor, no consumed-mAh counter and no battery model - none of which can be
written honestly without a sensor to check them against.

The path is 64 host checks on the F405 - 52 on the arithmetic and 12 on the
converter, its register bit positions and its conversion - plus the same
converter checks on the wing's part and the board's own reading of a fitted
divider, and two in the loop, where the simulator's pack sags and the console's
own words are read back.

There is also **no arming gate on the battery**, deliberately. A low pack does
not stop the aircraft arming, and it does not trigger anything. The reason is
the order this has to be done in: when this was written the divider was not
fitted, so the first honest reading has to come from a bench session that has
not happened yet, and a gate
written before that reading would be a gate that either never fires or fires on
a number nobody has checked. The level is reported; when it has been seen
against a real pack, refusing to arm on it is a small change to the flight
core and a test.

And nothing here has seen a real cell. The board's divider *is* fitted (the
owner soldered it; `docs/05-bringup.md` §8 is the two lines that say the
firmware has been told), but no pack has been measured and the reference
voltage is a nominal 3.3 rather than anything measured. The arithmetic is
checked on the host, the register bit positions are checked against RM0090 in
`tests/test_regs.c`, the chain runs end to end in the simulator - and, since
2026-09-18, so does the converter's *success* path: `ak_adc_read_counts()`
clears a status register, starts a conversion and waits for the end flag, and
the third of those is a peripheral action that a page of memory cannot produce,
so until the converter had a model of that one flag (`tests/host_adc_model.c`)
every host test could only ever reach its timeout. The meter is still the thing
that decides whether `vbat_ratio` is 11.0.

## On the console

```text
> battery
battery:   3S, 12.00 V pack, 4.00 V a cell - ok
divider:   11.000 pack volts per volt at the pin (vbat_ratio)
thresholds: low under 3.500 V a cell, critical under 3.300 V
readings:  78 taken, 0 rejected as implausible
adc:       PC0, ADC1 input 10; 10k over 1k against 3.30 V, 4095 counts full scale
raw:       1252 counts = 1.008 V at the pin
```

`preflight` carries the same level as a fact rather than a fault: the count it
prints is the number of things that mean the firmware is wrong about *itself*,
and a flat pack is not one of those.

## What a critical pack does about it

For a long time this page ended one step short: the pack state was measured,
filtered, counted, printed and sent to the handset, and **nothing acted on
it**. A quadrotor whose cells fell below the critical threshold went on flying
until they could not hold it up - measured in the simulator, with the pack at
3.25 V a cell and a pilot flying it, the aircraft climbed away to 360 m and
633 m from home with the motors still turning, which is the flight before the
one that ends in a field.

So there is a return for it, and it is `battery_rth`: **0 by default**, for the
same reason the fence is off by default - it takes an aircraft off a pilot
whose link is up, and a firmware that did that the first time a cell dipped is
a firmware nobody would fly. With it on, a pack that reads critical while the
aircraft is flying and has a position fix engages the navigator, which brings
the aircraft home and lands it (a quadrotor; a wing circles, because it cannot
do anything else). The console says why:

```text
battery: 3.29 V a cell - bringing it home
```

Three details are deliberate:

- **It latches.** A pack sags under load and comes back when the load comes
  off, so a return that followed the voltage back up would hand the aircraft to
  a pilot who is watching a flat battery. Once it has started it runs until the
  flight ends - the aircraft landing, or the arm switch going off - and the
  in-the-loop session sags a pack, recovers it to 12.00 V mid-return and checks
  that the aircraft does not change its mind.
- **The sticks do not cancel it**, which is deliberately *not* the rule a
  mission uses. The pilot here is already holding a stick: the pack went flat
  while they were flying it, so "the sticks are off centre" describes the
  flight they were already making rather than an instruction to take it back.
  The override is the switch that means it: disarm.
- **It needs a fix to start.** There is nothing to come home to without a
  position, so a flat pack with no GPS is the case in
  [14-navigation.md](14-navigation.md)'s "losing the fix" section: hold what
  the aircraft can still measure.

What is measured, in `aerialkit-fw-sim 110 battery`
([evidence/sil-battery.txt](evidence/sil-battery.txt)): the link stays up and
the pilot keeps flying; the pack sags to 3.25 V a cell at 15 s; the navigator
engages and the console says so; the pack recovers to 12.00 V at 25 s; the
aircraft comes home anyway and stops its motors 2 m from home at 80.5 s,
staying down. What is *not* claimed: any of it on an airframe, and the numbers
are one 3S pack in a simulator - the thresholds are the same three the
reference implementations use, and they are parameters.

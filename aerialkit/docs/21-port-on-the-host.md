# AerialKit - running the port itself on a development machine

Everything else in this repository tests the core or a driver. Until this
existed, `src/arch/stm32f405/` - the clock, the pins, the UARTs, the tick - had
never been *executed* by anything. The simulator brings its own console, its
own clock and its own UARTs; the image check reads the vector table; and the
register encodings are checked as constants against the manual. Constants
against a manual is not the same as code that runs.

`tests/test_arch.c` maps the peripheral region at the addresses the code
expects and then calls the real functions against it:

```text
mmap(0x40000000, 0x40000)     RCC, FLASH, GPIO, USART, timers, DMA, SPI, I2C, ADC
mmap(0xE000E000, 0x1000)      SysTick, the system control block

ak_clk_init()        -> read back RCC_PLLCFGR, RCC_CFGR, FLASH_ACR
ak_uart_init()       -> read back BRR, CR1, CR2, MODER, AFR, PUPDR, RCC
ak_console_attach()  -> ak_console_write() -> USART_DR
USART1_IRQHandler()  -> the ring the receiver's bytes arrive in
ak_arch_time_init()  -> SYSTICK_LOAD, and five ticks
ak_output_init()     -> the servo bank it was handed (TIM2 or TIM4), TIM3, the
                        burst registers, and DMA1 stream 4
ak_output_write()    -> the compare registers, the DMA enable, a second frame
DMA1_Stream4_IRQHandler() -> the flags, the enable bit, the frame count
```

The registers the code writes are the registers the test reads, so this checks
*behaviour* rather than transcription: the clock tree's arithmetic decoded from
the bits it left behind, the divisor for every port the firmware drives, the
pins it claimed, the byte that reaches the data register, the interrupt that
fills the ring, the reload that makes a millisecond.

## What it found on the first run

**The UART divisor was sixteen times too large, on every port.**

| | written | wanted | what it would have run at |
| --- | --- | --- | --- |
| console, 115200 at 42 MHz | 5833 | 365 | 7200 baud |
| GPS, 9600 at 42 MHz | 70000 | 4375 | 600 baud |
| receiver, 420000 at 84 MHz | 3200 | 200 | 26250 baud |

RM0090 30.6.4: with oversampling by 16, `baud = fCK / (16 x USARTDIV)`, and BRR
carries USARTDIV shifted left four - so `BRR = USARTDIV x 16 = fCK / baud`. The
code multiplied the *numerator* by sixteen instead, which is USARTDIV times 256.

On the first flash that is a console printing nothing legible, a receiver that
never frames a packet and a GPS that never reports a fix - three symptoms that
look like three problems, none of which is the problem, and all of which are
installations and wiring before anybody suspects arithmetic that looks
plausible. It is the single most likely way this firmware's first bench session
would have gone badly, and nothing in the repository could see it.

Restoring the old formula makes four of these checks fail. That is the
difference between a test and a decoration, and it is why the numbers in the
table are in the test as literals worked out from the manual rather than as the
code's own expression rearranged.

**And the console's write waited forever.** `ak_console_write_raw` polled
transmit-empty with no bound. A port whose clock is off - or whose pins were
claimed by something else - never sets that flag, so the firmware would have
stopped *inside the first line of its own banner*: no LED, no fault record, no
console, a board that looks dead rather than one that says what is wrong. It
gives up now and drops the rest of the line, which is both easier to diagnose
and truer: the port really did not accept it.

## The output path, which turns motors

The same treatment was then given to `output.c`, which is the configuration
with most ways of being wrong in the whole port: two timers, six pins, a DMA
burst that writes four compare registers at once, and an interrupt that closes
each frame.

| Checked, by running it | Why it is the check |
| --- | --- |
| 150, 300 and 600 kHz are 560, 280 and 140 timer ticks, so ARR is 559, 279 and 139 | one timer period is one DShot bit: a period that is wrong is a bit rate an ESC cannot sample |
| a rate the ESCs do not speak is refused rather than approximated | 250 kHz is a number somebody will type |
| the burst starts at word 13 of the timer (CCR1 at 0x34) and moves four half-words | DBA counts 32-bit words from the timer's base and DBL is the length *minus one*: either one wrong writes four values somewhere that is not the compare registers |
| the stream is memory-to-peripheral, incrementing, half-words, on channel 5, at high priority, with the transfer-complete interrupt | channel selection lives in the top byte of the control register, and a stream on the wrong channel never requests |
| the burst runs on timer 3 channel 1's compare event, and the interrupt is enabled in the controller | the two halves of "start a frame", in two different peripherals |
| the servo timer counts microseconds and wraps at 20000 | 50 Hz, and the pulse width is the compare value |
| the first bit of each motor's frame is in its compare register, and the four differ | a burst that wrote one value four times would pass everything else |
| a frame arriving while the burst is running is dropped and counted | the compare registers are the ESC's clock; a late frame must not be interleaved into one being sent |
| the interrupt clears the stream's flags, stops it, counts the frame, and leaves it ready | the flag register is split in halves and stream 4 is in the high one |

That last table column is the point of the whole exercise. None of these were
wrong this time - which is worth saying plainly, because a verification that
only ever finds bugs is a verification nobody trusts when it finds none. The
output path is now *executed* rather than transcribed, and `docs/07-outputs.md`
says what it still is not: no waveform has been seen, and one scope on the
motor pads is still the only thing that can say whether an ESC agrees.

## Where the register-block trick stops

**An attempt at the model, and what it cost (2026-09-16).** A first pass at the
flash controller's model was written and withdrawn. The seam in `flash.c`
worked - the driver's unlock sequence, its programming loop and its flag checks
all reached the model, and the guard checks moved onto it cleanly. What did not
work is the erase: the sector the record lives in was never actually emptied,
and the save still read back correctly, because programming rewrites the whole
record anyway. A byte-level check on the modelled flash is what caught it. The
model's counter said it had erased, the flash said otherwise, and the flash was
right.

The second attempt found the reason, and it is a one-line kind of reason. The
part's sectors are **not all the same size** - four of 16 KB, one of 64 KB and
seven of 128 KB (RM0090 table 3) - and a model that assumes one size erases
somewhere else entirely while reporting success. Nothing in the driver can
notice, because the driver hands over a *sector number* and the part is what
decodes it into an address. Rebuilt with that same wrong assumption, the tests
described below fail instead of passing; that is written up at the end of this
section, because a test that cannot fail is decoration.

The sensor bus and the flight pack's ADC are on the other side of that line -
they are pure register configuration, so both execute:

| Checked, by running it | Why it is the check |
| --- | --- |
| SPI2's control register is exactly `0x0357`: master, mode 3, software slave select, APB1/8, enabled last | a bit added or dropped later is a different bus, and none of the sensors answer on a bus that is not the one they want |
| SCK, MISO and MOSI on alternate function 5, MISO pulled up and the driving pins not | an idle MISO floats, and a floating MISO is a bus that reads noise |
| a transfer on a bus whose flags never move gives up and says so | there is no sensor on the bench board; the failure that matters is the firmware waiting for one |
| the ADC's prescaler is four, its sequence is one long and holds the battery's channel, and end-of-conversion is per conversion | the prescaler is in the *common* block rather than in ADC1, which is the sort of thing whose symptom is a conversion that never completes |
| channel ten's sample time is in SMPR1 and channel three's in SMPR2, each in the right place | the two registers split at channel ten, and a driver that wrote the wrong one would leave the part sampling with the reset value - the shortest time it has, against a divider far too weak for it |
| a conversion that never finishes is reported rather than read | the data register holds the *last* conversion, and a plausible stale number in a control loop is worse than an error |

The flash controller was the one file this could not cover, because its success
path needs the *part* to act on what was written: the keys in FLASH_KEYR have to
clear the lock bit in FLASH_CR, and a start bit has to erase a sector before a
word can land in it. A plain region stores what is written to it and does
nothing else. It is covered now, by a model behind a seam, and that is the next
section.

## The flash controller, modelled

`flash.c` routes three things through a seam - a register read, a register
write, and the write of a data word into the flash itself - and with
`-DAK_HOST_FLASH` set for that one object they go to `tests/host_flash_model.c`
instead of to registers. On the target the three are plain loads and stores,
and **the image is byte-for-byte what it was before the seam existed**: built
from the same revision with the same stamp, the tree with the model and the
tree without it produce identical `.bin` files. That was not free - the data
write started as a function, which cost eight bytes by moving the programming
loop around, and it is a macro now for the same reason the comparison is
written down: the one file a board has to check should not be rewritten by a
change that only exists to test it.

The model is the part's *behaviour*, not its storage: the key sequence and the
lock bit it clears, the status flags and their clear-by-writing-one, the two
control bits that start an operation, and the sector geometry above. The flash
itself is the region the test maps at `0x08000000`, and the model erases and
programs into that memory - so what the driver reads back is what is really
there, and a test can read it byte for byte afterwards rather than ask the model
how it did.

| Checked, by running it | Why it is the check |
| --- | --- |
| a part that has never been written is all ones, at both ends of the region | the state the configuration record's magic depends on; if the model handed back zeros, "nothing saved" and "saved as zeros" would be the same answer |
| a sector erases, and a program puts words where they were addressed | the two operations the whole file exists for, run end to end |
| programming over a word that is already programmed is refused, and the word that was there is left alone | this is what a save that never erased looks like: the part refuses it rather than corrupting the sector, and the driver reports the refusal |
| a 16 KB sector erases to its own end and leaves its 64 KB neighbour alone | the geometry, which is the thing the withdrawn model got wrong and the one thing the driver cannot check for itself |
| the configuration record over that pair: empty part, save, read back, and the rest of the sector *still erased* | the tail is what an erase that never happened leaves behind. This is the check that caught the first attempt |
| saving again into a sector that already holds a record | every word of the old record is programmed, so the part would refuse all of them: the only way this succeeds is if the save erased first |
| a controller whose operation never finishes is given up on, left locked, with the erase bit disarmed and the record that was there still there | the wait for the busy flag is a loop with a bound in it (`AK_FLASH_TIMEOUT`), and this is the only check that reaches the bound. On a board this is the failure with no symptom: the firmware waits for an operation that will never complete, and an aircraft that stops saying anything is what a hang at boot looks like |

And the evidence that those checks can fail, because that is the only thing that
makes them evidence. Run against a model whose erase is a no-op, three of them
fail by name - *erasing sector 3 empties it*, *a save erases the sector it
writes into and reads back*, and *saving again over a sector that is not erased
still works*. Run against a model whose sectors are all 128 KB, the erase lands
off the end of the mapped part and the test dies where it stands. Both were run
here, and both are what the withdrawn attempt produced.

**The last row is the one the model did not use to be able to produce.** Every
other refusal in the file is the part saying *no* - it will not take the keys, it
will not program over a word, the address is not aligned - and the model could
say all of them. A part that accepts an operation and never finishes it is a
third thing: it says nothing at all, and a driver built on "wait for the flag"
waits for ever. `host_flash_model_set_stuck_busy()` is that part (the F405's
model and the AT32's both have it now, `OBF` for the second), and the checks are
about what the *failure* leaves behind as much as about the timeout: the
controller locked rather than left open, the erase and program bits disarmed
rather than armed for whoever comes next, and the record that was in the sector
still readable. The difference between "the save failed" and "the save failed
and your settings are gone" is a sentence a person at a bench cares about, and
it is the erase-then-program order - not the timeout - that decides it.

What is left untested is a real cell's timing: how long an erase takes, whether
the part reports busy while it happens, and what the flags say when a supply
sags mid-operation. The model does its work at once and never sets `BSY`, and
that is a deliberate simplification rather than an oversight - the wait loop is
the driver's and it is not what this is here for.

### What the model then bought

The same file now drives the blackbox's own region end to end, because the
board hands the log a table of five sectors and three adapters and everything
underneath them is the driver the model is standing in for: 2184 records - the
exact number a 128 KB sector holds after its header - fill one sector, the log
stops when the next sector turns out to hold somebody else's bytes, the erase
on the ground clears exactly that sector and no other, and the dump counts both
sectors' records afterwards. A blackbox that quietly wrote into the sector its
own configuration lives in would be found by that test, and by nothing else in
this repository.

## What it is not

It is not a chip, and the list of what it cannot see is longer than what it
can: no analogue behaviour, no clock that takes time to settle, no interrupts
actually raised (the handlers are called directly), no bus contention, no
electrical faults, no silicon erratum. The byte stream out of a UART is checked
at its last byte, because a memory-mapped register has no history - the whole
stream is what the simulator's console capture and the ESP32 evidence check, on
their own terms.

Neither is it real hardware. `docs/05-bringup.md` is still the checklist, and
nothing here removes a single item from it. It moves the *arithmetic and the
bit positions* out of the "hope" column, which is where they were.

## The barometer's bus, and where the list now ends

I2C was the last port file outside the harness. Its timing arithmetic was
already pinned in `tests/test_i2c.c`; what runs now is the register side and
the failure side, neither of which needs a trap because neither needs the part
to act on anything - a bus nobody has plugged into simply never raises a start,
and that is what the checks describe: the three timing registers, the pins on
the alternate function **and open drain**, a read and a write on a dead bus
being errors rather than values with the peripheral left ready to try again,
and a base address that is not one of the three controllers being ignored.

That last one is the only place a base address is validated, and the read and
write functions do not, which is written in the test where the next person will
read it rather than find it. Every caller in this firmware passes a board
constant, so it is a note rather than a bug.

Every file in `src/arch/stm32f405/` is now at least partly executed by the host
build, and the two that needed the device to *act* on a write have both been
closed the same way: a seam in the driver and a model behind it. The flash
controller's is above; the sensor bus is next.

## The sensor bus, running transactions

I2C was the last row that said "only a board can confirm this", and what closed
it was the same shape as the flash: `i2c.c` routes every register access through
two functions, and `tests/host_i2c_model.c` answers them - a modelled master
with the flags the driver waits on, and slave devices with a register file and a
pointer that walks.

That is worth more here than the register arithmetic was, because the thing
that had never run is a *sequence* rather than a number:

| Checked, by running it | Why it is the check |
| --- | --- |
| a one-byte read returns the register it named, with the NACK and the STOP going out before the byte is read | this is the shortest of the three read lengths and the one where the acknowledge has to be withdrawn early, or the slave starts a second byte nobody wants |
| a two-byte read returns both bytes in order | the POS case: the acknowledge moves one byte later, and BTF - not RXNE - is what says both are in |
| a longer read walks the register pointer and stops after the last byte | the acknowledge is held for all but the last two bytes and then withdrawn, which is where an off-by-one gives a reading one byte short |
| a write reaches the device's register, ending on BTF rather than TXE | TXE only says the data register is free; the STOP that followed it would cut the last byte off the wire |
| an address nobody answers is an error, and the bus still works afterwards | this is every boot on a board with no barometer fitted, and the flag the driver has to clear by hand is what makes the *next* transaction work |

Two things the model had to be taught, both of which are the kind of thing a
first draft gets wrong in a way that *looks* like a driver bug: the part clears
the START and STOP bits itself once they have gone out - leaving them set made
the driver's next control write look like another start, and the transaction was
tossed before its byte was read - and a slave's register pointer walks as the
master reads it, which is what makes "four bytes from here" one transaction on
every sensor that has an auto-incrementing address.

What that still does not settle: timing. Bits do not take time in this model, a
slave that stretches the clock is not modelled, and nothing here can say whether
the CCR and TRISE values produce 400 kHz on a real wire. That is the scope's,
and `docs/05-bringup.md` still has the checklist for it.

### And the bus that stops answering

The transactions above are all *successful* ones, apart from an address nobody
acknowledges - which the part itself reports with a flag. The other half of the
promise in the port's opening comment - "every wait has a bound, and a bound
that is reached resets the peripheral rather than returning half a reading" -
had never been reached on this machine at all: the modelled bus always
answered, so every `reset_keeping_settings()` and `transfer_abort()` was
compiled and never run. That is the code that decides whether a barometer that
dies takes the aircraft with it.

The model can now stop answering after a chosen number of status *changes*, and
both ports' tests walk that number up - zero is a bus that never comes up, and
each step lets one more wait succeed before the bus dies, so the start, the
address phase, the byte a read waits for and the stop are each asked in turn.
At every point: an error rather than a hang, and a bus that works again when it
answers. The section in [20-i2c.md](20-i2c.md) has the detail, including the two
things writing it taught - the unit a stall should be counted in (status
changes, not reads: a wait that times out polls the register two hundred
thousand times, and counting those spent the whole budget on one failed
transaction and broke the test next door), and the AT32's model never clearing
its start bit, which the hardware does and which the driver's abort path
depends on.

## The board's own file, and the saved configuration

Linking the real `src/boards/AERIALKIT_F405/board.c` came last, because it needs
the whole port to link behind it. It bought the configuration record - a magic
word, a serial, a length, a checksum and the text, in a *ring* of thirty-two
slots in the last sector of the part - and the record's *refusals* are the half
that matters:

| Checked, by running it | Why it is the check |
| --- | --- |
| an erased sector reads as "nothing saved" rather than as a configuration of zeros | a part that has never been saved to is all ones, and treating that as settings is how a board boots with nonsense parameters |
| a complete record reads back, text and all | the fields are where the caller thinks they are |
| and the newest of two records is the one that loads | the ring's whole point: the serial decides which record is newer, not the slot number |
| a record whose bytes do not match its checksum is one invalid slot, and the record before it still loads | this is what a save interrupted by a power failure leaves - and with the ring it costs the new settings, not the aircraft's old ones |
| a save whose *program* is refused leaves the previous record readable | the failure the single-record save could not survive: it erased the only copy before writing the new one |
| thirty-three saves come round the ring and the last one loads | the bulk erase happens once every thirty-two saves, and the serial still says which record is newest after it |
| a sector of damaged records is an error rather than an empty part | "nobody has saved here" and "what was saved is gone" are different sentences, and the preflight prints them differently |
| a record of no length, a length past the end of the record, and a buffer too small are each refused rather than truncated | the three ways a caller or a corrupt sector can ask for the wrong number of bytes |
| a save the flash controller refuses is reported rather than assumed | the same failure path the flash checks describe, seen from the caller's side |

The checksum in the test is written from FNV-1a's own constants rather than
borrowed from the board file, so the two agreeing is evidence about the record's
layout rather than about the code agreeing with itself.

**The window that was here, and what closed it.** The F405 erases 128 KB at
once, so when all thirty-two slots have been used the sector has to be erased
before the ring can start again. This file used to record what a power cut
inside that erase cost - everything, once every thirty-two saves instead of on
every save - and that removing it needed a second sector to alternate between,
which would come out of the blackbox. That was accurate and it was the wrong
answer: the sector was spent.

The ring now alternates between the last two 128 KB sectors, and the erase
always lands on the bank the newest record is *not* in, so a power cut inside
it leaves every record of the current bank, newest included. `make
config-recycle` is the measurement: it drives the real board through
thirty-two saves, then a recycle whose program the controller refuses, and
reads the configuration back. With one bank that read returns zero - the same
answer a part nobody has saved to gives, which is why nothing downstream could
notice. With two it returns the last good record.

The cost is stated where it is paid: the blackbox is five sectors rather than
six, 640 KB rather than 768 KB. The AT32F435's port has 2 KB pages and erases
only the slot it is about to write, so it never had the window and is
unchanged.

Two host-build compromises, both stated in the test: `ak_arch_reset` is a stub,
because the real one is inline Cortex-M4 assembly a development machine's
assembler will not take and it is not what is under test; and the record's
address is repeated in the test, so a board that moves its sector has to change
that line too rather than silently reading a sector nobody writes.

## The USB console, which is where this technique paid for itself

The OTG FS controller is the device on this port that a host can *see*, and it
is also the one whose failure is quietest: a board that does not enumerate is a
board with no console, no error, and a cable somebody will unplug and plug in
again. It is on the same mapped-register treatment as everything else - the
window at `0x50000000`, its FIFOs 4 KB apart inside it - and that turned out to
be enough to run the whole driver, because the pieces a packet is made of live
in **two different registers**: `GRXSTSP` says whose the packet is and how many
bytes it has, and the bytes themselves come out of FIFO 0.

That the two are separate is the thing that makes the driver testable at all -
but the FIFO is a *queue*, and for a long time the test pretended it was not. A
mapped page returns the same word however many times it is read, so a setup
packet was two reads of the same four bytes; the test wrote the *request* into
the byte a host puts `bmRequestType` in, and the driver read the request out of
the byte a host has the *type* in. The two agreed with each other, which is why
thirty checks passed, and neither agreed with USB - so every request a real host
sent was stalled. **That is what the board did on 2026-09-17**: the device
appeared on the bus and then `device descriptor read/64, error -32`, `Device not
responding to setup address`, for ever. The FIFO is now behind a seam of its own
(`AK_HOST_USB`, `tests/host_usb_model.c`), the driver reads `bRequest` from the
byte USB 2.0 section 9.3 puts it in, and every packet in both tests is one a
host actually sends - with `SET_ADDRESS` among them for the first time. The
check that proves the test would have caught it: put the old decode back and
sixteen checks fail.

The checks that matter are the ones that failed first:

| What it found | What it would have looked like on the bench |
| --- | --- |
| the status word was being read out of *FIFO 0* instead of `GRXSTSP` | the first word of every payload is eaten as if it were a status word, so the host never receives what it asked for, at any point in enumeration. This is the check that the two addresses in `tests/test_regs.c` exist for |
| `STUPCNT` was never written, so endpoint 0 was never armed to hand over setup packets | ST's own driver programs three there on every return from a request (`USB_EP0_OutStart`). A device that never writes it answers the first descriptor request and then stops hearing anything - enumeration that gets half way and no error anywhere |
| the banner written before the host was ready was queued but never pushed | the ring is sized for exactly that banner, and nothing sent it once configuration arrived: the first thing a serial terminal would have shown is nothing, until the next console write, whenever that was |
| the second packet of a descriptor longer than one packet was taken from *past the end of the one-packet buffer*, with a length of zero | the configuration descriptor is **67 bytes**, so every host reads it as 64 and then 3 - and the 3 never came. What arrives is a configuration descriptor the host cannot parse: no CDC interface, no `/dev/ttyACM*`, no console, and nothing on the board to say why. Every test until then had asked for a descriptor that fits in one packet; the fix keeps the pointer the rest of the table is at, and the check is the two packets |
| **`bRequest` was read from the wrong byte of the setup packet** - out of the little-endian pair rather than from byte 1, which is where the host puts it | every standard request decodes as a number that matches nothing: `GET_DESCRIPTOR` reads as 0x0680, the device stalls it, and the host reports `device descriptor read/64, error -32` and then `Device not responding to setup address`. The device is on the bus and cannot be enumerated, which is a board with no console, no banner and no way in - and the host test could not see it, because it fed the driver the same reversed packet (2026-09-17's bench log, and Artery's own library for the same class of core parses the buffer the other way round) |

The class requests are the fourth thing that came out of reading the reference
drivers rather than the manual: Linux's `cdc_acm` sends `SET_LINE_CODING` and
`SET_CONTROL_LINE_STATE` when it probes and opens the port, and while it
survives a stall on either, a device that answers them is a device with one
fewer explanation for a port that behaves oddly. `SET_LINE_CODING` is the
interesting one because it has a *data stage*: the transfer only ends when the
device answers the status stage, so the test checks the zero-length packet that
finishes it and then that `GET_LINE_CODING` gives back the seven bytes.

Beyond the packets, both tests now drive the parts of the poll a host causes
rather than the test choosing: `SET_ADDRESS`, the status stage that arms endpoint
0 again, the speed interrupt, a descriptor type nothing implements, a console
line longer than one 64-byte packet (which is what makes the bulk IN endpoint's
continuation matter), a write larger than the transmit ring, and the bulk OUT
completion that arms the endpoint for the next packet. Both `usb.c` files have
**no line the host suite does not reach**.

What the technique still cannot reach is the bus. There is no timing, no
DATA0/DATA1 toggle, no retry when the endpoint NAKs, and the core's own half of
the contract - that reading the bytes is what drops a status entry, that EPENA
clears when a transfer completes - is the test's premise rather than something
observed: a page of memory has to be *told* that a completed transfer disabled
the endpoint, and that writing a bit back to an interrupt register clears it,
which is why the test does those two things itself. Two of the bugs above were
only *reachable* because the test sets the registers a real core would; whether
the core agrees is exactly the question the bench answers, and `journalctl -k`
is the instrument.

## The third part, and the same two seams

The AT32F435 port needed both of these models and neither of them was the F405's
file with different numbers: `tests/host_flash_model_at32.c` behind
`-DAK_HOST_FLASH_AT32` (two banks, 2 KB pages, an address register and a status
word where this part clears flags by writing a *zero*) and
`tests/host_i2c_model_at32.c` behind `-DAK_HOST_I2C_AT32` (a peripheral that is
told the address, the direction and the byte count at once, and does the
address phase, the acknowledge and the stop by itself).

The I2C one paid for itself on its first run, which is worth recording because
it is the third time in this project that a check has been wrong or caught
something on the day it was written rather than months later on a bench:

| What it found | What it would have looked like on the bench |
| --- | --- |
| the mask that clears the fields a new transfer owns left out the byte count, so each transfer OR'd its count into the previous one's | the first transfer after a boot works and every one after it times out: three bytes, then one, and the part still told to move three. That is a barometer that answers once and then never again, and the part it is blamed on is the barometer |

The mask is the reference's now (0x03FF67FF), and the constant beside it says
where the value came from rather than where the comment came from - the comment
was right and the value was the low half of it, which is exactly the sort of
mistake that is invisible without something that answers.

The third part's USB is the one device on it that did *not* need a model: its
controller is the same core the F405 has at the same address, so the F405's test
- a mapped page, the status queue driven by hand, a device walked through
enumeration and the class requests a serial driver makes - runs against it
unchanged, 32 checks. What is different is checked on the way: the USB clock
comes from the PLL divided by six rather than from the crystal, the core starts
in power-down, the phy clock starts gated, and GUSBCFG bit 6 is this part's
turn-around time rather than the F405's transceiver select - so the F405's line
there would have written zero into a field this part needs to carry packets.
Those four are exactly the kind of difference that a "same core, same driver"
claim has to be able to point at, and the other three are in the file with the
bit each one came from.

## The same method, on the second target

The ESP32 does not have registers to map - it has IDF, and a driver owns each
peripheral - so the seam is a different shape: a stand-in for IDF's own headers
(`tests/idf-stub/`) and a model behind them (`tests/host_esp32_*.c`), with the
port's real files compiled against both. Two of that port's three never-run
areas are closed that way now, and both found something:

| | what runs | what it found |
| --- | --- | --- |
| the outputs | `src/arch/esp32/output.c` against a modelled RMT and LEDC, with the symbols handed over decoded back into the frame an ESC would see | **every frame was a quarter of a frame**: `rmt_transmit()` takes the payload's size in bytes and the port was passing its symbol count |
| the sensor buses | `src/arch/esp32/spi.c` and `i2c.c` against modelled devices - a register file that auto-increments, a select that is held or bounced, a part that stops answering - with the core's own ICM-42688-P and DPS310 drivers on top of them | the wire, the address byte, the register pointer and the burst framing are what they claim to be, so far |
| the receiver's and the GPS's UARTs | `src/arch/esp32/uart.c` against a modelled IDF UART: a ring the test fills, the driver's event queue, a transmit buffer, and switches for the failures a bench cannot arrange on purpose | the protocol switch, the drained events and the once-only framing line all hold; nothing was wrong here |

The second row's *first* finding was about the harness rather than the port: a
delay in a port file (`ak_imu_open()` asks its bus for the datasheet's fifteen
milliseconds) waits on a tick that no timer moves on a host, so the test hung.
On the host build a delay now advances the modelled tick
(`-DAK_HOST_TICK`, on `src/core/time.c` and the F405's systick object alone) -
a property of the test environment, not of the firmware, and the only way a
host can be honest about a wait.

What is left on that target with no host coverage is the radio - Wi-Fi is a
vendor blob driving a peripheral this machine does not have, and it cannot be
moved here. It is at least run end to end under QEMU, with the console, the
protocol, the receiver's and GPS's ports, the sensor buses' *absence*, and the
configurator's window all talking to it.

## The stack, which is the one question running the code cannot answer

Everything above *executes* the port. This section is the other kind of check:
one the compiler can answer about the image and no run on a development machine
can, because the test build gets eight megabytes of stack and the part has tens
of kilobytes. A firmware that overflows its stack on the bench does not fail
here - it fails there, as a board that goes quiet between two things it was
supposed to print, which is the second half of the same symptom the section on
the USB console ends with.

Two flags and a walk of the graph answer it. `-fstack-usage` makes GCC write
the frame size of every function it did not inline into a `.su` file beside the
object, and the image build has passed it since the first target - it is what
`size` and the readiness work have been quoting all along. `-fcallgraph-info=su`
makes it write who-calls-whom into a `.ci` file, and `tools/stack_report.py`
walks the second to add up the first. `make stack-check` is the two together
(about twenty seconds, and the ARM toolchain and nothing else - no board).

The flag has to reach the *compiler*, and the Makefile sets `CC := $(CROSS)gcc`
with the options after it, so it goes on `CC`. **The first draft of the script
passed `EXTRA_CFLAGS=`, and this sentence used to add that no target in this
Makefile reads it - which is false, and has been since the day this script was
committed: `e454275` added `CFLAGS += $(EXTRA_CFLAGS)` to the Makefile in the
same commit, and that line is what builds the fitted and tell-tale images.**
So the draft's failure cannot be re-derived from the tree today and is left as
what it is, an observation whose stated cause no longer holds; the report
printed a graph of one node and a "deepest chain" of one function, a check that
passed by not running, which is the failure mode this repository keeps meeting
from the other side. It is written down here because the printed number was
plausible. The rule survives the reason and has a better one: a *command-line*
`EXTRA_CFLAGS` overrides the environment, so sending the callgraph flag that way
would displace a board variant's defines rather than join them, and `CC` is a
place nothing can take it away from.

| What the part has, from `arm-none-eabi-size -A` on the image | Bytes |
| --- | --- |
| SRAM (112 KB of SRAM1 and 16 KB of SRAM2, laid out contiguously) | 131,072 |
| `.data` + `.bss`: every static buffer, the rings, the parameter table | 56,828 |
| `.noinit`: the fault record and the retained log, which survive a reset | 21,584 |
| left between the last section and `_estack`, which is where the stack runs | 52,656 |
| CCM, mapped and unclaimed (`docs/02-hardware.md`) | 65,536 |

Measured on 2026-09-17, F405 and AT32 images (`docs/evidence/stack-f405.txt`):

| | Bytes |
| --- | --- |
| the deepest chain of calls the compiler can resolve | 2,480, over 5 frames |
| the same walk with one largest-frame call at every unresolved site | 8,584 |
| the chain through `ak_flight_step`, which is the control loop | 116, over 4 frames |
| the largest frame anywhere under `src/core/flight/` | 112 (`ak_log_write_record`) |
| the three interrupt handlers | 0 to 4 each |

The five frames of that deepest chain are the whole story of where this
firmware's stack goes: `Reset_Handler` (8 bytes) calls `ak_firmware_main` (264,
which includes everything the compiler inlined into the loop), which calls
`preflight_run` (2,120 - a report builder), which calls `arm_line` (64) and
`ak_flight_arm_check` (24). Read the list of largest frames rather than that one
number and the pattern is plainer: `preflight_run` 2,120, `save_parameters`
2,056, `ak_flight_selftest` 552, `ak_cli_run` 368, `ak_flashlog_dump` 360,
`calibrate` 288 - every one of them a thing that happens on the ground, at
boot, or because somebody typed a command. The control step's own chain is a
hundred and sixteen bytes, and the AT32 image reports the same five frames
because the chain is in `src/core/` and both parts share it.

So the deepest thing the compiler can see is 4.7% of what is free, and the
pessimistic reading is 16%. What that has bought is not a proof, and the four
things it is not are worth as much as the number:

**It is not a proof, because function pointers have no edges.** This firmware
hands drivers around as tables - four of them, deliberately - so the callgraph
has **506** call sites it cannot follow, and the report counts each as zero.
The 2,480 is therefore a *floor*. The 8,584 is what the same walk costs if
every one of those sites calls the largest frame in the whole image, which
cannot be true of all of them at once, so it is a *ceiling*. The truth is
between the two, and the gap is the reason the Makefile's `STACK_LIMIT` is a
tripwire at 12,288 bytes with room above today's reading rather than a budget
anybody has earned.

**It says nothing about what runs at the same time as what.** These are chains,
not sums: an interrupt that fires while the loop is three frames deep puts its
own frames on top, and nothing here models that - except to note that the three
handlers this port has are 0 to 4 bytes of frame each.

**A frame size exists only for a function the compiler did not inline.** The
report has 403 of them for a program with more functions than that, and the
locals of everything inlined are counted in the caller's frame - which is the
right answer, and also the reason a person looking for `nav_update` in the list
will not find it.

**And it is the F405 and the AT32, not the ESP32.** The second target's stacks
are per-task and sized in its own configuration; `make stack-check` builds one
image and the ESP32's is not one of them.

`scripts/check-stack.sh AERIALKIT_GHF435 at32f435 at32f435rg aerialkit-ghf435`
runs the same check against the wing's part, and `--from NAME` (repeatable)
roots the walk somewhere other than the whole image - which is how the
control-loop number above was measured rather than guessed at.

## The same suite under the sanitizers, and what was undefined in it

Every check in this file asks whether the firmware computes the *right* answer.
None of them could ask whether the *way* it computes it is defined, because on
the compiler in front of you the undefined thing almost always does what you
meant: an out-of-bounds index lands in the next array, a signed overflow wraps,
a left shift of a negative shifts. That is what `make sanitize` is for. It
builds this suite and all thirty simulated sessions with
`-fsanitize=address,undefined`, runs them, and fails on the first finding
(`-fno-sanitize-recover=all`) rather than printing a list and exiting 0. It
takes about four minutes on the NAS, needs no ARM toolchain and no board, and
`build-asan/` is its build directory.

It found the following on its first run, in a suite that was 1441 checks green:

| Where | What the sanitizer said | What it would have been |
| --- | --- | --- |
| `tests/test_aerialkit.c`, `tests/crsf_fixture.c` | `index 22 out of bounds for type 'uint8_t [22]'` | the CRSF frame packers wrote a third byte for the last of sixteen eleven-bit channels: twenty-two bytes to the bit, so the last channel ends on the last bit. Both wrote a zero there, so the frames were right and nothing noticed - the sort of bug that stops being harmless the day somebody reorders the stack |
| `tests/test_arch.c` (two checks) | `load of address ... with insufficient space`, then ASan's `global-buffer-overflow in text_sum` | the saved-configuration checks passed a length of 26 for a 24-byte string. The record's checksum read two bytes past a string literal, and the two bytes went into a field nothing compared - the AT32's version of the same check has always used `strlen`, which is why only this one had it |
| `src/core/sensors/ak_baro_bmp280.c` | `left shift of negative value -26983040` and `-4509070` | **the datasheet's own arithmetic**: it writes signed left shifts and a real part's calibration makes those operands negative. Five shifts in that function are that shape. The numbers are unchanged; they are done on the unsigned bit pattern now, which is the arithmetic every compiler here has always produced, and the sanitizer can see it is defined |
| `src/core/sensors/ak_baro_bmp388.c` | `signed integer overflow: 16777215 * 1140850620000` | the pressure compensation multiplies by the raw reading twice, and at a full-scale reading the second product wants 1.9e19 against a signed 64-bit ceiling of 9.2e18. The reading is one the driver *refuses* - but it refused it after the undefined thing had already happened. Guarded now, before the multiply; a reading that cannot be carried gets the same answer the datasheet's own zero-division case gets |
| `tests/test_log.c` | `index 272 out of bounds for type 'uint8_t [272]'` (three sites) | the fake flash is a flat part modelled as `uint8_t [sectors][bytes]` and indexed as `fake_flash[0][offset]`, which is out of bounds of the *sub-array* as soon as the offset leaves the first sector even though the bytes are there in the object. The accesses go through a byte pointer into the whole array now, and a read or a word-write that would hang off the end is refused rather than half done |
| `tools/fw_sim.c` | `index 8192 out of bounds for type 'uint8_t [8192]'` | the same flat-index shape in the simulator's flash double - which every simulated session exercises, so it was live in every SIL run this repository has ever done. `tools/akproto_sim.c` had it too and was *not* reported, because the protocol checker is not part of `make test`; it was found by grepping every flat index in the tree once this one was named, and fixed with it |

The two firmware findings are the ones worth keeping. Neither could have been
found from the outside - the aircraft flies, the barometer reads the right
pressure, the tests pass - and both are in the arithmetic that decides an
altitude, which is the number a fixed wing's return-to-home and altitude hold
are built on. The BMP388 one is the more interesting: the reference
implementations (Bosch's API and Betaflight's port) write the same unguarded
product, so this firmware is where the guard lives, and `docs/03-attribution.md`
cites them for the formula rather than for the guard.

What it is not: a proof. It is the run that was done, on the paths that were
run - the sanitizer sees the tests' inputs and the sessions' trajectories, and
nothing here is coverage-guided. What it is, is a check that costs four minutes,
fails loudly, and would have caught every one of the seven rows above.

**And the set of sanitizers grew once the first wave was fixed.** `undefined`
does not include the two that matter to a flight core in a different way: a
float division by zero is not undefined at all - it is `inf`, which propagates
quietly into an altitude or a motor output - and a float-to-integer conversion
that is out of range is the same shape. `make sanitize` checks both now
(`-fsanitize=float-divide-by-zero,float-cast-overflow`), and the divisions came
back clean across the suite, the thirty sessions and the fuzzer, which is
worth having: the flight maths divides by airspeed, by altitude differences and
by `dt`, and none of those divisions is by zero on any path this machine can
fly.

**The conversions did not, and where it failed is the point.** It stopped in
the loop that writes one blackbox record per iteration:

```text
src/core/main.c:981:55: runtime error: 32768.5 is outside the range of
representable values of type 'short int'
```

That is a float-to-`int16` conversion of 3276.85 degrees. The field is the log's
`yaw`, a tenth of a degree in an `int16`, and it holds up to 3276.7 - so this
had been there since the field was added, on any flight long enough to turn the
aircraft nine times. Nothing had ever turned it that far, because it takes
minutes: the first thing in this repository long enough to do it is the
three-hundred-second hold session added to `make sil` in the same piece of work
this page is about, and the first run of the sanitizers that included those
sessions found it inside the log writer.

The reason it is worth a section rather than a line is that the angle *should*
be allowed to get that big. An attitude is carried through the flight core as a
continuous angle on purpose: the quadrotor's hover reconstructs the course it is
making by subtracting two yaw readings, and an angle that wrapped underneath
that subtraction would turn a quarter turn of travel into three quarters of one
the other way. The fields an angle *leaves* the core through are periodic - a
tenth of a degree in an `int16` overflows after nine turns, and a CRSF attitude
frame's 1e-4 radians overflows after 187 degrees - so the wrap belongs at the
conversion, not in the estimate. It is `ak_wrap_pi()` in
`src/core/flight/ak_math.h` now: whole turns out in one step, at most one turn
of correction, total for any input (a NaN, an infinity or an absurd angle comes
back as zero rather than as undefined), and measured at 0.0016 degrees of error
over the +/-159 turns a session can reach.

The record and the attitude telemetry are both written through
`ak_attitude_ddeg()`, so a heading read off the console and a heading read out
of a log cannot come out differently. `make test` has six checks on it - five on
the wrap, one on the CRSF frame, including the sanitizer's own number at nine
turns - and every recorded log in `docs/evidence/` was re-recorded from this
build: 2911 records, none of them with an attitude outside the +/-180 degrees
the field holds. Before the change 1881 of those same records carried a value
outside it, because they were continuous angles; they are headings now, which is
what a field that narrow can actually hold.

## The ESP32's pack converter, and a unit that was off by one

`src/arch/esp32/adc.c` was the last file on that target a host had never run.
The reason is in its own comment: this chip does not hand back counts to be
converted, it hands back **volts**, because the original ESP32's calibration is
line fitting in eFuse - and that calibration is IDF's code, which is exactly
the part a host cannot have. QEMU models no ADC, so the file had been compiled
and nothing more.

The split that makes it testable is the one the port already makes: IDF does
the conversion, and the port does *the unit, the channel, the attenuation,
which calibration the answer came from, and what to do when any of it fails*.
So `tests/idf-stub/esp_adc/` declares the three headers the file calls the way
IDF declares them - including the enum values, which matters here - and
`tests/host_esp32_adc_model.c` answers them with a raw reading the test sets
and a line through two points the test sets. `tests/test_arch_esp32_adc.c`
drives the real port file against that: 29 checks covering the converter coming
up, the reading being the chip's line rather than the datasheet's, the report
saying which calibration it used, a unit or channel nobody opened being an
error, a failing conversion staying an error, and a converter that never comes
up being refused rather than retried behind the caller's back.

**And it found the bug that the file had been carrying since it was written.**
The board header says:

```text
#define AK_BOARD_VBAT_UNIT      1 /* ADC1 */
#define AK_BOARD_VBAT_CHANNEL   6 /* GPIO34 */
```

which is right about the chip: GPIO34 is ADC1 channel 6. But IDF's `adc_unit_t`
counts from zero - `ADC_UNIT_1` is **0** and `ADC_UNIT_2` is 1 - so a board
asking for ADC1 was asking IDF for *the second unit*: different pins (none of
them GPIO34), and the unit the radio owns whenever Wi-Fi is on. The port passes
the unit through now with the translation written down in one place
(`unit - 1`), and the check asserts both halves of it: the board's number is
what the report prints, and IDF is given the enum that goes with it.

The same model caught a second, smaller one: a second `ak_esp_adc_init` asking
for a *different* unit returned success and kept the first converter, so a board
with two channels would have got one channel's readings under the other's name.
One converter per board is what this port supports, and it says so now.

What the model is not: a converter. There is no attenuation curve, no noise, no
sample time and no eFuse - the two points are the test's, so every voltage in
that file is a number the test chose, and what is being checked is the port's
side of the contract rather than the silicon's. The one thing on this target
that no host could reach was the radio - it is a vendor blob for a peripheral
this machine does not have - and even that has stopped being true for the
port's side of it: see the next section.

## The ESP32's network, in the configuration a board is in

`src/arch/esp32/net.c` is the reason that target exists, and it builds its
medium at compile time: QEMU's ESP32 has a PHY and no radio, a devkit has the
reverse, and a build that carried both would have one of them failing on every
board it ever ran on. So every automated run of that file - the protocol over
the network, the telemetry stream, the configurator's window over a forwarded
socket, all of it in `docs/evidence/esp32-*.txt` - has been the *Ethernet*
build. Everything under `#if CONFIG_AK_NET_WIFI` had been compiled and never
run, here or anywhere else, because the owner's board is one of the two answers
this work is parked on. That is the same hole the ESP32's ADC, its buses and
the fitted half of a board file each had, and it is closed the same way.

The real file is compiled a second time, as the Wi-Fi build, against stand-in
IDF headers in `tests/idf-stub/` - esp_wifi, esp_netif, esp_event, lwip's
sockets and FreeRTOS's stream buffers, each a subset of IDF v5.5's own with the
file it came from named in it - and run by `tests/test_arch_esp32_net.c` against
`tests/host_esp32_net_model.c`: a radio that records what it was asked to do, a
netif whose address the test sets, an event loop the test raises events
through, stream buffers that really are buffers, and a socket that plays a
client - one that goes quiet, then sends a frame, then hangs up, which is the
sequence the task's loop exists to survive.

**Thirty-eight checks**, and what they pin is the *decisions* rather than the
radio: which role the image takes for a given pair of settings and what happens
when there is nothing to join; that the driver is told `WIFI_STORAGE_RAM`,
because the credentials are parameters and flash is for the record `save` owns;
that the station asks for WPA2 and that the access point refuses to come up
without a real password - the link is the only authentication this protocol
has; that the two events the console depends on, the address arriving and the
link dropping, are registered and acted on, re-association included; that the
report says "not listening", "no address" or the address, and not silence; and
the socket task itself, one client at a time, with every byte the client sent
reaching the flight loop and every byte the loop wrote leaving on the same
pass. The file is 170 of its 179 lines under the coverage map, where it was
not compiled at all before.

Two things it found on its first run, which is what it is for:

- **The access point's name and its length could disagree.** The name was
  copied into a 32-byte field with `strncpy(dst, src, sizeof dst - 1)`, which
  holds 31 characters and a terminator, and the length field was taken from the
  name it was *given* rather than from the copy - so a 32-character
  `wifi_ap_ssid` broadcast a length the array did not hold. The copy is bounded
  and always terminated now, and the length is what it copied.
- **The report printed `0.0.0.0` where the event handler's line says "no
  address yet".** The `net` command is the one a person types at a bench when
  the configurator cannot find the board, so it says the same thing now.

And one thing it pinned without changing. `select()`'s timeout is
`{ 0, 2000 }`, and a `timeval`'s field is *microseconds* - so the socket task
wakes every two milliseconds while a client is attached, not every two seconds.
That is the right shape for this loop (an answer the flight loop wrote must not
wait for the client to say anything) but nothing in the file said which unit it
was, so the check asserts the two milliseconds and the code says so in a
comment. A silent change to "2000 milliseconds" would be a two-second delay on
every push of telemetry, and it would look like a radio problem.

What this still cannot say: that a radio does what it is told. That needs a
board, and it is the owner's ESP32 choice, which is parked with the airframe.

## The fault record, which is the file that explains a board that stopped

`src/arch/arm/cortex-m4/fault.c` is reached from the vector table when something
has already gone wrong, and that is the one thing a test cannot arrange: a real
fault needs the part. It is also the file whose *silence* cost this project its
two bench sessions - a board that stops between the banner and the loop is a
board whose fault record nobody has read. So the record half of it is a function
of its own now (`ak_fault_record(frame)`), the stop stays where it was
(`ak_fault_capture(frame)` calls it and then loops), and `tests/test_fault.c`
calls the record with a frame it makes itself:

```text
r0 r1 r2 r3  r12 lr pc xpsr      <- eight words, in that order, on the stack
```

Thirteen checks. The one worth naming is the **pc**: the record has to contain
the *frame's* pc - the instruction that faulted - and not the address of the
handler that recorded it, because that number is what somebody feeds to
`arm-none-eabi-addr2line` afterwards. An off-by-one anywhere in that layout
does not fail loudly; it sends the reader to a line of code that was fine. The
rest are the same family: the fault status registers come from the part's own
registers (through the page `test_arch.c` maps), the count is the number of
faults since power-on - which is how a person tells a loop of resets from a
single stop - and a record with no magic is *not* trusted, which is what an
unfinished record looks like when the write itself caused the reset.

What is still not exercised, and cannot be here: the stop itself (a test that
cannot return is not a test), and the ESP32's equivalent - that port has no
handler to run, because IDF owns the panic path, so `src/arch/esp32/fault.c`
asks IDF *why* the chip restarted on the next boot. Its no-fault path runs at
every emulated boot; its "it restarted because it panicked" branch has never
been taken, and the honest way to take it is a devkit with a deliberate crash.

## The same story on the second target, where IDF owns the panic path

That last paragraph is a promise to come back, and this is it. The ESP32 has no
handler of its own: IDF prints the backtrace, dumps core if it is configured
to, and resets - so the port's equivalent of the ARM record is to *ask* IDF why
the chip restarted (`esp_reset_reason()`) and to keep that in the record's
fault-status field. Three files on that target were run for the first time
against `tests/host_esp32_system_model.c` and the two headers it needed
(`tests/idf-stub/esp_system.h`, and the GPIO surface `led.c` calls, whose enums
are IDF's own values because the model's job is to see what the port asked for):

| Checked, by running it | Why it is the check |
| --- | --- |
| a power-on, a software reset and a deep-sleep wake leave no fault | the record exists to explain a *crash*; a chip that was switched on is not one, and a file that recorded every boot would be a fault log nobody reads |
| a panic, the task watchdog, the interrupt watchdog and IDF's other watchdog each record a fault, with the reason in the field | this is the whole diagnostic on a devkit: "it restarted because it panicked" against "somebody rebooted it". The reason is kept in `cfsr`, which is a fault *status register* on the ARM port and "which kind of failure" here - the field's meaning is the reader's, not the part's |
| and the pc is zero, deliberately | IDF printed the backtrace; this port did not keep it, and a made-up address in that field would send somebody to `addr2line` for nothing |
| the CPU clock is the value the *build* was configured with | `system.c` reads it rather than querying it, so a configuration that changed under it is a wrong number in the banner rather than a crash - and the ESP32 build uses IDF's generated header, which is the check on this stub |
| a reboot asks IDF for exactly one | `ak_arch_reset()` is one call, and the console's `reboot` is the command that reaches it |
| the LED's pin is configured as an output, with no pull and no interrupt, and starts dark | two of the four tell-tale patterns on that port are this file: a pin configured with a pull-up is an LED that is dim, and one configured as an input is an LED that never lights |
| and every later `set` drives the pin it was initialised with | the blink loop calls `set` thousands of times and never re-initialises, which is a thing a static pin variable has to get right |

Nothing was wrong here, which is worth saying plainly - the third time in this
file that a check has come back empty, and the reason it is written down is the
same each time: a verification that only ever finds bugs is a verification
nobody trusts when it finds none. The renamed entry points are the one
mechanical trick worth naming: one firmware has one `ak_fault`, and the tests
binary already holds the ARM port's, so this port's record, present, clear and
`ak_arch_reset` are compiled with `-D` renames for the host build alone - the
same trick the third target's board file uses to live in one binary with the
F405's.

## The parsers, under bytes nobody sent them

Everything above feeds the firmware *frames*: built from the wire description,
plus the handful of malformed cases somebody thought of. But every parser here
eats a wire that somebody else is in charge of - CRSF and SBUS from the radio,
UBX from the GPS module, the config protocol on the console or the network - and
the failure mode that matters is not "it decodes a bad frame", it is "it stops
decoding good ones". A parser left waiting for a length that never arrives is a
link that stays dead until the aircraft is power cycled, which is why the CRSF
parser has a gap timer in the first place.

`make fuzz` is `tools/fuzz_parsers.c`: a fixed seed, a lot of bytes, and two
properties checked on every iteration.

| Checked | Why |
| --- | --- |
| nothing crashes, out of bounds or overflows | which is what the sanitizers see: `make sanitize` runs this same tool against the sanitized build, and that is where it earns its keep - an index computed from a length field is invisible in an ordinary build |
| after *any* amount of random input, a valid frame still decodes | the flight-relevant property above, checked with a frame this tool builds itself from the wire description, so "valid" does not depend on the parser |
| and the same after a frame that is *nearly* right: a valid one with bits flipped and then cut short at a random byte | pure noise reaches a parser's length checks and its sync search but rarely its state machine; a damaged frame reaches all three |
| a whole-payload unpack (`ak_crsf_unpack`) stays inside eleven bits per channel | the same question one layer down, where a channel unpacker that trusts its input would be the bug |

Measured on the NAS on 2026-09-17:

```text
make fuzz                             200,000 iterations, 102 MB of random bytes, 1.6 M decode-checks, all clean
make fuzz HOST_OUT=build-asan ...     the same run under ASan+UBSan: 55 s, no findings
five more seeds at 60,000 each        another 154 MB, all clean
```

**It has not found anything yet**, which is the fourth check in two days to come
back empty and is said here for the same reason as the others. What it is *not*:
coverage-guided, or a proof. It is a fixed seed and a lot of bytes; the honest
reading of a green run is "nothing this seed found", and a new seed is one
argument away (`make fuzz FUZZ_SEED=0xdeadbeef`). What it is, is a check that
runs in seconds as part of `make test`, and stops a parser from being changed
into one that can be wedged by noise.

## And the whole suite as the laptop sees it

This machine is aarch64 and the machine of record - the laptop, and the one that
flashes the board - is x86_64. "It builds and runs on both" is a claim the plan
makes, and until now it was a *procedure* rather than a command: install a cross
compiler and qemu, type three lines, read the output. Procedures like that go
stale quietly, so `make cross-check` is the three lines:

```text
make cross-check     2001 checks, 0 failed; two simulated sessions PASS; the fuzzer green
                     (~80 s on the NAS, and it skips with a line where the
                     cross toolchain is not installed)
```

What it catches that a same-architecture run cannot: a signedness assumption
(`char` is signed there and unsigned here, which has already cost this
repository one warning in the ESP32 UART check), a pointer width, an alignment,
and a warning that only one of the two compilers has an opinion about. What it
is not: the laptop. It is qemu-x86_64 with the cross libc, so it proves
architecture-independence and compiler cleanliness, not that the laptop's
toolchain is happy - which is what running `make test` there is for, and the
reason this target says "the same suite", not "the machine of record".

It is deliberately *not* part of `make test`: it needs a cross toolchain this
machine happens to have, and it would double the suite's time for a claim that
only changes when the code does.

## What the F405 port writes, against the reference drivers

The most expensive bug in this project's short life was a **register the driver
never wrote**: the OTG core's `GCCFG`, which left the USB device powered down
and put nothing on the bus for twenty-five seconds while every host test was
green - because a mapped-register test checks the writes a driver *makes*, and
that one was absent. That is a class, not an accident, so the whole port has
been read against the reference drivers since (`upstream/betaflight-2026.6.1`,
whose HAL configuration for each peripheral is what ST's own driver does), and
what follows is the result: what is written, and the handful of things that are
deliberately not.

| Peripheral | Written, and checked against the reference | Deliberately not |
| --- | --- | --- |
| clocks | `RCC_AHB1ENR`/`AHB2ENR`/`APB1ENR`/`APB2ENR` for every peripheral the port touches (GPIOs, TIM2/3, DMA1, USART1-6, SPI2/3, I2C1-3, ADC1, OTG FS), then read back | - |
| GPIO | `MODER`, `OTYPER`, `OSPEEDR`, `PUPDR`, `AFRL/H` per pin, with the speed each signal needs (fast for DShot, default elsewhere) | - |
| USART | `BRR` (the divisor, after the bug that made every port sixteen times slow), `CR1`/`CR2`/`CR3`, and the pins on their alternate function | `GTPR` (smartcard, not used), `CR3.HDSEL` (half duplex) |
| SPI | `CR1` (master, mode 3, software NSS, APB1/8, enabled last), `CR2` | `I2SCFGR` (its reset value is SPI mode), `CR2.SSOE` (NSS is software) |
| I2C | reset, `CR2.FREQ`, `CCR`, `TRISE`, `CR1` (`PE\|ACK`) last, and pins on the alternate function with open drain | `OAR1/OAR2` (this firmware is a master only) |
| ADC | `CCR.ADCPRE`, `CR1` (12-bit), `CR2.EOCS`, `SQR1/SQR3`, `SMPR1/SMPR2`, then `ADON` and `SWSTART` | `CCR.TSVREFE`/`VBATE` (no temperature sensor or VBAT channel in use) |
| TIM2/TIM3 | `PSC`, `ARR`, `CCRx`, `CCMRx` (PWM mode 1, preload), `CCER`, `CR1.ARPE`, **`EGR.UG`** to latch the shadow registers, `DIER.CC1DE` for the DMA burst, `DCR` for the burst base and length | `BDTR`/`MOE`, which only the advanced timers have - TIM2 and TIM3 are general-purpose, and a port that moved to TIM1 or TIM8 would need it. That sentence is the point of this table |
| DMA | the stream's `CR`, `PAR`, `M0AR`, `NDTR` (memory-to-peripheral, half-words, incrementing, high priority, transfer-complete interrupt, channel 5), then the NVIC enable | FIFO mode and burst beats (direct mode is what a four-transfer burst wants) |
| flash | `ACR` (wait states, prefetch, caches), the `KEYR` unlock sequence, `CR` per operation, and every `SR` flag checked after each step | - |
| SysTick | `LOAD` from the actual system clock, `VAL`, `CTRL` (core clock, interrupt, enable) | - |
| USB | core reset, `GUSBCFG`, **`GCCFG`** (`PWRDWN` and `NOVBUSSENS` - the bug above), FIFO sizes, `GAHBCFG`, `GINTMSK`, `DCFG`, `DIEPMSK`/`DOEPMSK`/`DAINTMSK`, endpoint 0, `DCTL` cleared to connect | the reference's 20 ms wait after the reset (this runs before the tick exists - trap §17), `GUSBCFG.TRDT` (full speed needs none) |

The result is the honest one: **nothing missing was found** this time, in any of
the files above, and the two things that *were* found earlier in the day came
from exactly this reading (the `GCCFG` pair, and the AT32 writing half of them).
What the table is for is the next person: it says which registers are written
deliberately and *why the others are not*, so a driver that adds a peripheral -
or moves an output to an advanced timer - has a list to check itself against
rather than a blank page and a green test suite.

### And the same reading for the wing's part, where the clocks are not the F405's

The AT32F435 is the third target and the board the wing will fly on, and its
clock tree is close enough to the F405's to be dangerous: the same "APB timers
run at twice the bus clock" rule, but a different register layout underneath.
The four questions that decide whether the motors and servos move, read against
Artery's own driver library and Betaflight's AT32 platform layer:

| Question | Answer |
| --- | --- |
| what clock do the timers actually run at? | the system clock, 288 MHz. Artery's `timerClockFromInstance()` doubles the APB clock **only when the APB divider is 4 or more**; this port runs APB1 at the system clock, so there is no doubling and `AK_TIMER_HZ` is the system clock |
| is there a second divider inside the timer? | yes - `CTRL1.CKD`, which the reference always writes to `TMR_CLOCK_DIV1`. Its reset value is `0x00` *and* this port assigns `CTRL1 = OCMEN` (rather than OR-ing), so the field is DIV1 either way. The assignment is in `output.c` for both timers |
| does the shadow-register latch happen? | yes: `EVEG.UG` for TMR2 and for TMR4, the same write the F405's port makes with `EGR.UG`. Without it the first period after `TMREN` uses the reset prescaler and period |
| how does the DMA burst find its trigger? | through the DMAMUX: the controller's `MUXSEL` bit is set and each channel's request id is programmed - `0x43` for `TMR4_CH1` and `0x44` for `TMR4_CH2`, which are Artery's own numbers from `at32f435_437_dma.h`, cited in `src/arch/at32f435/regs.h` |

Nothing was missing here either, and the four rows are the reason this was worth
doing rather than assuming: every one of them is a place where a plausible port
would produce a servo period that is twice what was asked for, a first DShot bit
at the wrong rate, or a burst that never starts - and no host test can see any
of it, because the tests encode the *assumed* timer clock.

## What the firmware runs, and what it never runs

Everything above asks whether the firmware is right. `make coverage` asks which
parts of it were ever *asked*: the host tools built with `--coverage`, the suite,
the fuzzer, the bench checklist, the protocol clients and a session of every
kind, then `tools/coverage_report.py` over the `.gcno`/`.gcda` GCC leaves behind.
It is a map for the next test rather than a score to hit - the parts no host can
reach are the port registers (a mapped page is not a chip), the radios and
anything that needs the part to act on a write - and it found the one that had
gone unnoticed for the whole project:

```text
the firmware, 77 files:    96.9% of lines executed (8332 of 8595)
  src/core/flight 99.0%     src/core/sensors 98.8%     src/arch 98.5%
  src/boards 96.6%          src/core/main.c 92.3%       src/core/ak_cli.c 87.7%
  the largest gaps: main.c 110 lines, ak_cli.c 45, the F405's board file 14,
                    ak_text.c 8, the GHF435's board file 7, the AT32's usart.c 6
```

**Two entries have left that list since it was first taken, and both of them
were a path nothing had ever asked about.** The F405's SPI *loop* - the register
loop a target runs for the bench board's sensor bus - and the AT32's
`flash.c`, whose give-up path no model could reach until the model could say
"this operation never finishes". The numbers above are from 2026-09-18, after
the configuration ring replaced the single record in both board files, which is
also why `src/boards` moved from 96.3% to 96.6%.

**The last two points of that are this map doing its job.** `main.c` was 80.5%
with 276 lines nothing reached, and most of the difference was
`preflight_run()`'s FAIL branches - the diagnoses a person reads first, over
USB, when a fresh board does not behave - which had been compiled and never
executed, because the simulator's board was always the well-behaved one. The
`badboard` sessions ([15-preflight.md](15-preflight.md), and "the board is the
one that is wrong" in [18-software-in-the-loop.md](18-software-in-the-loop.md))
are a board that lies one fact at a time, and they took the file to **83.2% with
238 lines left**. What is left there is the console's own refusal messages for
arguments nobody typed, plus the two ports' register paths, which are the part a
mapped page cannot prove. `console.c` reached **100%** in the same run.

**Of `ak_cli.c`'s own 45 lines, all but a handful are one shape**: the
`io.hook == 0` refusals - `this board has no outputs`, `this board has no
receiver input`, `this board has no inertial sensor`, `this board has no
rangefinder`, and the same sentence for the barometer, the pack, the sensor bus,
the blackbox, the logs, the GPS, the navigator, the preflight, the protocol and
`reboot`. They are the *core's* contract for a build that leaves a hook out, and
`main.c` fills every one of them for every board, which is why no build on this
machine reaches them: the sentence a person actually reads on a board without a
part is the board layer's own (`--    baro:      none fitted`), not this one.
Kept, counted, and written down here rather than deleted - a core that refuses
in its own words is what makes the hooks optional in the first place.

**And the sensor buses came down the same way, to zero.** The F405's `i2c.c` and
the AT32's are both **100% executed** by the host build: what was left there was
every `reset_keeping_settings()` and `transfer_abort()` - the paths a bus that
stops answering takes - and the models can now stop answering, one status change
at a time, and be held busy by somebody else. Neither file has a line the host
suite does not reach, and neither do the sensor drivers - `ak_imu_bmi270.c`,
whose twelve-byte sample read and whose bench messages had never been executed,
and `ak_baro_dps310.c`, whose SPL06 half (three extra coefficient bytes, a
twelve-bit split, two more terms of the polynomial) had never run either (see
[09-sensors.md](09-sensors.md)). That is what took `src/core/sensors` from 94.7%
to **98.5%**.

**And so did the two USB files, which is the same story with a bigger ending.**
Both were at 19 lines unreached and both are now at **zero**: the FIFO seam
above is what made it possible, and the twelve checks it added are the packets
and interrupts a host causes. `src/arch` is at **94.1%**, from 90.7% when this
map was last read - and the last point of that came from a bug the map had not
been able to see, because the *test* was wrong in the same way as the driver.

**And the ESP32's output layer, which is the same lesson a third time.** It was
at 21 lines unreached - every way the RMT and LEDC can refuse - and the model
now refuses on demand, so all of them run and the file is at **zero**. What made
it worth doing was not the lines but what the last one showed: `ready()` was
answering "is an ESC being driven yet" rather than "are the channels this board
declares up", and a quadrotor - which declares no servos - was called ready on
the strength of its servo timer. See [17-esp32-port.md](17-esp32-port.md);
`src/arch` is at **96.1%** now.

**And both ESP32 sensor buses, to zero as well** - the last two port files with
a gap this machine could close. Both were at 14 and 10 lines unreached: the
*peripheral* refusing rather than a part staying quiet (a bus another driver has
claimed, no device slots left, a transaction the queue will not take), the
re-probe path that removes the old address first, and the barometer *driver's*
own initialisation through this port's bus. That last one is the integration
check the file was missing: the tests probed the part but never opened it, so
the port's `ak_bus_t` delay - the thing a part needs between its reset and its
coefficients - had never been called. One more model fidelity bug fell out of
writing them: `ak_host_spi_set_silent()` returned a failure from the *queue*
call, which is "a peripheral that refuses the transaction", where its own header
documents "a transfer queued and never completed" - so the driver's *collection*
timeout had never run. And one coverage artifact, which is worth naming because
it looked like a real gap: `if (a || b) return b ? 0 : -1;` on one line, with
`-O2` folding the second test into the first, leaves the branch unattributed -
the same behaviour, said as two `if`s.

**And the ARM side of that story, which is the wing's IMU.** The I2C buses have
had a model since 2026-09-16; the *SPI* buses had only the mapped register
block, which is enough for the port's own loop - the arch tests pin that - and
useless for anything that expects a part to answer. So the bus the wing's
ICM-42688-P sits on had never been driven anywhere: the board's read, its write,
its burst and its delay were compiled and never called, and the SPI loopback's
report - the check a person runs with one jumper wire - had only ever printed
its *timeout*. Both ARM SPI ports now have the seam the I2C ports have, with a
modelled device behind it (`tests/host_spi_model.c`); the public entry delegates
to the model while the register loop keeps a name of its own
(`ak_spi_transfer_loop`), so the map does not lose the loop to the seam.

The wing's board is now checked the way it will be flown: the ICM opens through
the board's own bus and returns a sample in g and radians a second, the DPS310
opens through the board's *other* bus with the driver's waits landing on the
board's own delay, and the loopback says both of its sentences. That took
`src/boards` from 90.8% to **95.5%** and left the two SPI ports at 7 and 0 lines
unreached.

**And the coverage map said "7" on the port that is on the bench.** Keeping the
register loop under its own name is what was supposed to stop the seam from
hiding it, and on the AT32 it did - that test has driven its loop through the
mapped block since it was written. The F405's test only ever ran the loop down
its *failure* path ("a transfer on a bus that never reports a flag gives up"),
because on that port every check went through the modelled entry point. So the
loop a target actually runs for the bench board's sensor bus had never
completed a transfer anywhere. It has now, with the flags standing in for the
part - which is all the loop needs from a bus: that the register it wrote is
the one it reads back, which is exactly what the MOSI-to-MISO jumper in the
bring-up checklist gives a person. Four checks, and the two ports finally
agree: 0 and 0 lines unreached.

One assumption is stated rather than hidden: the model cannot see the chip
select (a GPIO the driver sets directly), so it classifies a transfer by its
*shape* - one byte is an address, two or more are an address and data, and what
follows a one-byte address is more data for it. That is the byte stream every
driver in `src/core/sensors` and both boards produce.

**And the ESP32's last two port files, to zero.** `net.c` (9 lines) and
`uart.c` (8) were the other role's failures - a station that cannot create a
netif, an access point whose radio will not start, a listener another service
already holds, an accept that fails while the loop is running - plus the GPS
port's "which UART did not come up" line and its drop counter. The net port
needed the same host-only forget seam its SPI/I2C/ADC siblings have, because it
keeps process-global state; without it a test that wanted a *failed* start after
a successful one was checking the port's memory of the first attempt, which is
exactly how the first version of those checks behaved (green when run early, red
when run late). `src/arch` is at **97.8%**.

**And the console's own refusals, which the map had been pointing at all
along.** After the converters, the largest gap left in the firmware was
`main.c` - 238 lines - and most of it was the console's answers to arguments a
person gets wrong. That is the same class as everything above: a path nobody has
run. The third pass of `make command-check` is what runs them, and it took
`main.c` from 83.2% to **86.0%**.

**And the converters, which are the last of this kind.** `ak_adc_read_counts()`
clears a status register, starts a conversion and polls for the end flag - and
against a page of memory the third step can never happen, because a write to a
register does not make a conversion finish. So the *success* path of the flight
pack's converter had never run anywhere: every host test could only reach the
timeout, and the pack's voltage was one of the things this repository listed as
"the bench will tell us". Both converters now have a model of that one flag
(`tests/host_adc_model.c`) and nothing else - every other register is still the
mapped block, so the checks that read back the prescaler, the sample time and
the sequence keep reading the real writes. What that bought: the divider's
counts turn into a voltage the core would agree with, a converter that never
finishes is a timeout rather than the last value in the register, and the
console says which of the two it is. `src/boards` is at **96.3%**, from 90.8%
when this section was last read, and `src/arch` at **97.8%**.

What is left in the F405's board file is a *configuration* fact rather than a
gap: its divider code is compiled in both builds and executed in the fitted one,
and the coverage map is the bare build. This sentence used to add that the fitted
pass "runs the same checks and prints 2001 with them in" - the 2001 was a stale
count and the claim was false: the fitted pass ran the *bare* checks, and the
fitted half of the code it was supposed to cover had never been executed. That is
fixed as of 2026-09-29 and measured below, under *And the fitted board*.

The board layer went from **30.6% to 90.8%** while this section was being
written, which is the clearest statement of what the map is for: the files with
the most unreached lines were not the awkward ones, they were the *glue*. What
closed the last of it was running the hooks a person at a bench reaches for -
the line the banner prints about the crystal, the LED the tell-tale blinks, the
region the long log survives a reset in, the two ways out of the firmware and
the four answers a board with no network gives - in the two board tests, through
the same `ak_test_f405_crystal()` / `ak_test_at32_clock_up()` setup the arch
tests use, so the two cannot disagree about what a part with a working crystal
answers.

### And every command, typed at the firmware once

`docs_check.py` asks the firmware for its command list and looks for each name
in the pages; that says a command is *documented*. Nothing said a command
*answers* - the sessions type the ones they need, and a person at a bench types
all of them, including the ones no session has ever used. `make command-check`
(`tools/command_smoke.py`) is that: it starts the simulator's console, asks for
`help`, and types every command on the list, waiting for the prompt after each
one.

```text
make command-check     74 commands typed, 0 did not answer
```

What it catches is the failure a bench notices first: a command that faults, or
hangs, or answers nothing at all - which on a board is a console that has stopped
talking, and on this machine is a test that never comes back. It is a smoke
test, not a state machine: a command that needs arguments answers with its
usage, and a command that does something is left to do it, because the question
is the *next prompt*.

The commands that *have* a body behind their arguments are typed a second time
with a representative one - `get rate_kp_roll`, `set rate_kp_roll 0.3`,
`calibrate vbat 12.6`, `mission add 52.1 4.9`, `log reset`, `home clear` - which
is what took the count from 26 to 32. Those are the invocations a person types
at a bench (`log reset` is in the bring-up checklist), and a body that faults is
the same failure as a command that hangs.

**And a third pass types nonsense at it** (2026-09-18), because that is what a
person at a bench does while learning the console: a name that does not exist, a
value out of range, a word where a number belongs, an extra argument, a command
that is not a command. Twenty-seven of those, and the same two questions asked -
did the prompt come back, and is there a fault recorded - which is how this
found that `main.c` had 39 lines of refusals no test had ever reached:

```text
set rate_kp_roll abc       not a number: abc
set rate_kp_roll 99999     out of range 0.000..3.000
calibrate accel 9          calibrate accel: '9' is not a face
mission add abc def        mission: 'abc, def' is not a position
```

Nothing in the pass is destructive - `reboot`, `dfu`, `defaults`, `save` and
`log reset` are deliberately absent, because a nonsense argument to those either
does nothing or does the thing, and this pass is about refusals. What it buys is
not the coverage: it is that the console survives being typed at by somebody who
is guessing, which is the same failure the two passes above exist for - a
console that has stopped talking - and it is where the argument parsing lives,
where a board dies on a number nobody meant.

**And the middle pass grew a state machine's worth of order** (2026-09-18, the
same day's second pass at this): the representative arguments were a *dict*,
which can hold one argument per command, so `mission start` was only ever typed
after a waypoint existed - and the command's other answer, `mission: nothing to
fly`, had never been reached. The list can hold a command more than once now, the
order is the order somebody learns the console in, and the mission's states are
a run through them: start with nothing, add a waypoint, list, start, stop. The
list-full refusal is there too, because a *valid* position is the only way to
reach it - the fifth waypoint is refused by the parameter table rather than by
the parser that reads the degrees.

Two things it taught on the way in, both about the tool rather than the
firmware: a first draft waited for two prompts per command (the boot's and the
answer's) when only the first read has two, so every command after the first sat
out a timeout and the run looked like a hang; and looking for the word `fault`
in an answer flags three innocent commands - `defaults`, and the two that say
"no fault recorded" - so the check is now for a *recorded* fault specifically.
It also ends its session by closing the input rather than killing the process,
which is what lets `make coverage` count the lines these commands run.

## And the client against the firmware's own server

`tools/akproto_check.py` drives the protocol *simulator* - a stand-in that
answers with a synthetic table, a synthetic status and a synthetic log. That is
a check of the client and of the protocol's shape, and its expectations are the
stand-in's: it asserts, for instance, that the table has exactly thirty-one
entries, because the stand-in builds its table from the flight core's own
definition and the number is pinned so a change to the core's table is noticed
here. The firmware's own table has **ninety**, and the numbers in that check
were never the aircraft's.

What a board does when the configurator opens its console is the aircraft's own
server: `src/core/main.c`'s protocol callbacks over `src/core/ak_proto.c`, the
real table in `src/core/ak_params.c`, the real logs. Until now that path was
exercised only through the ESP32, over the network, under QEMU - which is slow
and needs ESP-IDF. `tools/akproto_firmware_check.py` (`make
firmware-proto-check`, and part of `make proto-test`) drives it on the console
of `aerialkit-fw-sim`, the same binary the sessions use, in about a second:

| Checked | Why |
| --- | --- |
| hello names the product this build was made as | the one field a client uses to decide it is talking to what it thinks |
| and says how many parameters it has - and every one of those indexes answers with a name and a value | the *firmware's* count, not a number written in the check: this table grows with every feature. What is checked is that the promise is kept, all ninety of it |
| and no two parameters share a name | a duplicate is a configurator row that edits the wrong thing |
| a parameter accepts the value it already has, and refuses one that is out of range with the board's own words | the round trip the configurator's revert and its refusal both go through |
| status parses, an aircraft that is not armed says so, and there are four motor outputs | the fields a window shows first |
| each log reports its count and comes back as a table, with the client's own header | the pull the configurator's log pane does, over the wire a board has |
| and a save goes through the board's own storage | the one write the protocol makes |

Two properties came out of writing it, and both are worth knowing rather than
fixing:

- **The two RAM logs are rings and the flash log is not.** A ring's count is its
  capacity once it is full, and a pull returns exactly that many records - so
  the arithmetic can be checked. The flash log fills, wraps, and is *written
  while it is being read* (the console's own view said "502 records in 4
  sectors" a moment after the protocol said 449), so for that one this checks
  the framing and the header and leaves the count to be a count.
- **Nothing in the server was wrong.** That is the fourth or fifth check this
  week to come back empty, and it is the one with the most reach: it is the
  path a configurator takes when it is plugged into a real board.

### The file that had never run: the board's own

Every simulated session runs the *simulator's* board, and `tests/test_arch.c`
drives the port's drivers directly - so the layer that decides which USART the
receiver is on, which pin selects the IMU, which SPI port the sensors hang off
and what "no divider fitted" reads as had never been executed by anything on
this machine. It is also the first layer a bench exercises.

`tests/test_board_f405.c` runs it now, starting with `ak_board_init()` - the
exact sequence the target runs at boot, in the same order - and checking what it
wired, against the same mapped register page as everything else:

| Checked, by running it | Why it is the check |
| --- | --- |
| the console's divisor is the one its port and rate need | a board on the wrong USART, or at the wrong rate, is a bench where nothing anybody types is heard. This is the *board's* half of the divisor bug the port had |
| the receiver's and the GPS's ports are each their own, at their own rates | the receiver's frames and the GPS's are the same kind of bytes from two different places, and a board that swapped them is a receiver that never arms |
| what arrives at the receiver's port is what the board hands over, and an empty ring is not a byte | the ring between the port's interrupt and the core's poll is the board's, and it is where a byte is dropped or duplicated |
| an overflowed ring counts its drops | the flight core reads that count; a ring that silently overwrote is a receiver that stops parsing |
| the IMU has a bus with all four ways of using it, brought up by the board's own init, with the chip select idle high | the select is the difference between one conversation and eight thousand for the BMI270's configuration upload, which is the burst path |
| a bus with nothing on it says so, and leaves the select released | this is every boot on a bare board, and the honest answer is an error rather than a reading |
| the LED is where the board says it is, and lit means lit | this board's LED is active *low*, and the boot's tell-tale patterns are read off it - which is why the board configures it before the first stage is marked: a boot that stopped in its clock still has an instrument |
| with nothing fitted there is no barometer bus, no rangefinder bus and no pack divider, and the report says so | four parts are deliberately absent; a core that believed a floating pin would report a healthy pack |
| and the first target has no network, which is not the same answer as "not connected" | the core opens no protocol on a board that says this |
| the SPI loopback says something definite about the wires | it is the diagnostic a person runs at a bench, and "timed out" is one of its honest answers |

39 checks. Two of them failed first for the test's own reasons, both worth
naming because they are the same trap twice: the divisor rounds to the nearest
(a floor was one low), and a mapped page cannot report the clock *switch-status*
field the hardware sets - so the divisors are checked against the clock the port
reports rather than the 168 MHz the registers were configured for, which is what
`test_arch.c` checks those two registers for instead.

### And the wing's board, which is a different board

`tests/test_board_ghf435.c` is the same file for the third target, and the
reason it is not a copy is the reason a board file exists: this board's console
is **USART1** where the F405's is USART2, its receiver is on **USART2** with its
receive pin on PB0 - a pin the F405 uses for nothing like it - and its sensors
are on **SPI1** and **I2C2** instead of SPI2 and I2C1. It has the IMU, the
barometer and the pack divider *fitted*, so the "nothing is fitted" checks that
are right for the bare dev board would be wrong here, and it has two motors and
two servos on its own timers, which is the shape the wing needs.

39 more checks on the same pattern: the boot sequence, the console's port and
rate, the receiver's and the GPS's ports and rates, the multiplexer register
that says the receiver's receive pin is on the alternate function the board
names - the thing a wrong pin shows up as, a receiver that hears nothing - both
rings and their drop counts, the sensor buses and their pins, the fitted parts,
the absence of a network, the SPI loopback, and the outputs coming up with two
motors and two servos.

The AT32's register names are not the F405's (it sets and clears a pin through
two different registers rather than one bit-set/reset register), and the test
uses that part's - which is a small thing that an abstraction shared between the
two boards would have hidden. One check also had to be rewritten for an honest
reason: the barometer bus's *reads* succeed here, because the AT32's I2C model
is what answers on the host, so what is checked instead is the pin the bus is
on.

### And the second board on the *same* part, which is where the rename earns its keep

`tests/test_board_feather.c` is the fourth of these and the first that shares
its arch layer with another board *in one host binary*: the Adafruit Feather
F405 is an STM32F405 like the WeAct board, so the arch objects are shared and
only `board.o` differs. That is what the Feather's header carries a rename block
for - `#ifdef AK_HOST_FEATHER`, 57 `#define ak_board_x ak_feather_x` lines - one
binary cannot hold two definitions of `ak_board_init()`, and the header is where
the board contract is declared.

The flag has to be on the **test's** object as well as the board's, and that is
not symmetry: it is what makes the test's own calls to `ak_board_init()` and the
rest resolve to *this* board. Without it the test compiles against the unrenamed
names, links against the other board's file, and every assertion in it becomes a
check of the wrong board - which is how the line was found, with **30 failures**
in this file on the first build. The quiet version of that mistake is the one to
be afraid of: a test written against the WeAct board's pin map passes.

54 checks, and five facts differ from the WeAct board's, which is the whole reason
a board file exists:

| Fact | WeAct `AERIALKIT_F405` | Feather `FEATHER_F405` |
| --- | --- | --- |
| console | USART2, on APB1 | **USART6 on PC6/PC7, on APB2** |
| shape | 4 motors, 2 servos | **2 motors, 2 servos** |
| servo bank | TIM2 CH1/CH2, PA0/PA1 | **TIM4 CH3/CH4, PB8/PB9** |
| IMU | BMI270 on SPI2, with a chip select | **LSM6DSO on I2C1, PB6/PB7, at 0x6A** |
| LED | PC13, active *low* | **PC1, active *high*** |

The clock is not a difference, and this table carried it as one until
2026-09-29: **both boards have a 12 MHz crystal.** What differed was a string. The WeAct header assumed 8 MHz, `clk.c` set PLLM from that assumption,
the part ran at 252 MHz with USB at 72, and the banner printed the assumption on
every boot; the Feather's board file inherited the line, and its own header then
repeated it. So the two ports printed the same wrong crystal, each believed the
other was the 8 MHz board, and each board's test asserted its own file. The
Feather's test is what broke the ring - it names 12 MHz and refuses 8 - and the
WeAct's test, its banner fallback and both headers were corrected in the same
pass. A board test that pins a *number* rather than a *pin* is the one that can
stay wrong for as long as every file agrees with the file it was copied from -
which is the whole of the time that matters.

**Corrected 2026-09-30: the ring was broken from outside, and the answer was the
other way round.** This paragraph used to say "both boards have a 12 MHz crystal"
and to name the Feather's test as what broke the ring. The board that was
actually measured - the WeAct, on 2026-09-29, from its own banner - is 8 MHz, and
the 11.95 MHz reading that had turned its header from 8 to 12 was taken on a
*different* F405 (traps 212, 213). So the WeAct's header, its banner fallback
and its test went back to 8, and this paragraph's "what differed was a string" is
half right: what differed was a string, and the string the Feather's file carried
was never a measurement of the Feather either. This board's 12 stands on
Adafruit's variant file, quoted in `src/boards/FEATHER_F405/board.h`, and has
never been measured on the board in hand. The lesson below survives the
correction and is in fact what the correction cost: a board test that pins a
*number* rather than a *pin* can stay wrong for as long as every file agrees with
the file it was copied from. Here it stayed wrong in the files' favour for a day
and was then corrected by a board that was not the one any of them described.

Four things in it are worth having in the record, because each was wrong in the
first draft and each is a property of the *host*, not of the board:

  - **The first half of the name is the host build's.** `AK_BOARD_STR`
    stringifies `AK_BOARD`, and the host build passes `-DAK_BOARD=host` - so
    `ak_board_name()` reads `"host / Adafruit Feather STM32F405 Express"` here.
    The check asserts the breakout half and the absence of `WeAct`.
  - **The modelled I2C bus is reset *before* the board brings the port up**, not
    after: `host_i2c_model_reset()` clears the port's own configuration with it,
    so a reset afterwards undoes `ak_board_imu_init()`. The bus's registers are
    also read through the model and not off the mapped page, because under
    `-DAK_HOST_I2C` `i2c.c` routes them there and the page stays zero.
  - **`host_i2c_last_register()` is the slave's pointer after the transfer, not
    the register the master sent** - a one-byte read of 0x0F leaves it at 0x10,
    which is the same reason the arch test's three-byte read at 0x04 reports 0x07.
  - **The frame counters are process-global and cumulative**, and
    `ak_output_write()` counts a *skipped* frame and returns before the DMA is
    consulted. So the test ends the previous test's burst with
    `DMA1_Stream4_IRQHandler()` and reads both counters as deltas - otherwise a
    frame that was silently dropped would still show the right compare registers,
    and the check would pass for the wrong reason.

### And the *fitted* board, which is compiled *and* run

The F405's barometer and pack divider are bench facts - two `#if` flags that are
0 on the board as it sits - and the workspace harness has built that
configuration beside the default one since the day it was found that the ESP32's
Wi-Fi half had never been compiled at all. Compiling it proves the code exists;
it does not prove the answers are right, and this board's tests are exactly the
place where a fitted part changes an answer (the barometer's bus exists, the
pack's report is about a pack).

So the board's tests are written against those same flags - `#if
AK_BOARD_BARO_FITTED` asserts the bus is there, `#else` that it is not - and
`targets/aerialkit-f405/target.conf` runs `test` as well as `check` in the
fitted configuration.

**For a day the flags reached the image and not the tests**, and this section
said otherwise until it was measured on 2026-09-29. `$(EXTRA_CFLAGS)` is added to
`$(CFLAGS)` and not to `$(HOST_CFLAGS)`, so in the fitted configuration every ARM
object was fitted and every host object was compiled exactly as the bare build
compiles it: the run reported **2,269 checks, 0 failed** - the bare build's
number to the check - and `-DAK_BOARD_VBAT_FITTED` appeared **zero** times in its
compile lines. The `#else` arms above were the only arms the host had ever run,
and each fitted check was the bare answer written twice. 2,269 was not a
coincidence: the fitted run *was* the bare run.

Putting the flags into `$(HOST_CFLAGS)` is not the one-line change it looks like,
and that was measured too: the host binary holds three boards, and a define
saying "the barometer is soldered on" is a lie about two of them. That build
reports `2270 checks, 5 failed`, all five in `tests/test_board_feather.c` - the
board that genuinely carries neither part and has five assertions saying so.

The cure is `$(HOST_FITTED_CFLAGS)`: a variable of its own, scoped by `$(BOARD)`
inside the Makefile, reaching the board object and the board-test object of the
board the image is being built for and no others. `EXTRA_CFLAGS` and
`HOST_FITTED_CFLAGS` are set from **one list** in `target.conf`, because two
records of one fact are two things that can disagree - which is the shape of both
traps 205 and 206, and the reason this one existed at all. Measured on
2026-09-29 across the two configurations:

| | bare | fitted |
|---|---|---|
| the suite | 2,269 checks, 0 failed | **2,270 checks, 0 failed** |
| the barometer's bus | `with nothing fitted, there is no barometer bus` | `with the barometer fitted, its bus is there` |
| the pack divider | `and the pack divider is not fitted`, and two more | `with the divider fitted, the board says so`, and three more |

The count moves by one rather than standing still, because the F405's
`#if AK_BOARD_VBAT_FITTED` arm has one check more than its `#else`. And the scope
is visible in the compile lines rather than inferred from the count: the two
defines are on `src/boards/AERIALKIT_F405/board.o` and
`tests/test_board_f405.o` and on neither of the Feather's two objects, which is
why its five assertions stay green.

# Monitor TODOs

Bring-up of the 6502 emulator uploads through this monitor and `JSR`s the image.
These items are the changes that would shorten that loop. They ship in a new
burn of `startup.s`. The image in the EPROM today is this file, linked at
`$100000`, padded to `EPROM_SIZE` (16384) with the checksum in the last four
bytes.

Measured on the current burn, 9600 8N1, `/dev/ttyACM0`:

- `write <addr>,<byte>` moves one byte per round trip, about 30 bytes/second.
  The 7906-byte BASIC image takes about 260 seconds. The core takes about 140.
- A guest that never `RTS` owns the serial line until the reset button.
  POST then fills `$000400`–`$00FFFF` and leaves the last pattern there, so a
  payload anywhere above the vector page is gone. The BASIC image is at
  `$4400` and the core at `$8000`.
- `checksum` is a `DBRA` loop, so the count argument is one less than the
  number of bytes. `checksum 004400,1EE1` sums the 7906-byte image.
- Success for `write` is the bytes `Ok` LF. The prompt is `"> "` with no
  newline, so it sits in front of the next reply. `checksum` prints eight hex
  digits and a line feed, and no `Ok`.

Latch bit 0 at `$200000` is TX, RX, and the LED. Idle mark is `MOVE.W #1`,
which leaves the LED on. A word read tested with `AND.W #1` is the pin.
`BTST` of the even address reads the other half of the bus and does not see it.

## Load a block

- [x] `load <addr>,<count>` parses the header with the line reader, then
      takes `count` raw bytes into memory. The reply is the byte sum, a line
      feed, and `Ok`.
- [x] Count is the number of bytes. Raw bytes bypass `SERIAL_RX_STRING`. The
      line buffer is 32 bytes and cannot hold a hex record.
- [x] `upload_binary.py` sends `load` and waits for the sum. At 9600 a raw
      stream is about a kilobyte per second, so the BASIC image lands in
      about eight seconds.
- [x] A range that would run into the monitor stack at `$00F000` is `Error`,
      before any raw byte is read.

## Fixed entries for uploaded code

The routine addresses move every time the monitor is edited. Uploaded code
calls the longword table at `$100400`. `entries.inc` is generated from the
vasm listing and the build fails if `ENTRIES` is not that address.

- [x] Longword table at `$100400`, before `START`: transmit, receive, poll,
      timed receive.
- [x] Poll samples with `MOVE.W ($200000)` and `AND.W #1`, returns that bit
      in `D1`, and waits for nothing.
- [x] Timed receive uses the same sample delays as `SERIAL_RX`. On timeout
      or a bad stop bit it returns `-1` in `D1.B`. Blocking `SERIAL_RX` still
      waits, and a bad stop bit returns to the start-bit wait.
      `SERIAL_RX_STRING` already retries on `-1`.
- [x] Transmit keeps its contract: byte in `D1.B`, `D1` preserved,
      `D0`/`D2`–`D4` saved.
- [x] `gen_entries.py` writes `entries.inc` from the listing.

## Keep a payload across reset

The test covers `$000400`–`$00FFFF` and leaves the last pattern in place.
Nothing is restored. There is no untested gap to hide a signature in, so
`hold` has to read it before the fill.

- [x] `hold` writes a two-longword signature at `$000400` (`HOLD` / `SIG!`),
      the first two longs of the system page `$000400`–`$0007FF`. Payloads
      start at `$000800`. The next reset sees the signature before the fill,
      skips the test, and prints `RAM KEPT`. The signature stays until
      `release`.
- [x] With no signature, POST fills RAM as it does now.
- [x] `ramtest` runs the destructive test on demand. It does not return
      through the stack, because the test fills that stack.

## One reply shape

- [ ] `checksum` walks `count` bytes. The count is the byte count, not the
      `DBRA` trip count. A long countdown, so a 16-bit `DBRA` is not the limit.
- [ ] Print the eight hex digits, a line feed, then `Ok` and a line feed.
- [ ] Print the prompt as its own line, `"> "` followed by a line feed, so a
      reader that stops at a line feed takes one reply and nothing from the
      next one.
- [ ] Update `upload_binary.py` for the new checksum count and the prompt line.

## Return from jump

`PARSE_JUMP` does the `JSR`, then `MOVE.W #1,($200000)`, `DELAY1S`, a newline,
and `Ok`. Restoring idle mark is required, because the guest can leave the
shared pin low. The one-second wait is the whole turnaround on a small test.

- [ ] Keep the mark restore. Wait a few bit times, then print `Ok`.

## Line buffer

`BUFFER` is `$000408`, 32 bytes, the first free bytes after the hold marker.
POST has finished by the time the monitor uses it. The string reader writes
the terminating zero at `$000428`.

- [x] Move the buffer into the system page at `$000400`, after the hold
      signature.
- [ ] Give it room for a longer header if `load` needs one. The raw payload
      still does not go through it.

## Help text

- [ ] Document `load` next to the existing commands. `hold`, `release`, and
      `ramtest` are already in the help text.
- [ ] State that bit 0 at `$200000` is TX, RX, and the LED, that idle is mark,
      and that a byte can be received while the monitor is outside `SERIAL_TX`.
      A held Ctrl-C is what a guest break poll can see.
- [ ] State that the pin is bit 0 of a word read. A byte read of the even
      address is the other half of the bus.

A second latch bit for the LED is a board change. With that bit, an error
blink can run while the serial line stays at idle mark. `BLINK_NUMBER_FOREVER`
currently takes the line off 9600.

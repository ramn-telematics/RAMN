# Host tests

Firmware logic tested on a normal machine with `gcc` — no ARM toolchain, no
board, no CAN bus.

```sh
cd firmware/RAMNV1/test/host && make
```

## What it covers

`test_telematics_spi.c` drives the ESP32 → STM32 poll-response decode path in
`ramn_telematics.c`. The assertion surface is `RAMN_FDCAN_SendMessage`:
`fakes.c` records every frame the firmware tries to put on the vehicle bus, so
each test says *given these SPI bytes, this must reach the bus*.

## How it is wired

The test `#include`s `ramn_telematics.c` directly. `ProcessESP32Response` is
`static`, reads a file-static RX buffer, and ends in a HAL call, so it cannot
be reached any other way without first refactoring code that can only be
verified on hardware. The ESP32 repo's `image_stream` host test uses the same
approach.

`stubs/` replaces `main.h`, `ramn_canfd.h`, `ramn_telematics.h` and
`ramn_uart.h` with the minimum the module needs: the FDCAN types and enum
values, the RAMN scalar types, and the FreeRTOS surface. The enum values are
copied from the real STM32L5 HAL headers — if they drift, the tests lie, so
change them only against the HAL source.

`ramn_config.h` is **not** stubbed. The real one is on the include path so CAN
ID definitions cannot drift from the firmware.

## Two rules that keep findings honest

**Fixtures are built, never hand-written.** `build_can_response()` computes
MSGLEN and the checksum. A hand-typed checksum that happens to be wrong is
indistinguishable from a firmware bug, and costs an hour before you notice.

**Known bugs are `CHECK_BUG`, not comments.** A `CHECK_BUG` that fails prints
`KNOWN BUG` and does not fail the build. A `CHECK_BUG` that *passes* **fails
the build**, telling you the defect is fixed and the marker must go. A marker
cannot silently outlive the bug it documents.

## Adding a case

Add a `case_*()` function, call it from `main()`. Use `CHECK` for behaviour
that must hold now, `CHECK_BUG` for a defect you are recording but not fixing
in this change — with a note saying *why* it fails, so the next reader does
not have to re-derive it.

## Bisecting a regression

The same suite can be run against an older revision of the module:

```sh
make control REV=776ee66
```

If a `CHECK_BUG` reports **`NOW PASSING`** at `REV`, the defect did not exist
there — it is a regression introduced later. If it still reports `KNOWN BUG`,
it predates `REV`.

Result for the six defects recorded here, against `776ee66` (the last
revision before image streaming, confirmed working on hardware):

```
15 checks | 0 hard failures | 0 known bugs confirmed | 6 markers to remove
```

All six pass there and fail on `d128bdf`, so all six arrived with the image
streaming change — none is pre-existing.


## ECU A: the image receive path

```sh
make ecua      # just this suite
make           # both
```

`test_screen_image.c` compiles `ramn_screen_image.c` white-box and asserts on
`RAMN_SPI_WriteImageChunk` — exactly the pixel bytes that would reach the
ST7789. That is the only question worth asking about a decoder: for these CAN
frames, what lands on the panel?

Two `CHECK_BUG` markers record the same defect at two scales. ECU A decodes
each 0x301 frame independently, but the ESP32 RLE-encodes the whole image as
one stream and cuts it at fixed offsets, so blocks straddle frame boundaries
and cannot be rejoined. `RLE_Decode` itself is correct — it reproduces every
golden vector — so this is an architecture problem, not a decoder bug. The fix
is to reassemble before decoding (charter A-05); when it lands, both markers
start passing and the harness will fail the build asking for their removal.

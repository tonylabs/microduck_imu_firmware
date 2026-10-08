# microduck IMU firmware (`imu_to_ft` v11)

Firmware for the STM32C011F6U6TR on the microduck IMU board: it reads an
LSM6DSV16XTR over SPI and serves the result on the servo bus as a **Feetech
FT-SCS** device at ID 200.

> **v11 publishes raw samples.** v7 to v10 rotated every gyro, accelerometer
> and quaternion sample by a fixed per-robot mounting trim before putting it on
> the bus, because the board was measured 12.4 degrees nose-down on duck-a4ca
> and the host had no knob for it. The board is mounted straight now, so the
> correction is gone: what this firmware publishes is what the sensor measured.
> `Src/mount.c` and `Inc/mount.h` are deleted, the status flag `0x10`
> (`MOUNT_CORR`) is retired, and address 40 no longer serves a correction
> quaternion. The firmware is 5 640 bytes smaller for it - the quaternion maths
> was the only thing in the image pulling in software floating point. If a
> crooked board ever turns up again, the whole module is in git at `v7`.
>
> **v10 changed the protocol.** Up to v9 this board was a Dynamixel Protocol
> 2.0 device serving its block at address 124, because the servos were XL330s.
> They are now **FEETECH HD1910**, which speak FT-SCS, so the board does too.
> What moved: the wire format, the address of the sensor block (124 -> **56**),
> the identity registers live. What did not move: the ID (200), the 20-byte
> block layout, the pinout, and every line of the sensor path.
>
> **The bus now runs at 500 kbps**, not the 1 Mbps it did under Dynamixel and
> not the HD1910's factory default. The servos have to agree: their EEPROM
> baud rate register (address 6) must be set to code **1**, and the host must
> open the port at 500000.

---

## How the Radxa ZERO 3W actually gets IMU data

**It never talks to the LSM6DSV16X.** There is no I2C or SPI link between the
SBC and the sensor. The path is:

```
Radxa ZERO 3W  --/dev/ttyS2, FT-SCS, 500 kbps--+-- ID 200  this board --SPI--> LSM6DSV16XTR
   robotd                                      +-- ID  ..  HD1910 servo
                                               +-- ... 15 servos total
```

Per control tick (50 Hz) the host issues **one** `SYNC_READ` covering this board
and all fifteen servos, at **address 56, length 12**. That address is not
arbitrary: on an HD1910 it is the start of `present_position` /
`present_speed` / `present_load` / `present_voltage` / `present_temperature`,
and 12 bytes covers the lot - so the IMU can ride the same transaction instead
of being a separate poll. This board is listed **first** in the ID list, so it
answers before the servo burst.

Address 56 is to FT-SCS exactly what 124 was to Protocol 2.0: the first
feedback register, and the one the per-tick read already has to start at.

Source of truth for the protocol: `rust_servocom` -
`src/feetech_protocol/feetech.rs` for the framing and the checksum (its unit
tests are the vectors this firmware is checked against), `docs/FEETECH.md` for
the memory table, `src/servo/feetech/hd1910.rs` for the servos' side of it.

The host does no sensor fusion. The LSM6DSV16X's **SFLP** (sensor fusion
low-power) block produces a game-rotation quaternion and estimates its own gyro
bias; the host only rotates it into the trunk frame, derives gravity, and runs
a 3-sample median for spike rejection.

### The 12 bytes the tick consumes

| bytes | contents |
|---|---|
| 0..6 | gyro x/y/z, `i16` LE raw counts, +/-500 dps (17.5 mdps/LSB) |
| 6..12 | SFLP game rotation vector x/y/z as IEEE binary16 |

`w` is **not** transmitted - the host reconstructs it as
`sqrt(1 - x^2 - y^2 - z^2)`. All-zero quaternion bytes mean *SFLP has not
written its table yet*, and the host should hold its last good value rather
than snapping to identity (snapping would tell fall detection an
unknown-orientation robot is upright). This firmware therefore publishes zeros
until the first quaternion actually arrives, roughly 0.7 s after power-up.

A block identical to its predecessor means "the read succeeded but the board
handed back the same sample". The byte-18 sample counter in the full block
below makes that fault unambiguous.

### The full 20-byte block this firmware serves at 56

| offset | bytes | contents |
|---|---|---|
| 0 | 6 | gyro x/y/z, `i16` LE, +/-500 dps |
| 6 | 6 | SFLP quaternion x/y/z, binary16 LE |
| 12 | 6 | accelerometer x/y/z, `i16` LE, +/-4 g (0.122 mg/LSB) |
| 18 | 1 | sample counter, wraps at 256 |
| 19 | 1 | status flags |

Status flags: `0x01` IMU present (WHO_AM_I matched), `0x02` SFLP ready,
`0x04` FIFO overran since last publish, `0x08` SPI transfer timed out.

`0x10` was `MOUNT_CORR` up to v10 and is **retired, not reused**: a host that
still knows the old meaning would read a recycled bit as a correction being
applied.

Samples go on the bus exactly as the LSM6DSV16X measured them, in the sensor
frame. Nothing is rotated, scaled or filtered on this board - rotating into the
trunk frame is the host's job, and it is the only place that knows how the
board is mounted.

### The rest of the register file

Addresses **0-8 are the real FT-SCS header**, because that is what a bus scan
reads and it costs nine bytes to be scannable. Everything above that is this
board's own: an address that means `min_angle_limit` or `present_speed` on an
HD1910 means whatever this table says it means here. **This is not a servo
control table** and nothing tries to make it look like one.

| addr | size | value |
|---|---|---|
| 0 | 1 | firmware major version (11) |
| 1 | 1 | firmware minor version |
| 2 | 1 | `END` = 0, i.e. little-endian, as every FT-SCS device |
| 3 | 2 | model, `u16` LE = `0xC011` - where `servocom`'s scanner looks |
| 5 | 1 | ID (200) |
| 6 | 1 | baud rate code - **1 = 500 kbps** in Feetech's numbering (0 is 1 Mbps, 4 is 115200) |
| 7 | 1 | return delay (0) |
| 8 | 1 | response status level (1 = answer every instruction) |
| 16 | 22 | diagnostic counters (v4), eleven `u16` LE, wrapping, never cleared - take differences: requests that reached the parser, checksum failures, mid-packet gap resets, receiver FE/NE, `SYNC_READ`s answered, replies framed, replies completed (TC), largest mid-packet gap tolerated (us, v5), the gap that caused the last reset and the byte it struck at, receiver overruns (v6). `read 200 16 22` after a run says where a request the host never got an answer to actually died |
| 56 | 20 | the sensor block above |

Gaps read as zero, as does anything past the end of the table - see *Reads
never fail* below.

`0xC011` is far outside the range any real Feetech servo reports (the highest
registered in `servocom` is the SM40BL at 10248), so a scan cannot mistake this
board for a servo. It is also what this board reported as its Dynamixel model
number, so a board stays identifiable across the protocol change. `servocom`'s
`ServoKind` has no entry for it, so `scan --protocol feetech` prints *"Found
motor with id 200 with ... (raw: 49169)"* - correct, just unnamed. Adding it to
`src/servo/mod.rs` would give it a name.

---

## The wire format, and what it costs

FT-SCS. Both directions are

```
FF FF  ID  LEN  (INST | ERR)  PARAM...  CHK
```

`LEN` counts everything after itself including `CHK`, so `LEN = nparams + 2`.
`CHK` is the bitwise complement of the 8-bit sum of `ID`, `LEN`, the
instruction-or-error byte and the parameters; the two header bytes are not
summed. Instruction packets carry an instruction byte in that slot, status
packets carry an error byte.

Three differences from Protocol 2.0 matter, and all three show up in the code:

* **There is no byte stuffing.** `FF FF` can and does appear inside a payload -
  a gyro reading of -1 is two of them - so the header is *not* a reliable
  resynchronisation point. The inter-byte gap rule is what recovers the stream,
  and under FT-SCS it is the only thing that can. (Under Protocol 2.0 it was
  merely the better of two options.) A run of three or more `FF` is handled by
  staying in the ID state: no device has ID 255, so the header must have been
  the *last* two.
* **`LEN` is one byte**, so a packet is at most 259 bytes and a read at most
  253. The buffers are sized for the protocol maximum rather than for what this
  board expects, so no legal request can overrun them or be answered short.
* **The checksum is an 8-bit sum**, not a CRC-16. It catches a mangled byte but
  not every transposition - nothing to be done about that, it is what the
  servos on the same wire use. The upside is that the 512-byte CRC table v4
  needed to keep the ISR affordable is gone; a wrapping sum is a few cycles a
  byte.

Supported instructions: `PING` (0x01), `READ` (0x02), `WRITE` (0x03),
`REG_WRITE` (0x04), `ACTION` (0x05), `RESET` (0x06), `REBOOT` (0x08),
`SYNC_READ` (0x82). Everything that would change state is acknowledged and
ignored - identity is compiled in and the sensor block is an output - rather
than refused, because on FT-SCS the only way to refuse is silence, and silence
is indistinguishable from a dead board. `SYNC_WRITE` (0x83) is dropped at the
instruction byte, before it costs a checksum: on this bus it is the host's
per-tick broadcast of goal positions, and it is never ours.

**Broadcast `PING` is deliberately unanswered.** A Feetech bus is scanned by
pinging each ID in turn - `servocom`'s scanner does exactly that - so answering
the broadcast buys nothing and would put this board's reply on the wire at the
same moment as fifteen servos', making the scan garbage for all sixteen.

### Reads never fail

FT-SCS has no way to say *"that address does not exist"*: the error byte is
eight hardware-fault bits and nothing else. So a read of an address this board
does not have returns **zeros, of exactly the length asked for**, and a read
running off the end of the table is zero-filled rather than truncated.

That is not laziness, it is the only safe answer on a shared bus. The host
reads status packets back in ID order; a missing or short reply does not cost
one device's data, it costs every device queued behind us too. A reply of the
right length always keeps the bus in step. The Feetech memory table has
`Undefined` rows of its own, so zero is also what a real servo does here.

The one request that gets no answer at all is a read longer than 253 bytes,
which `LEN` cannot describe - there is no correctly framed packet to send.

### The error byte

Every FT-SCS status packet carries one, and a Feetech-aware host decodes it
without being told anything about this board. So when the IMU is not answering
- a bad `WHO_AM_I` at boot, or SPI timing out since - this firmware sets
**bit 1, the magnetic-encoder bit**: *"the position sensor is not answering"*,
which is the closest true statement FT-SCS can make about an IMU that is not
answering either. A dead board therefore shows up as an error in generic
tooling, with no IMU-specific host code.

The authoritative answer is still byte 19 of the sensor block. Set
`FT_ERR_MIRRORS_FAULTS` to 0 in `ft_slave.h` for a board that must never look
faulted to generic tooling.

---

## Pin assignment

Two board revisions exist. `BOARD_REV` in `Inc/board.h` selects one (default
**2**); `-DBOARD_REV=1` in the build settings overrides it. Nothing in the `.c`
files names a pin directly — every pin goes through `board.h`.

### Rev 2 — buffered bus (`hardware-bdx/microduck-imu/1.1`, schematic 2026-09-12)

STM32C011F6U6TR (U1), UFQFPN20.

| Pin | Port | Net | Function |
|---|---|---|---|
| 2 | VDD/VDDA | +3.3V | AP2210K LDO, fed from the MP2225 5 V buck (VIN on J2) |
| 3 | VSS/VSSA | GND | |
| 4 | PF2/NRST | RESET | 10 k pull-up + 100 nF, TC2030 pin 3 |
| 9 | **PA4** | SPI_CS | → IMU CS (pin 12), 10 k pull-up — software NSS |
| 10 | **PA5** | SPI_SCK | → IMU SCL/SCK (pin 13), AF0 = SPI1_SCK |
| 11 | **PA6** | SPI_MISO | ← IMU SDO/SA0 (pin 1), AF0 = SPI1_MISO |
| 12 | **PA7** | SPI_MOSI | → IMU SDA/SDI (pin 14), AF0 = SPI1_MOSI |
| 13 | **PA8** | IMU_INT | ← LSM6DSV16X INT1 (pin 4) |
| 14 | **PA11** | LED | D2 anode on +3.3V through R9 1 k — **LOW = lit**. AF2 = TIM1_CH4 |
| 15 | **PA12** | DXL_DIR | → U5 OE#, R7 10 k pull-up. AF1 = USART1_DE |
| 16 | PA13 | SWDIO | TC2030 pin 2 |
| 17 | PA14/BOOT0 | SWCLK | TC2030 pin 4, 10 k pull-down on BOOT0 |
| 18 | **PB6** | USART1_TX | → U5 A. AF0 = USART1_TX |
| 19 | **PB7** | DXL_DATA | ← U5 Y and J1/J4/J5 pin 3. AF0 = USART1_RX |

Unconnected: PA0–PA3, PC14, PC15. Pins 14 and 15 are PA11/PA12 by default and
become PA9/PA10 only if `SYSCFG_CFGR1.PA11_RMP` / `PA12_RMP` are set; the
firmware never sets them, and must not — the default mapping is the one that
carries `TIM1_CH4` and `USART1_DE`.

J1, J4 and J5 are identical 3-pin 2.0 mm (GND, +5V, DXL_DATA) for
daisy-chaining. J2 is the 2-pin VIN input.

#### How the bus is shared on Rev 2

U5, an **SN74LVC1G125** single-gate buffer, is the transmit/receive switch:

```
PB6 USART1_TX ──► A   Y ──┬── DXL_DATA ── J1/J4/J5
PA12 DXL_DIR  ──► OE#     └── PB7 USART1_RX
                  (R7 10 k pull-up)
```

* **Receive:** OE# high, Y is high-impedance, the bus belongs to whoever is
  talking, and PB7 reads it as an ordinary `USART1_RX`. R7 guarantees this
  state from power-on until the firmware says otherwise, and until then every
  MCU pin is high-impedance.
* **Transmit:** OE# low for exactly the duration of the status packet, and Y
  drives the bus from PB6.

DXL_DIR is not toggled by software. PA12 is `USART1_DE`, the USART's own
driver-enable output: with `CR3.DEM` set it asserts DE ahead of every start bit
and releases it after the last stop bit, timed in sixteenths of a bit by
`DEAT`/`DEDT` (4/16, i.e. 500 ns at 500 kbps). `CR3.DEP` inverts it to match the
active-low OE#. The firmware therefore cannot release the bus late, or early.

Because PB7 sits on the bus, everything this board transmits comes straight
back into its own receiver. `RE` is cleared for the duration of a transmission
and the RX register is flushed at `TC`, so our own status packets never reach
the parser.

PB6 and PA12 are switched into alternate-function mode **only after** USART1
is clocked, configured and enabled. Put into AF earlier, the pins would be
driven by an unclocked peripheral and could pull OE# low — U5 enabled, the bus
driven — for the few microseconds until the USART wakes up.

### Rev 1 — direct bus (schematic 2026-09-05)

| Pin | Port | Net | Function |
|---|---|---|---|
| 5 | **PA0** | IMU_INT | ← LSM6DSV16X INT1 (pin 4) |
| 9–12 | PA4–PA7 | SPI | as Rev 2 |
| 13 | **PA8** | LED | D2 anode on +3.3V through R2 1 k — **LOW = lit**. AF13 = TIM14_CH1 |
| 16, 17 | PA13, PA14 | SWD | as Rev 2 |
| 18 | **PB6** | DATA_TX | servo bus data, J2/J3 pin 3, **direct** |

Unconnected: PA1, PA2, PA3, PA9/PA11, PA10/PA12, PB7, PC14, PC15.

DATA_TX runs from PB6 straight to the connectors — **no bus transceiver, no
direction pin**. PB6 is `USART1_TX`; `USART1_RX` cannot be routed to it. So:

* USART1 runs in **single-wire half-duplex** (`CR3.HDSEL`), which feeds the
  receiver from the TX pin.
* HDSEL alone would leave the transmitter idling the pin high and fighting every
  servo. So PB6 sits in alternate-function **open-drain** while receiving (an
  idle-high transmitter simply releases it) and flips to **push-pull** for the
  ~230 µs it takes to send a status packet. Open-drain throughout would be
  simpler, but the pull-up alone cannot make the rising edges at this rate.

On both revisions the IMU's SDx and SCx are tied to GND, and INT2 / OCS_AUX /
SDO_AUX are open — plain 4-wire SPI with no OIS chain.

### Clock — and the absence of a crystal

**There is no crystal on this board, and none is needed.** PC14/OSC_IN and
PC15/OSC_OUT are unconnected (see the NC list above), and nothing in the
firmware ever enables HSE. Everything runs from **HSI48**, the on-chip 48 MHz RC
oscillator, which is running out of reset.

`clock_init()` in `main.c` touches three things: one flash wait state, `HSIDIV`
to 000, and `SW` to HSISYS. That is the whole clock tree. The divider is the
part that matters — HSIDIV comes up at 0b010, so the reset SYSCLK is 12 MHz, and
at 12 MHz a megabaud USART is unreachable with 16x oversampling (`USARTDIV`
must be at least 16), and the board was a 1 Mbps device when this was written.
At 48 MHz `BRR` lands on exactly 96 for the 500 kbps the bus now runs at - and
on exactly 48 at 1 Mbps - so the divisor carries no rounding error either way.

So the entire bus-rate error budget is HSI48 trim. From DS13866 Rev 5, Table 40:

| | |
|---|---|
| f at V_DD = 3.0 V, T_A = 30 °C | 47.92–48.40 MHz (−0.17 % / +0.83 %) |
| drift, T_A = 0 to 85 °C | ±1 % |
| drift, T_A = −40 to 125 °C | −2.5 % / +2 % |

An 8N1 link tolerates roughly ±2 % end to end, and everything else on the
bus — the HD1910s, a USB-TTL adapter — is crystal-clocked, so nearly all of that budget is
ours to spend. ±1 % across 0–85 °C leaves real margin; the −40 °C corner is
where it would get tight, which is not where this robot lives. If a particular
board ever does turn out to be marginal, `HSITRIM` in `RCC->ICSCR` is the knob,
at about 0.3 % per step.

---

## Building

This is a **bare CMSIS project** — no HAL, no CubeMX, no `.ioc`. It was built
from the STM32CubeIDE *Empty Project* template for STM32C011F6Ux and then given
the CMSIS headers it needs.

That is not a compromise. The firmware configures SPI1, USART1, TIM3 and every
GPIO at the register level regardless, so the HAL was only ever supplying
`HAL_Init()`, `SystemClock_Config()` and `HAL_GetTick()` — the forty lines now
at the top of `main.c`. Dropping it took the image from 9 240 bytes to 3 512.

Open the project in STM32CubeIDE and press **Ctrl+B**. There is nothing to
generate first.

What was added to the Empty template:

* `Drivers/CMSIS/` — device headers (`stm32c0xx.h`, `stm32c011xx.h`,
  `system_stm32c0xx.h`) and the Cortex-M0+ core headers, from
  STM32Cube FW_C0 V1.1.0.
* `Src/system_stm32c0xx.c` — ST's `SystemInit()`, which `Startup/` calls.
* Two include paths in `.cproject` (`../Drivers/CMSIS/Include` and
  `../Drivers/CMSIS/Device/ST/STM32C0xx/Include`) and the `STM32C011xx` define,
  in both the Debug and Release configurations.

`board.h` also defines `STM32C011xx` under an `#ifndef`, so the firmware still
compiles if the build setting is ever lost.

The pin map is not in a GUI anywhere — it lives in `Inc/board.h` and in the
table above. That is the one thing given up by not using CubeMX, and on a board
whose peripherals are all configured by hand it costs little.

---

## Source layout

| File | What it does |
|---|---|
| `Inc/board.h` | pin map per board revision, transcribed from the schematics; `BOARD_REV` lives here |
| `Src/main.c` | clock, SysTick, main loop |
| `Src/lsm6dsv16x.c` | SPI1 at register level; chip config; FIFO drain for SFLP |
| `Src/ft_slave.c` | FT-SCS slave on USART1, all in the ISR; hardware DE (Rev 2) or HDSEL + open-drain (Rev 1) |
| `Src/app.c` | poll loop, sensor block assembly, LED |

From the CubeIDE template and not worth reading: `Src/syscalls.c`,
`Src/sysmem.c`, `Src/system_stm32c0xx.c`, `Startup/`, `STM32C011F6UX_FLASH.ld`.

### IMU configuration

Software reset, then `WHO_AM_I` must read `0x70`. I²C and I3C disabled
(`IF_CFG.I2C_I3C_disable`), `BDU` and `IF_INC` on. Accelerometer and gyroscope
both at **120 Hz high-performance**, ±4 g and ±500 dps, gyro LPF1 enabled.

The quaternion is only reachable through the FIFO — there is no output register
for it — so the FIFO runs in continuous mode batching **nothing but** the SFLP
game rotation vector (tag `0x13`) at 120 Hz. Gyro and accelerometer are read
straight from `OUTX_L_G` / `OUTX_L_A`, which keeps the FIFO one word deep and
means a late poll cannot make the quaternion lag the rates.

### Timing

Rates, and how they line up with the host:

| | rate | |
|---|---|---|
| `robotd` control loop | 50 Hz | `microduck/robotd`, and `CONTROL_DT = 0.02` in `microduck_rl` |
| this board publishes | 500 Hz | measured 499.1 Hz |
| LSM6DSV16X accel/gyro/SFLP | 120 Hz | `CTRL1`/`CTRL2`/`SFLP_ODR` |

The publish rate is ten times the host's read rate, so it is never the limit -
**the IMU's 120 Hz ODR is.** Worst-case sample age at a host read is one ODR
period plus one poll period, 8.3 + 2 = ~10 ms, against a 20 ms tick. `robotd`
already treats the observation as one full tick old by construction ("at 50 Hz
the policy is trained on data of exactly this age"), so that fits with room.

120 Hz also keeps a host-side stale-sample detector quiet: at 2.4 IMU samples
per tick every host read gets a fresh block, so a run of identical blocks
cannot build up. An SFLP ODR of 15 or 30 Hz would trip one constantly.

* Poll: every 2 ms (500 Hz), four SPI transactions of ~20 µs each.
* Bus: the whole protocol runs in `USART1_IRQHandler`, so a request is answered
  as soon as its last CRC byte lands. `return_delay_time` is 0, matching what
  the host asserts of the servos.
* The USART runs in **FIFO mode** (v3): eight bytes of receive buffering, so
  the ISR may be away for eight byte times - 160 µs at 500 kbps - before a byte is lost, against one byte time
  with the bare `RDR`. Both CRCs — the check on a request and the signature on
  the reply — are table-driven, about 2 µs each. v2 computed them bitwise
  (~25 µs each) with no FIFO, and also buffered and CRC-checked the host's
  88-byte `SYNC_WRITE` every tick (~90 µs deaf); measured on the bus, v2
  answered about 85 % of `SYNC_READ`s and was silent for the rest — and because
  this board is first in the ID list, a silent IMU keeps every servo behind it
  silent too, so each miss cost the host a whole tick plus its read timeout.
* Foreign packets — about fifteen servo status packets per tick — are dropped at
  the ID byte before any CRC is computed. A broadcast whose instruction is not
  `PING` or `SYNC_READ` (the `SYNC_WRITE` above) is dropped at the instruction
  byte, for the same reason.
* A gap of more than 2 ms between bytes abandons a part-received packet (v5;
  it was 100 µs, and the Radxa's UART — which refills its transmit FIFO from an
  interrupt that is routinely 100–300 µs late — was tripping it on 3–9 % of
  requests, every one of them otherwise clean). Bytes inside a packet are 10 µs
  apart and the next request is a 20 ms tick away, so a real gap still means
  the rest is not coming;
  without this, a truncated packet keeps counting its `LEN` down into the *next*
  packet's bytes and costs a second transaction too — a whole missed control
  tick. Resynchronising on the header instead is tempting, since byte stuffing
  keeps `FF FF FD 00` out of any payload — but not out of the two CRC bytes,
  which are not stuffed.
* TIM3 free-runs at 1 MHz. It is used for one thing: holding off when this board
  is *not* first in a `SYNC_READ` ID list. The host always lists it first, so in
  practice that wait is zero.

### What has and has not been verified

The protocol is checked by a **23-case round-trip test built from the shipping
`Src/ft_slave.c`** (compiled with `-Dstatic=` so the parser, the framer and the
table are reachable; requests go straight into `rx_byte()` and replies are read
back out of `s_tx`, with the USART interrupt masked). It is built for the
STM32C011 with the firmware's own compiler and flags, and run **both** under a
Cortex-M0+ emulator and **on the board itself**, the results read out over SWD.
All 23 pass in both places. What it covers:

* **The checksum**, against all six packet vectors in `rust_servocom`'s own
  `feetech.rs` unit tests - `ping(1)`, `read(1,0x2B,1)`, `write(10,24,[1])`,
  `sync_read([11,12],30,2)` and two status packets - each also recomputed by an
  independent implementation inside the test, so a shared bug cannot make one
  pass.
* **`PING`**: answered unicast with a payload-free status packet; silent on the
  broadcast address and on someone else's ID.
* **`READ`** of the identity header a scan uses: address 3 returns
  `11 C0` (model `0xC011` little-endian), address 5 returns ID 200 and baud
  code 0.
* **The per-tick `SYNC_READ(56, 12)`** with this board first in the ID list,
  byte for byte against a hand-computed frame - with a sensor block containing
  `FF FF FD 00`, and an explicit check that those four bytes come back
  **un-stuffed**. That is the one place a Protocol 2.0 habit would silently
  corrupt an FT-SCS payload.
* **`SYNC_READ` this board is not listed in**: silent.
* **A wrong checksum**: silent.
* **A read running off the end of the table**: right length, zero-filled.
* **`WRITE`**: acknowledged, and the table is unchanged afterwards.
* **A broadcast `SYNC_WRITE`**: dropped - and the `PING` immediately after it
  is still parsed, which is the regression that matters, since dropping a
  packet mid-stream is exactly how a parser loses the next one too.
* **Four `0xFF` then a `PING`**: still parsed.
* **The error byte** following the fault flag in both directions.

Also verified off-target:

* **It builds and links** with the GCC 14.3 that ships with CubeIDE, clean
  under `-Wall -Wextra`, for both board revisions.

  | config | flash | RAM |
  |---|---|---|
  | Rev 2, Release (`-Os`) | 3 820 / 32 768 - **12 %** | 3 788 / 6 144 - 62 % |
  | Rev 1, Release (`-Os`) | 3 728 / 32 768 - 11 % | same |
  | Rev 2, Debug (`-O0 -g3`) | 7 344 / 32 768 - 22 % | same |

  RAM is 2 252 bytes of `.bss` plus the linker script's 0x200 heap and 0x400
  stack. `.bss` grew about 300 bytes over v9: the receive and transmit buffers
  are now sized for the FT-SCS maximum rather than for the longest packet this
  board expects. Flash fell by 5 640 bytes in v11 - `mount.c` did quaternion
  arithmetic in `float`, and on a Cortex-M0+ with no FPU that pulled the whole
  software floating-point library into the image. Nothing else here uses it.

Verified **on hardware** (Rev 2 board, STLINK-V3MINIE over SWD, target at
3.21 V, device ID 0x443), 2026-10-04:

* Flashed and read back verified; the firmware runs, SysTick counts past 21 s.
* The FT-SCS header reads back exactly as intended: `fw 11.0`, `END 0`,
  `model 0xC011`, `id 200`, `baud_code 0`, `delay 0`, `response level 1`.
* `s_imu_present = 1` - the LSM6DSV16X answered `WHO_AM_I` over SPI1.
* Status byte `0x03` = `IMU_PRESENT | SFLP_READY`, with `FIFO_OVERRUN` and
  `SPI_ERROR` clear. Error byte `0x00`.
* **The published block is byte-for-byte the raw sample** - the 18 data bytes
  at 56 compared against `s_gyro`/`s_quat`/`s_acc` read straight out of
  `s_sample`, equal on every one of three reads. That is the v11 change, and it
  is the only way to tell from outside that no rotation is being applied.
* Addresses 40-47, which served the mounting correction up to v10, read back
  all zero.
* Accelerometer magnitude **0.9988 / 1.0046 / 1.0051 g** across three reads.
  This is the load-bearing number: it confirms the +/-4 g full scale and the
  0.122 mg/LSB conversion together, because only the right pair gives 1 g.
* Gyroscope within a few dps of zero on a stationary bench.
* SFLP quaternion non-zero and normalised.
* **Every published block differs from the one before it**, and the sample
  counter advances (99 -> 171 -> 241), so a frozen-IMU detector will not
  false-trip.
* Diagnostic counters all zero, including `chk_fail`, `gap_reset` and
  `line_err` - nothing on the wire, as expected with no master attached.

Earlier revisions of this board also measured, on **Rev 1** hardware and still
true of the sensor path, which neither v10 nor v11 touched: publish rate 500.45 Hz
against the 500 Hz schedule (+0.09 %); gyro bias steady at -0.44 / 0.00 /
+0.19 dps; uptime past three minutes with no hang or drift; the LED PWM running
in hardware with SWD intact.

Still unverified: **the bus itself.** `ft_slave_replies()` reads 0 because
nothing is polling - that needs the robot, or a USB-TTL adapter on the bus
connector driving FT-SCS at 500 kbps. Everything on this board up to the
connector is now proven, and the framing it would put on the wire is proven
byte for byte; what has not been watched is a real master getting an answer,
and PA12 going low around each status packet and nowhere else.

### LED

D2's anode sits on +3.3 V through a 1 k resistor, so the LED pin **sinks** - low
is lit. The pin is driven as PWM at 999 Hz with 1000 duty steps, which is what
lets the second beat be dimmer than the first: **TIM1_CH4 (AF2) on PA11** on
Rev 2, **TIM14_CH1 (AF13) on PA8** on Rev 1.

| state | pattern |
|---|---|
| healthy | **heartbeat**, 60 bpm: an 80 ms lub at full, 120 ms dark, a 60 ms softer dub, then dark until the next beat. Hard edges, no fade (v8; v3–v7 was a 2 s breath, too slow and soft to read as alive; v2 was a lub-dub every 5 s, which read as "blinked and stopped") |
| fault | **~12 Hz** hard on/off |

The two are unalike in kind, not just in rate - one beats, the other strobes -
so which one you are looking at is obvious at a glance instead of needing to
be timed.

Three details that matter more than they look:

* **Not TIM3.** TIM3 is the microsecond clock the bus slave times its
  turnaround with, fixed at `ARR = 0xFFFF`; as a PWM period that is 15 Hz,
  i.e. visible flicker. On Rev 2 the LED pin only reaches TIM1, and TIM1 is an
  advanced timer: its outputs stay off until `BDTR.MOE` is set
  (`LED_TIM_HAS_MOE` in `board.h`). Forgetting that is silent - the timer
  counts, CCR updates, the pin never moves.
* **`CCxP` inverts the channel**, so `CCRx` reads as brightness rather than as
  darkness on a pin that sinks.
* **Duty is the square of the level.** The eye is not linear; without it the
  dub at level 600 looks nearly as bright as the lub at 1000, and the beat
  loses its second, quieter sound.

Fault detection is unchanged: a bad `WHO_AM_I` at boot, or an SPI transfer
timing out at any point after it. `bus_error` is refreshed every 2 ms poll, so
a fault is held for 2 s - otherwise an intermittent failure would strobe for one
poll and nobody would see it. A state change re-arms the timer immediately, so a
fault appearing in the rest between beats shows up at once.

A missing IMU does not stop the bus slave: the board still answers, with a zero
block and `IMU_PRESENT` clear, which is diagnosable from the host. A silent
board looks exactly like an unplugged connector.

---

## Datasheets

`docs/LSM6DSV16XTR.pdf` (DS13510 rev 4) and
`docs/STM32C011F6U6TR.pdf`. Register addresses and bit positions in the
driver are cited against the former.

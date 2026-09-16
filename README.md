# microduck IMU firmware (`imu_to_dxl` v2)

Firmware for the STM32C011F6U6TR on the microduck IMU board: it reads an
LSM6DSV16XTR over SPI and serves the result on the Dynamixel servo bus as a
Protocol 2.0 device at ID 200.

---

## How the Radxa ZERO 3W actually gets IMU data

**It never talks to the LSM6DSV16X.** There is no I²C or SPI link between the
SBC and the sensor. The path is:

```
Radxa ZERO 3W  ──/dev/ttyS2, Dynamixel 2.0, 1 Mbps──┬── ID 200  this board ──SPI──> LSM6DSV16XTR
   robotd                                            ├── ID  ..  XL330 servo
   duck-control::bus::DynamixelIo                    └── ... 15 servos total
```

Per control tick (50 Hz) the host issues **one** `SYNC_READ` covering this board
and all fifteen servos, at **address 124, length 12**. That address is not
arbitrary: on an XL330 it is the start of `present_pwm` / `present_current` /
`present_velocity` / `present_position`, and 12 bytes covers all four — so the
IMU can ride the same transaction instead of being a separate poll. This board
is listed **first** in the ID list, so it answers before the servo burst.

Source of truth on the host side:

| What | Where |
|---|---|
| ID 200, 1 Mbps | `microduck/duck-control/src/model.rs` — `IMU_DXL_ID`, `BAUD_RATE` |
| addr 124, len 12, IMU first in the list | `microduck/duck-control/src/bus.rs` |
| block layout and decoding | `microduck/duck-control/src/imu.rs` |

The host does no sensor fusion. The LSM6DSV16X's **SFLP** (sensor fusion
low-power) block produces a game-rotation quaternion and estimates its own gyro
bias; `SflpDecoder` only rotates it into the trunk frame, derives gravity, and
runs a 3-sample median for spike rejection.

### The 12 bytes the tick consumes

| bytes | contents |
|---|---|
| 0..6 | gyro x/y/z, `i16` LE raw counts, ±500 dps (17.5 mdps/LSB) |
| 6..12 | SFLP game rotation vector x/y/z as IEEE binary16 |

`w` is **not** transmitted — the host reconstructs it as `√(1 − x² − y² − z²)`.
All-zero quaternion bytes mean *SFLP has not written its table yet*, and the
host holds its last good value rather than snapping to identity (snapping would
tell fall detection an unknown-orientation robot is upright). This firmware
therefore publishes zeros until the first quaternion actually arrives, roughly
0.7 s after power-up.

`duck-control` also watches for a block identical to its predecessor and warns
after 25 consecutive repeats — "the read succeeded but the board handed back the
same sample". The byte-18 sample counter in the full block below makes that
fault unambiguous.

### The full 20-byte block this firmware serves at 124

| offset | bytes | contents |
|---|---|---|
| 0 | 6 | gyro x/y/z, `i16` LE, ±500 dps |
| 6 | 6 | SFLP quaternion x/y/z, binary16 LE |
| 12 | 6 | accelerometer x/y/z, `i16` LE, ±4 g (0.122 mg/LSB) |
| 18 | 1 | sample counter, wraps at 256 |
| 19 | 1 | status flags |

Status flags: `0x01` IMU present (WHO_AM_I matched), `0x02` SFLP ready,
`0x04` FIFO overran since last publish, `0x08` SPI transfer timed out,
`0x10` a mounting correction is applied (v7, below).

Also served, for bus scans and diagnostics:

| addr | size | value |
|---|---|---|
| 0 | 2 | model number `0xC011` |
| 2 | 1 | firmware version |
| 3 | 1 | ID (200) |
| 4 | 1 | baud rate code (3 = 1 Mbps) |
| 5 | 1 | return delay time (0) |
| 32 | 8 | mounting correction in force (v7): the sensor-frame quaternion w, x, y, z as four `i16` × 10000. `(10000, 0, 0, 0)` means none |
| 8 | 22 | diagnostic counters (v4), eleven `u16` LE, wrapping, never cleared — take differences: requests that reached the parser, CRC failures, mid-packet gap resets, receiver FE/NE/ORE, `SYNC_READ`s answered, replies framed, replies completed (TC), largest mid-packet gap tolerated (µs, v5), the gap that caused the last reset and the byte it struck at, receiver overruns (v6). `read 200 8 22` after a run says where a request the host never got an answer to actually died |

Supported instructions: `PING` (0x01), `READ` (0x02), `WRITE` (0x03, accepted
and ignored — nothing is writable), `SYNC_READ` (0x82).

`duck-control::check_registers` asserts EEPROM registers on the **servos only**,
never on ID 200, so this board does not have to emulate the XL330 control table.

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
`DEAT`/`DEDT` (4/16, i.e. 250 ns at 1 Mbps). `CR3.DEP` inverts it to match the
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
| 18 | **PB6** | DATA_TX | Dynamixel data, J2/J3 pin 3, **direct** |

Unconnected: PA1, PA2, PA3, PA9/PA11, PA10/PA12, PB7, PC14, PC15.

DATA_TX runs from PB6 straight to the connectors — **no bus transceiver, no
direction pin**. PB6 is `USART1_TX`; `USART1_RX` cannot be routed to it. So:

* USART1 runs in **single-wire half-duplex** (`CR3.HDSEL`), which feeds the
  receiver from the TX pin.
* HDSEL alone would leave the transmitter idling the pin high and fighting every
  servo. So PB6 sits in alternate-function **open-drain** while receiving (an
  idle-high transmitter simply releases it) and flips to **push-pull** for the
  ~230 µs it takes to send a status packet. Open-drain throughout would be
  simpler, but at 1 Mbps the pull-up alone cannot make the rising edges.

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
at 12 MHz a 1 Mbps USART is unreachable with 16x oversampling (`USARTDIV` must
be at least 16). At 48 MHz `BRR` lands on exactly 48, with no rounding error at
all.

So the entire bus-rate error budget is HSI48 trim. From DS13866 Rev 5, Table 40:

| | |
|---|---|
| f at V_DD = 3.0 V, T_A = 30 °C | 47.92–48.40 MHz (−0.17 % / +0.83 %) |
| drift, T_A = 0 to 85 °C | ±1 % |
| drift, T_A = −40 to 125 °C | −2.5 % / +2 % |

A 1 Mbps 8N1 link tolerates roughly ±2 % end to end, and everything else on the
bus — the XL330s, a U2D2 — is crystal-clocked, so nearly all of that budget is
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
| `Src/dxl_slave.c` | Protocol 2.0 slave on USART1, all in the ISR; hardware DE (Rev 2) or HDSEL + open-drain (Rev 1) |
| `Src/app.c` | poll loop, sensor block assembly, LED |
| `Src/mount.c`, `Inc/mount.h` | per-robot mounting correction (v7): a fixed sensor-frame rotation applied to gyro, accel and the SFLP quaternion before publishing, so a board mounted a few degrees off reads as mounted straight. The host has no trim for this; the constants and the measurement procedure are in `mount.h` |

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

120 Hz also keeps `duck-control` from crying stale: at 2.4 IMU samples per tick
every host read gets a fresh block, so the 25-repeat `STALE_RUN_WARN` cannot
trip. An SFLP ODR of 15 or 30 Hz would trip it constantly.

* Poll: every 2 ms (500 Hz), four SPI transactions of ~20 µs each.
* Bus: the whole protocol runs in `USART1_IRQHandler`, so a request is answered
  as soon as its last CRC byte lands. `return_delay_time` is 0, matching what
  the host asserts of the servos.
* The USART runs in **FIFO mode** (v3): eight bytes of receive buffering, so
  the ISR may be away for 80 µs at 1 Mbps before a byte is lost, against 10 µs
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

Verified off-target:

* **It builds and links** with the GCC 14.3 that ships with CubeIDE, clean under
  `-Wall -Wextra` — no warnings from any of these sources.

  | config | flash | RAM |
  |---|---|---|
  | Debug (`-O0 -g3`) | 7 052 / 32 768 — 22 % | 3 540 / 6 144 — 58 % |
  | Release (`-Os`) | 3 512 / 32 768 — **11 %** | same |

  RAM is 2 004 bytes of `.bss` plus the linker script's 0x200 heap and 0x400
  stack.
* **`USART1_IRQHandler` and `SysTick_Handler` override the weak defaults.**
  Vector table offset `0xAC` holds `0x080011E9` and offset `0x3C` holds
  `0x08000339` — the two handlers, not `Default_Handler`. Worth checking, since
  both are placed by link-time override rather than by any config.
* **The CRC** against both worked examples in the ROBOTIS Protocol 2.0
  documentation (`89CA` and `0CA1`).
* **The framing**, by transcribing it and round-tripping the exact `SYNC_READ`
  the control loop issues — with a sensor block containing `FF FF FD`, which
  exercises byte stuffing in both directions.

Verified **on hardware** (STLINK-V3MINIE over SWD, target at 3.27 V, device ID
0x443):

* Flashed and read back verified; the firmware runs, SysTick counts.
* `s_imu_present = 1` - the LSM6DSV16X answered `WHO_AM_I` over SPI1.
* Status byte `0x03` = `IMU_PRESENT | SFLP_READY`, with `FIFO_OVERRUN` and
  `SPI_ERROR` clear.
* Accelerometer magnitude **0.9986 g** at rest. This is the load-bearing number:
  it confirms the +/-4 g full scale and the 0.122 mg/LSB conversion together,
  because only the right pair gives 1 g.
* Gyroscope within 0.5 dps of zero on a stationary bench.
* **Publish rate 500.45 Hz** measured against the 500 Hz schedule: 558 samples
  in 1 115 ms of firmware time, +0.09 %. Measured again after the PWM heartbeat
  was added, and the earlier figure without it was 499.1 Hz - a 0.27 % spread
  against a measurement precision of about +/-0.36 % (one count of a byte-wide
  counter at each end of the window). The fade is below the noise floor, which
  is where the arithmetic says it should be: ~300 cycles every 4 ms is 0.16 %
  of a 48 MHz core.
* **Every published block differs from the one before it** across five
  consecutive reads. This is the property `duck-control`'s frozen-IMU detector
  keys on, so it will never false-trip.
* Accelerometer magnitude across five reads: 0.9980 to 0.9991 g, a spread of
  about 9 LSB. Gyro bias steady at -0.44 / 0.00 / +0.19 dps.
* Uptime past three minutes with `IMU_PRESENT` still set and the counter
  advancing - no hang, no drift.
* **The LED PWM runs in hardware.** `TIM14` reads back `CR1 = 0x081`
  (`ARPE|CEN`), `CCMR1 = 0x068` (PWM mode 1 with preload), `CCER = 0x003`
  (`CC1E|CC1P`), `PSC = 47`, `ARR = 1000`; `CNT` advances between reads and
  `CCR1` was caught at 342, part-way through a fade. `GPIOA->MODER` has PA8 in
  alternate function with `AFRH = 0xD`, and PA13/PA14 are still `10` - SWD
  intact.
* SFLP quaternion non-zero and normalised, w = 0.99964, about 3 degrees off
  level - consistent with a board lying on a desk.

All of the above was measured on a **Rev 1** board. Still unverified: **the
Dynamixel bus itself**, on either revision. `dxl_slave_replies()` reads 0
because nothing is polling - that needs the robot or a U2D2 on the bus
connector. Everything upstream of the connector is proven on Rev 1; Rev 2 has
so far only been built (`-Wall -Wextra` clean, 3 872 bytes at `-Os`) and needs
the same bring-up pass - in particular that PA12 goes low around each status
packet and nowhere else.

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

* **Not TIM3.** TIM3 is the microsecond clock the Dynamixel slave times its
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

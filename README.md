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
`0x04` FIFO overran since last publish, `0x08` SPI transfer timed out.

Also served, for bus scans and diagnostics:

| addr | size | value |
|---|---|---|
| 0 | 2 | model number `0xC011` |
| 2 | 1 | firmware version |
| 3 | 1 | ID (200) |
| 4 | 1 | baud rate code (3 = 1 Mbps) |
| 5 | 1 | return delay time (0) |

Supported instructions: `PING` (0x01), `READ` (0x02), `WRITE` (0x03, accepted
and ignored — nothing is writable), `SYNC_READ` (0x82).

`duck-control::check_registers` asserts EEPROM registers on the **servos only**,
never on ID 200, so this board does not have to emulate the XL330 control table.

---

## Pin assignment (from `docs/schematic.pdf`)

STM32C011F6U6TR, UFQFPN20. Source: `docs/schematic.pdf`.

| Pin | Port | Net | Function |
|---|---|---|---|
| 2 | VDD/VDDA | +3.3V | AP2210K LDO, VIN 3.5–13.2 V |
| 3 | VSS/VSSA | GND | |
| 4 | PF2/NRST | RESET | 10 k pull-up + 100 nF, TC2030 pin 3 |
| 5 | **PA0** | IMU_INT | ← LSM6DSV16X INT1 (pin 4) |
| 9 | **PA4** | SPI_CS | → IMU CS (pin 12), 10 k pull-up — software NSS |
| 10 | **PA5** | SPI_SCK | → IMU SCL/SCK (pin 13), AF0 = SPI1_SCK |
| 11 | **PA6** | SPI_MISO | ← IMU SDO/SA0 (pin 1), AF0 = SPI1_MISO |
| 12 | **PA7** | SPI_MOSI | → IMU SDA/SDI (pin 14), AF0 = SPI1_MOSI |
| 13 | **PA8** | LED | D2 anode on +3.3V through R2 1 k — **LOW = lit** |
| 16 | PA13 | SWDIO | TC2030 pin 2 |
| 17 | PA14/BOOT0 | SWCLK | TC2030 pin 4, 10 k pull-down on BOOT0 |
| 18 | **PB6** | DATA_TX | Dynamixel data, J2/J3 pin 3, AF0 = USART1_TX |

Unconnected: PA1, PA2, PA3, PA9/PA11, PA10/PA12, PB7, PC14, PC15.
On the IMU, SDx and SCx are tied to GND, and INT2 / OCS_AUX / SDO_AUX are open —
plain 4-wire SPI with no OIS chain.

J2 and J3 are identical 3-pin JST EH (GND, VIN, DATA_TX) for daisy-chaining.

### The one non-obvious thing about the bus pin

DATA_TX runs from PB6 straight to the connectors — **no bus transceiver, no
direction pin**. PB6 is `USART1_TX`; `USART1_RX` cannot be routed to it. So:

* USART1 runs in **single-wire half-duplex** (`CR3.HDSEL`), which feeds the
  receiver from the TX pin.
* HDSEL alone would leave the transmitter idling the pin high and fighting every
  servo. So PB6 sits in alternate-function **open-drain** while receiving (an
  idle-high transmitter simply releases it) and flips to **push-pull** for the
  ~230 µs it takes to send a status packet. Open-drain throughout would be
  simpler, but at 1 Mbps the pull-up alone cannot make the rising edges.

`RE` is cleared during transmission so HDSEL does not echo our own bytes back
into the parser.

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
| `Inc/board.h` | pin map, transcribed from the schematic |
| `Src/main.c` | clock, SysTick, main loop |
| `Src/lsm6dsv16x.c` | SPI1 at register level; chip config; FIFO drain for SFLP |
| `Src/dxl_slave.c` | Protocol 2.0 slave, USART1 half-duplex, all in the ISR |
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

120 Hz also keeps `duck-control` from crying stale: at 2.4 IMU samples per tick
every host read gets a fresh block, so the 25-repeat `STALE_RUN_WARN` cannot
trip. An SFLP ODR of 15 or 30 Hz would trip it constantly.

* Poll: every 2 ms (500 Hz), four SPI transactions of ~20 µs each.
* Bus: the whole protocol runs in `USART1_IRQHandler`, so a request is answered
  as soon as its last CRC byte lands. `return_delay_time` is 0, matching what
  the host asserts of the servos.
* Foreign packets — about fifteen servo status packets per tick — are dropped at
  the ID byte before any CRC is computed.
* A gap of more than 100 µs between bytes abandons a part-received packet. Bytes
  inside a packet are 10 µs apart, so a real gap means the rest is not coming;
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

Still unverified: **the Dynamixel bus itself.** `dxl_slave_replies()` reads 0
because nothing is polling - that needs the robot or a U2D2 on J2/J3. Everything
upstream of the connector is now proven.

### LED

D2's anode sits on +3.3 V through R2, so PA8 **sinks** - low is lit. The pin is
driven as PWM by **TIM14_CH1 (AF13)** at 999 Hz with 1000 duty steps, which is
what lets the healthy pattern fade instead of switch.

| state | pattern |
|---|---|
| healthy | a faded **lub-dub** every 5 s: rise to full over 140 ms, decay, then a weaker second beat peaking at 65 %, dark from 700 ms |
| fault | **~12 Hz** hard on/off, no fade |

The two are unalike in kind, not just in rate - one breathes, the other strobes
- so which one you are looking at is obvious at a glance instead of needing to
be timed.

Three details that matter more than they look:

* **TIM14, not TIM3.** TIM3 is the microsecond clock the Dynamixel slave times
  its turnaround with, fixed at `ARR = 0xFFFF`; as a PWM period that is 15 Hz,
  i.e. visible flicker. TIM14 is otherwise unused and, unlike TIM1, needs no
  `BDTR.MOE` to drive its output.
* **`CC1P` inverts the channel**, so `CCR1` reads as brightness rather than as
  darkness on a pin that sinks.
* **Duty is the square of the level.** The eye is not linear; a linear ramp
  spends most of its travel in a range that already looks fully lit, and reads
  as a switch with a soft edge rather than as breathing.

Fault detection is unchanged: a bad `WHO_AM_I` at boot, or an SPI transfer
timing out at any point after it. `bus_error` is refreshed every 2 ms poll, so
a fault is held for 2 s - otherwise an intermittent failure would strobe for one
poll and nobody would see it. A state change re-arms the timer immediately, so a
fault appearing inside the quiet 4.3 s of a heartbeat shows up at once.

A missing IMU does not stop the bus slave: the board still answers, with a zero
block and `IMU_PRESENT` clear, which is diagnosable from the host. A silent
board looks exactly like an unplugged connector.

---

## Datasheets

`docs/LSM6DSV16XTR.pdf` (DS13510 rev 4) and
`docs/STM32C011F6U6TR.pdf`. Register addresses and bit positions in the
driver are cited against the former.

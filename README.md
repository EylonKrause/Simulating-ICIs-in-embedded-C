# Simulating ICIs in embedded C

A complete 200 Gb/s/lane PAM4 SerDes lane â€” analogue front end, equalisers,
CDR, and the **firmware that controls them** â€” written from scratch in C17 with
no dependencies.

The point of the project is not the DSP. It is the **hardware/firmware split**:
the datapath is modelled as silicon, the control plane is written as real
embedded firmware, and the only thing joining them is a register interface.
That separation is what lets the adaptation loop be unit-tested in CI with no
hardware at all.

```
200 Gb/s/lane PAM4  =  100 GBd  =  50 GHz Nyquist
8 lanes/macro x 6 macros        =  9.6 Tb/s per chip
```

This is a public-domain specification built from published part classes
(200G/lane PAM4 PHYs of the kind Broadcom and MediaTek ship). It models no
vendor's silicon and contains nothing proprietary.

---

## Where each job responsibility lives

| Responsibility | Implementation |
|---|---|
| Real-time embedded C for **data path control, management, and telemetry** | [`src/fw_bringup.c`](src/fw_bringup.c) â€” bring-up FSM, per-state timeouts, exponential backoff, telemetry counters |
| **Low-level drivers and HAL** to interface with DSP hardware blocks and registers | [`include/hal.h`](include/hal.h), [`src/hal.c`](src/hal.c) â€” register map, `volatile` MMIO, W1C semantics, critical sections |
| Firmware-based **adaptation for equaliser taps** | [`src/fw_adapt.c`](src/fw_adapt.c) â€” sign-sign LMS supervisor, gear shifting, leakage |
| Firmware-based **gain control (VGA/TIA)** | [`src/fw_agc.c`](src/fw_agc.c) â€” AGC with VGAâ†’TIA handoff |
| **Unit tests and CI** across hardware revisions | [`tests/test_all.c`](tests/test_all.c) â€” 49 checks, no hardware required |
| **Fixed-point optimisation** for resource-constrained systems | [`include/fixed.h`](include/fixed.h) â€” Q-format, saturation, accumulate-wide/apply-narrow |

**No `fw_*.c` file contains a single floating-point operation.** Floats appear
only in the hardware model, where they stand in for analogue physics.

---

## Architecture

```
  FIRMWARE  (fixed point, kHz)          fw_bringup.c  fw_agc.c  fw_adapt.c
      |                                              |
      |  hal_read32 / hal_write32  <-- the ONLY seam |
      v                                              v
  REGISTERS   volatile uint32_t[]   RW | RO | W1C   hal.c
      ^                                              |
      |  hardware drives RO/W1C   accumulators, status
      |                                              v
  HARDWARE  (floating point, 100 GBd)   hw_lane.c
      TX -> channel -> AFE -> CDR -> FFE -> DFE -> slicer
      tx.c   channel.c  afe.c  cdr.c      eq.c
```

Control flows one way only. Firmware writes control registers and reads
statistics; hardware writes status and accumulators. Swap `hal.c`'s backing
array for a real peripheral aperture and the firmware source is unchanged â€”
that substitution is the whole reason the tests can run without silicon.

| Module | What it does |
|---|---|
| `fft.c` | radix-2 FFT, used by the channel synthesiser |
| `channel.c` | insertion-loss model â†’ **minimum-phase** impulse response |
| `tx.c` | PRBS31, Gray-coded PAM4, TX FFE with an L1 (peak-power) constraint |
| `afe.c` | photodiode + TIA (with gainâ€“bandwidth tradeoff), VGA, CTLE |
| `eq.c` | FFE/DFE datapath and the sign-sign gradient accumulators |
| `cdr.c` | Mueller-Muller TED, type-2 PI loop, amplitude-normalised |
| `eye.c` | eye-diagram accumulation, PGM and ASCII rendering |

---

## Build and run

Needs only MSVC Build Tools (or any C17 compiler).

```bat
build.bat test_all           :: 49 unit tests, no hardware
build.bat ch_probe 30        :: channel synthesis, verified against its own model
build.bat cdr_probe 20 120   :: CDR loop in isolation, instrumented
build.bat link_sim 20 120    :: full bring-up, eye diagram, telemetry
build.bat test_all asan      :: any target under AddressSanitizer
```

---

## Verified results

**Channel synthesis closes the loop.** Ask for a loss, synthesise the impulse
response, transform it back, and measure what you actually got:

```
  freq[GHz]   model[dB]   synthesised[dB]
      10.0     -10.08         -10.05
      25.0     -18.42         -18.42
      50.0     -30.00         -29.86   <- Nyquist
```

**Minimum phase gives physical asymmetry.** Precursors die within two UI while
the postcursor tail runs long â€” which is exactly why a postcursor-only DFE is
worth building:

```
  k= -2  +0.00032
  k= -1  +0.07556  =====================
  k= +0  +0.16464  ==============================================  <- cursor
  k= +1  +0.12948  ====================================
  k= +2  +0.08982  =========================
  cursor 0.16464   sum|ISI| 0.51736   worst-case eye CLOSED
```

**Fixed-point rounding, measured over 8572 operations** (Q30 units, 1 LSB = 32768):

```
  truncation  -140,447,934   =  -0.5 LSB per operation
  rounding          -4,286   =  -1.5e-5 LSB per operation
```

A 32,768Ã— reduction in DC bias. In an open-loop filter that is a curiosity; in
an LMS accumulator or a CDR loop filter it is the difference between a loop
that holds and one that walks off target.

**The firmware's bus behaviour is asserted, not assumed.** The HAL counts its
own traffic, so the tests can require that the firmware never performs an
unguarded read-modify-write and never read-modify-writes a W1C register.

---

## Verified end to end

```
  t[ms]  state        VGA  TIA  CTLE
      0  WAIT_SIGNAL  32   8    12
      1  AGC          32   8    12
     12  CDR_LOCK     37   8    12
     20  EQ_TRAIN     37   8    12
    170  TRACK        37   8    12
    177  UP           44   8    12

  LINK UP after 177 ms

  converged FFE taps           HAL access audit
    w[2] =   -3                  read-modify-writes  226
    w[3] =  +24                  UNGUARDED RMW         0  (ok)
    w[4] =   -3                  W1C RMW bugs          0  (ok)
```

Cursor pulled down with negative neighbours either side -- an FFE cancelling
pre- and post-cursor ISI. `49 checks, 0 failures`.

## Known limitations

Stated plainly rather than hidden.

- **The CDR's steady-state frequency estimate rails at the anti-windup clamp**
  (+996 ppm reported against +120 actual). The loop tracks well enough for the
  link to come up and run error-free, but the integrator sits at its limit
  rather than settling on the true offset. Residual TED bias, not yet resolved.
- Ideal FEC. No RS-FEC or LDPC layer, so BER is pre-FEC only.
- The channel is a fitted two-term loss model, not measured S-parameters. No
  reflections, no crosstalk, no via stubs.
- Single lane. The 48-lane figure is a specification, not a simulated array.
- No PLL model. Bring-up goes reset -> AGC -> CDR -> equaliser; a real sequence
  also waits on PLL lock between reset and AGC.
- The eye is written to a `.pgm` file. Real silicon cannot write files -- it
  would packetise the eye-monitor histogram and stream it out over a low-speed
  management bus (I2C, SPI, or a debug ring) for a host to reassemble.

## Bugs found while building this

Documented where they occurred, because the diagnosis is worth more than the
fix. Every one of these was found by instrumenting, not by reasoning.

| Bug | Why it was hard to see |
|---|---|
| `hal_field_set(reg, ADAPT_CDR_EN, 0, 1)` computes `(1<<0) & (1<<1)` = **0** | The CDR was never enabled at all. `ADAPT_AGC_EN` is bit 0, so *that* one worked -- everything downstream silently did nothing. Fixed by adding `hal_bit_write()`, which takes a mask and no shift to mismatch. |
| Mueller-Muller TED sign inverted | Positive feedback. Small **constant** `mean(e)` with a monotonically running integrator is the fingerprint -- it means the sign is wrong, not that the gains need tuning. I tuned gains for a long time before checking. |
| LCG low-order bits have period 2 | `lcg() & 1` gives alternating "random" data, so worst-case ISI never occurs and every eye looks open. Silent, and it validates a receiver that does not work. |
| Lock detected from mean of `\|e\|` | The TED magnitude stays O(1) when locked; only its *signed* mean goes to zero. |
| A uniformly sweeping phase also gives mean(e) ~ 0 | Needs a second independent condition -- hence the integrator-stability test. |
| Hardware accumulators never cleared per block | The AGC read a lifetime average, concluded its corrections did nothing, and drove the gain to the rail. |
| Convergence judged from gradient magnitude | A sign-sign gradient does not shrink as taps settle. Threshold 600 never fired; 5600 fired instantly. Convergence has to be measured from **tap movement**. |
| `mu_shift` 4 orders of magnitude too large | Gradient ~500/block, `TAP_APPLY_SHIFT` 14, so 16384 blocks per tap code. The link came "up" with untouched taps. |
| Two consumers of one read-and-clear register | `fw_agc_step()` drained `REG_SYM_CNT` before telemetry read it, so symbol and error counts were always zero. |
| Signed left-shift of a negative value | Undefined behaviour. MSVC and gcc both compiled it silently; UBSan caught it on the first CI run. |
| Include guard `EYE_H` collided with a constant `EYE_H` | -- |

## Telemetry over a management bus

A chip cannot write `eye.pgm`. Everything a host learns about a lane arrives as
bytes over a slow side-channel while the data path runs at 100 GBd:

```
  data path        100 GBd    = 2e11 bit/s
  management bus     1 Mb/s   = 1e6  bit/s        200,000x slower
```

Every design decision follows from that ratio. [`src/fw_telem.c`](src/fw_telem.c)
packetises status, counters, taps and the eye histogram into fixed 32-byte
frames -- SOF, type, little-endian sequence, length, payload, CRC-8 -- and
[`apps/mgmt_host.c`](apps/mgmt_host.c) is a host that shares no memory with the
firmware: it hunts for framing byte by byte, validates every CRC, tracks
sequence gaps, and reassembles the eye from chunks.

```
  frames ok / bad     205 / 0
  sequence gaps       0
  bytes dropped by HW 0
  FFE taps              -1   +0   -3  +19   -3   -1   -1   -1
  eye reassembled     768/768 bytes (32 x 24)
```

Four things that only matter once the bus is real:

- **Backpressure.** The FIFO is 512 bytes. `REG_MGMT_STAT` reports free space
  and the producer checks it before every frame. Push into a full FIFO and the
  byte is simply gone -- so firmware that blasts loses the middle of its own
  eye and reports a corrupt one.
- **Chunking.** 32x24 downsampled is 768 bytes = 30 frames. The full 128x96
  histogram would be 12288 bins and take half a second. Telemetry is a
  background task; it never blocks a control loop.
- **Round-robin.** A long eye transfer must not starve status. Losing the link
  and being unable to say so is the worst failure mode here.
- **An indexed register window.** Firmware has no pointer into the capture RAM.
  It writes `REG_EYE_ADDR` and reads `REG_EYE_DATA`, two bus transactions per
  byte -- which is itself why you stream a downsampled eye.

## PLL lock in the bring-up sequence

```
RESET -> PLL_LOCK -> WAIT_SIGNAL -> AGC -> CDR_LOCK -> EQ_TRAIN -> TRACK -> UP
```

The reference PLL is the clock everything else depends on: a CDR cannot recover
a clock when there is none to recover against. [`src/pll.c`](src/pll.c) models
charge-pump settling, and the FSM **waits with a timeout** rather than assuming
-- a PLL can fail outright on a wrong divider, an absent reference, or a VCO out
of band. `pll_force_fail()` exists so that timeout path is reachable in tests.

Losing PLL lock is immediate death; losing CDR lock is not. That asymmetry is
in the code, and it came from a measurement: the link came up, emitted exactly
one telemetry frame, then tore itself down on a single dropped lock sample.
Loss-of-lock now needs 25 consecutive ticks. The difference between a glitch
and an outage is hysteresis.

## The RMW hazard, demonstrated rather than asserted

The HAL used only to *count* unguarded read-modify-writes. A count is a claim.
Now an interrupt actually fires at the one instant that matters -- between the
read and the write inside `hal_field_set()` -- and the test watches the update
vanish:

```c
hal_attach_isr(isr_sets_CTRL_EN, NULL);
hal_field_set(REG_CTRL, 0xF0u, 4u, 0x5u);      /* no critical section */

CHECK((reg & 0xF0u) == 0x50u, "our field was written");
CHECK((reg & CTRL_EN) == 0u,  "LOST UPDATE: the ISR set it, the RMW wiped it");
```

Wrap the same call in `hal_critical_enter/exit` and the interrupt is deferred,
replayed on exit, and **nothing is lost** -- exactly as a pending interrupt
behaves in an NVIC when PRIMASK clears. This is what `volatile` does *not* buy
you: it guarantees the accesses happen, not that they happen atomically.

## What I would add next

- **FEC.** RS-FEC (KP4) or LDPC, so BER is post-FEC and soft-decision gain is
  visible. Currently pre-FEC only.
- **Measured S-parameters** instead of a fitted two-term loss model -- no
  reflections, no crosstalk, no via stubs today.
- **Multi-lane.** The 48-lane figure is a specification; one lane is simulated.

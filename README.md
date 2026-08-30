# Simulating ICIs in embedded C

A complete 200 Gb/s/lane PAM4 SerDes lane — analogue front end, equalisers,
CDR, and the **firmware that controls them** — written from scratch in C17 with
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
| Real-time embedded C for **data path control, management, and telemetry** | [`src/fw_bringup.c`](src/fw_bringup.c) — bring-up FSM, per-state timeouts, exponential backoff, telemetry counters |
| **Low-level drivers and HAL** to interface with DSP hardware blocks and registers | [`include/hal.h`](include/hal.h), [`src/hal.c`](src/hal.c) — register map, `volatile` MMIO, W1C semantics, critical sections |
| Firmware-based **adaptation for equaliser taps** | [`src/fw_adapt.c`](src/fw_adapt.c) — sign-sign LMS supervisor, gear shifting, leakage |
| Firmware-based **gain control (VGA/TIA)** | [`src/fw_agc.c`](src/fw_agc.c) — AGC with VGA→TIA handoff |
| **Unit tests and CI** across hardware revisions | [`tests/test_all.c`](tests/test_all.c) — 49 checks, no hardware required |
| **Fixed-point optimisation** for resource-constrained systems | [`include/fixed.h`](include/fixed.h) — Q-format, saturation, accumulate-wide/apply-narrow |

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
array for a real peripheral aperture and the firmware source is unchanged —
that substitution is the whole reason the tests can run without silicon.

| Module | What it does |
|---|---|
| `fft.c` | radix-2 FFT, used by the channel synthesiser |
| `channel.c` | insertion-loss model → **minimum-phase** impulse response |
| `tx.c` | PRBS31, Gray-coded PAM4, TX FFE with an L1 (peak-power) constraint |
| `afe.c` | photodiode + TIA (with gain–bandwidth tradeoff), VGA, CTLE |
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
the postcursor tail runs long — which is exactly why a postcursor-only DFE is
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

A 32,768× reduction in DC bias. In an open-loop filter that is a curiosity; in
an LMS accumulator or a CDR loop filter it is the difference between a loop
that holds and one that walks off target.

**The firmware's bus behaviour is asserted, not assumed.** The HAL counts its
own traffic, so the tests can require that the firmware never performs an
unguarded read-modify-write and never read-modify-writes a W1C register.

---

## Known limitations

Stated plainly rather than hidden.

- **End-to-end CDR acquisition in `link_sim` is not yet reliable.** The loop is
  verified working in isolation — `cdr_probe` acquires a 120 ppm offset over
  VGA codes 34–40 and estimates it to within ~5% — but integrated with the AGC
  the operating points do not consistently overlap and bring-up often exhausts
  its retries. The failure is in the *integration*, not the loop; `cdr_probe`
  is the reproducer.
- Ideal FEC. There is no RS-FEC or LDPC layer, so BER is pre-FEC only.
- The channel is a fitted two-term loss model, not measured S-parameters. No
  reflections, no crosstalk, no via stubs.
- Single lane. The 48-lane figure is a specification, not a simulated array.
- The CTLE is a first-order zero/pole section; real CTLEs have more poles.

## Debugging notes worth reading

Several bugs found while building this are documented in the source where they
occurred, because the diagnosis is more useful than the fix:

- `hw_lane.c` — an LCG's low-order bits have period 2, so `lcg() & 1` produces
  alternating "random" data, worst-case ISI patterns never occur, and every eye
  looks open. Silent, and it validates a receiver that does not work.
- `cdr.c` — lock cannot be detected from the mean of `|e|`; the TED magnitude
  stays O(1) even when locked. Only the *signed* mean goes to zero.
- `cdr.c` — a phase sweeping uniformly through the UI also gives mean(e) ≈ 0,
  so a second independent condition is required.
- `cdr.c` — the sign of the reported ppm: `integ` must be *negative* to cancel a
  positive drift, so a healthy loop reads as tracking backwards if you forget.
- `hw_lane.c` — hardware statistics registers must be cleared per observation
  window. Left accumulating, the AGC reads a lifetime average, concludes its
  correction did nothing, and drives the gain to the rail.
- `eye.h` — an include guard named `EYE_H` collides with a constant named
  `EYE_H`.

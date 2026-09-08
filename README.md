# Simulating ICIs in embedded C

A complete 200 Gb/s/lane PAM4 SerDes lane -- analogue front end, equalisers,
CDR, and the **firmware that controls them** -- written from scratch in C17 with
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

## Where things live

| Concern | Implementation |
|---|---|
| Bring-up sequencing, timeouts, retry, telemetry | [`src/fw_bringup.c`](src/fw_bringup.c) -- the state machine, per-state timeouts, exponential backoff, counters |
| Register interface and low-level access | [`include/hal.h`](include/hal.h), [`src/hal.c`](src/hal.c) -- register map, `volatile` MMIO, W1C semantics, critical sections, per-lane window |
| Equaliser tap adaptation | [`src/fw_adapt.c`](src/fw_adapt.c) -- sign-sign LMS supervisor, gear shifting, leakage |
| Gain control (VGA and TIA) | [`src/fw_agc.c`](src/fw_agc.c) -- AGC with the VGA-to-TIA handoff |
| Forward error correction | [`src/fec.c`](src/fec.c), [`src/pcs.c`](src/pcs.c) -- RS(544,514) KP4, codeword framing, BER scoring |
| Unit tests, no hardware required | [`tests/test_all.c`](tests/test_all.c) -- 158 checks |
| Fixed-point arithmetic | [`include/fixed.h`](include/fixed.h) -- Q-format, saturation, accumulate-wide/apply-narrow |

**No `fw_*.c` file contains a single floating-point operation, and none of them
calls the hardware-side backend.** Both claims are enforced by a grep step in
CI rather than asserted in a comment -- they had each been quietly violated
once, and in both cases the violation wrote a register that nothing read, so
nothing failed.

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
array for a real peripheral aperture and the firmware source is unchanged --
that substitution is the whole reason the tests can run without silicon.

| Module | What it does |
|---|---|
| `fft.c` | radix-2 FFT, used by the channel synthesiser |
| `channel.c` | insertion-loss model -> **minimum-phase** impulse response |
| `tx.c` | PRBS31, Gray-coded PAM4, TX FFE with an L1 (peak-power) constraint |
| `afe.c` | photodiode + TIA (with gain-bandwidth tradeoff), VGA, CTLE |
| `eq.c` | FFE/DFE datapath and the sign-sign gradient accumulators |
| `cdr.c` | Mueller-Muller TED, type-2 PI loop, amplitude-normalised |
| `eye.c` | eye-diagram accumulation, PGM and ASCII rendering |
| `fec.c` | RS(544,514) over GF(2^10) -- KP4 -- with errors-and-erasures decoding |
| `pcs.c` | codeword framing, BER-tester pattern alignment, pre/post-FEC scoring |
| `touchstone.c` | Touchstone (.sNp) S-parameter reader |
| `hw_macro.c` | eight coupled lanes and one round-robin supervisor |

---

## Build and run

Needs only MSVC Build Tools (or any C17 compiler).

```bat
build.bat test_all                     :: 158 unit tests, no hardware
build.bat ch_probe 20                  :: channel synthesis, checked against its own model
build.bat ch_probe ..\data\pkg_backplane.s4p    :: the same, from S-parameters
build.bat cdr_probe 20 -80             :: CDR loop in isolation, instrumented
build.bat link_sim 20 -80              :: full bring-up, FEC, eye diagram, telemetry
build.bat fec_probe                    :: KP4 self-test and coding-gain measurements
build.bat s4p_gen ..\data\pkg_backplane.s4p 14  :: synthesise a 4-port channel file
build.bat macro_sim 8 14               :: eight lanes, one supervisor
build.bat macro_sim 8 14 0 4000 2      :: ...serviced two at a time
build.bat test_all asan                :: any target under AddressSanitizer
```

On Linux and macOS: `make test`, `./build.sh link_sim 20 -80`, or CMake.

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
the postcursor tail runs long -- which is exactly why a postcursor-only DFE is
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

A 32,768x reduction in DC bias. In an open-loop filter that is a curiosity; in
an LMS accumulator or a CDR loop filter it is the difference between a loop
that holds and one that walks off target.

**The firmware's bus behaviour is asserted, not assumed.** The HAL counts its
own traffic, so the tests can require that the firmware never performs an
unguarded read-modify-write and never read-modify-writes a W1C register.

---

## Verified end to end

A clean run, 12 dB and -80 ppm, first attempt:

```
  t[ms]  state        VGA  TIA  CTLE
      0  PLL_LOCK     32   8    8
     10  AGC          32   8    8
     13  CDR_LOCK     32   8    9
     21  EQ_TRAIN     32   8    9
    271  EQ_VERIFY    32   8    9
    345  TRACK        31   8    9
    346  UP           31   8    9

  LINK UP after 346 ms

  FEC  --  RS(544,514) over GF(2^10), t = 15 symbols
    equaliser latency   5 symbols (measured by pattern alignment)
    codewords decoded   298
    uncorrectable       0
    pre-FEC BER         0.000e+00
    post-FEC BER        < 6.5e-07   (no residual errors in 1531720 bits)
    KP4 margin          +25.9 dB against the 2.4e-4 pre-FEC limit

  management bus      268 frames, 0 bytes dropped
  HAL access audit    15 read-modify-writes, 0 unguarded, 0 W1C bugs
```

`158 checks, 0 failures`, and the same under AddressSanitizer.

**The interesting runs are the ones that do not go like that.** At 16 dB and
-200 ppm the search takes six attempts before it finds a workable front end,
and the two ways it fails are both worth seeing (abridged):

```
     17  CDR_LOCK     36   8    10
    317  FAULT        36   8    10   <-- CDR could not acquire: timed out
    ...
    342  EQ_TRAIN     39   8    13
    592  EQ_VERIFY    39   8    13
    666  FAULT        35   8    13   <-- converged, then FAILED VERIFICATION
    ...
   2045  UP           34   8    11        pre-FEC 1.2e-6, post-FEC clean
```

The second fault is the one that matters. Every loop reported convergence and
the link was still not good enough, so bring-up rejected its own answer and
tried a different operating point rather than declaring success. An earlier
version of this project had no such check and came up at 12 dB with every
status bit green and a pre-FEC BER of 7.6e-2.

**And the BER above is measured, which needs saying because it was not always
true.** An earlier version printed `pre-FEC BER 0.000e+00` from a counter that
only incremented during training and was only read in a state where training is
off. It was structurally incapable of being non-zero, and it hid a receiver
whose CDR never locked at all. Everything in the bug table below was found
after replacing it with a real measurement.

## Operating range

Swept across channel loss and reference offset, carrying RS-encoded traffic,
BER measured against a pattern-aligned reference. Each cell is the pre-FEC BER
and whether the payload came out of the decoder clean:

| channel | -200 ppm | -80 ppm | +120 ppm |
|---|---|---|---|
| 4 dB  | **0** clean | **0** clean | **0** clean |
| 8 dB  | **0** clean | **0** clean | 6.2e-7 clean |
| 12 dB | **0** clean | **0** clean | **0** clean |
| 16 dB | 1.2e-6 clean | 4.0e-4 errors | 1.5e-4 errors |
| 20 dB | 2.3e-5 clean | 3.6e-4 errors | 1.9e-5 clean |
| 24 dB | 6.5e-4 errors | 2.5e-4 errors | 1.2e-4 errors |
| 28 dB and above | did not come up | did not come up | did not come up |

Bring-up takes 350 ms to 2.7 s, depending on how many front-end operating
points the search has to try.

**4, 8 and 12 dB are clean at every reference offset -- zero pre-FEC errors
over 1.5 million bits. 16 to 24 dB is marginal. Above 24 dB the link does not
come up, and the state machine says so rather than pretending.**

That last part is the change that matters most. Bring-up measures its own error
rate, decision-directed, before declaring the link up, so a converged-but-wrong
solution is rejected and retried instead of shipped. An earlier version brought
the link up at 12 dB with every loop converged, every status bit green, and a
pre-FEC BER of 7.6e-2.

### The result worth stopping on

Look at 16 dB / +120 ppm: **pre-FEC 1.5e-4, comfortably inside the 2.4e-4 that
KP4 is specified against -- and the payload still comes out with errors.**

That is not a contradiction, it is the specification being read too loosely.
The 2.4e-4 figure assumes errors that are roughly INDEPENDENT. These are not:
they arrive in bursts, because a DFE that mis-slices feeds the wrong decision
back and corrupts the next several symbols. Most 20-block windows measure
exactly zero and an occasional one measures 2.6e-3. A burst long enough to put
more than 15 corrupted symbols into one codeword is uncorrectable no matter how
good the average looks.

**A mean pre-FEC BER is not sufficient to size a FEC. The error DISTRIBUTION is
part of the specification, and quoting the average alone is how a link passes on
paper and fails on the bench.** It is also exactly why the decoder here supports
erasures: when something else already knows where the burst was, the survivable
burst length doubles.

## FEC -- RS(544,514), the KP4 code

`fec_probe` self-tests the codec, then measures what it buys.

```
  1. codec self-test
     clean codewords ............................ ok
     1..15 symbol errors, hard decision .......... ok
     1..30 erasures ............................. ok
     2*errors + erasures <= 30 .................. ok
     beyond budget: 40/40 declared uncorrectable, 0 miscorrected

  4. burst errors with the burst LOCATION known
     burst    hard decode    burst positions erased
     15        60/60          60/60
     16         0/60          60/60
     30         0/60          60/60
     31         0/60           0/60
```

Both cliffs land exactly where the algebra says: 15 symbols hard, 30 with the
positions marked, because an erasure costs one parity symbol and an error costs
two.

**A measured negative result, kept because it is worth more than a plausible
claim.** The obvious way to get soft-decision gain is to flag samples that land
near a slicer threshold and erase those symbols. Measured, it does not pay:

```
  margin   flags    errors    covered   hard fail soft fail
  0.010    7.2      14.7      3.3       52        62
  0.020    14.8     14.8      5.8       54        75
  0.085    87.8     15.3      14.0      60        60
```

The trade needs `flags < 2 x (errors actually covered)` and the table never
satisfies it. The reason is geometric: PAM4 has three thresholds and its inner
levels sit between two of them, so the population *near* a threshold is several
times the population that *crossed* one, at every margin. This is why KP4 in
Ethernet is a hard-decision code, and why real soft gain needs a soft-decision
*code* rather than a hard code fed reliability flags.

Where erasures do pay is side information -- a burst whose location something
else already knows: a loss-of-lock flag, a lane error counter, a bring-up FSM
not yet in TRACK. That costs no false flags, and it doubles the survivable
burst.

## Measured S-parameters

`touchstone.c` reads a Touchstone v1 file; `channel.c` turns it into an impulse
response by interpolating magnitude and **unwrapped phase** separately,
extrapolating the loss trend and group delay above the measured band, and
removing the bulk propagation delay.

Three things a fitted `a*sqrt(f) + b*f` loss curve cannot express, all visible
in `ch_probe` output from a 4-port file:

```
      50.0         -27.63   <- Nyquist
      62.0         -56.30                 <- via-stub notch
  long tail:
    +  9 UI   +0.00619
    + 10 UI   +0.00657                    <- reflection echo, 2 x 55 ps
    + 11 UI   +0.00720
    + 12 UI   +0.00502
  far-end crosstalk
    peak coupling   -42.4 dB below the through peak
```

I do not have a measured file I can publish, so `s4p_gen.c` synthesises one
from physics -- skin and dielectric loss, two impedance discontinuities summed
as a geometric series, a quarter-wave open stub, and frequency-rising FEXT --
and every file it writes says **SYNTHESISED, NOT MEASURED** on its first line.
The reader takes a real vendor sweep unchanged.

The reader handles the format's genuine wart: Touchstone v1 stores a 2-port
**column major** (`S11 S21 S12 S22`) and every other port count row major.
Reading a `.s2p` row major silently swaps the through path with the reverse
one, which on a reciprocal passive channel is nearly invisible -- until it is
not. There is a unit test for it.

## Multi-lane -- eight lanes, one processor

`hw_macro.c` runs eight lanes side by side. What only appears at this scale:

- **Crosstalk.** Every lane's transmitter must run before any lane's receiver,
  because lane 0 is aggressed by the block lane 1 is sending *now*. Doing it
  lane-at-a-time shifts all the crosstalk by a block, which is invisible in a
  BER number and completely wrong.
- **Every lane must carry different data.** Give eight lanes the same PRBS seed
  and the crosstalk arriving at a victim is a filtered copy of its own signal --
  perfectly correlated, well behaved, and entirely fictional.
- **Per-lane state.** One control processor serves all eight, so every loop
  accumulator, IIR and settle counter exists per lane. Leave them global and
  the macro converges to the average of eight different channels while each
  lane reports itself converged.
- **A windowed register file.** `hal_select_lane()` gives each lane an aperture
  and keeps the per-lane firmware byte-identical. The hazard that comes with it
  is that the window is shared mutable state: an ISR that repoints it and
  returns leaves the interrupted code writing a neighbour's registers. The HAL
  saves and restores it, and there is a test that fails if it stops.
- **Loop bandwidth divided by the service rate.** A lane serviced every N
  blocks gets one control iteration every N milliseconds, so every timeout has
  to scale with how many lanes share the CPU. A single-lane bench passes
  without this and the product does not.

Eight lanes, losses spread 10 to 18 dB and reference offsets spread -200 to
+120 ppm, all serviced by one supervisor:

```
  lane  IL@Nyq   ppm     up at    pre-FEC BER   post-FEC   uncorrectable
  0     10.00    -200    422 ms   0.000e+00     clean      0 / 298
  1     11.15    -154    362 ms   0.000e+00     clean      0 / 298
  2     12.29    -109    341 ms   0.000e+00     clean      0 / 298
  3     13.44     -63    340 ms   0.000e+00     clean      0 / 298
  4     14.58     -17   2088 ms   0.000e+00     clean      0 / 298
  5     15.72     +29   2036 ms   6.165e-07     clean      0 / 298
  6     16.86     +74    never    5.009e-01     --         298 / 298
  7     18.00    +120   1062 ms   0.000e+00     clean      0 / 298

  7 of 8 lanes up
  supervisor services  3200 each -- the round robin starves nobody
  UNGUARDED RMW        0   (ok)
  W1C RMW bugs         0   (ok)
  bytes dropped        0   (ok)
```

**Seven lanes carry FEC traffic with zero pre-FEC errors and no uncorrectable
codewords.** Lane 6 sits at 16.9 dB, in the same band the single-lane sweep is
weakest in, and it is honest that the macro reports 7 of 8 rather than an
average.

`macro_sim 8 14 0 4000 2` runs the same thing with the supervisor servicing
only two lanes per block, which quarters every control loop's bandwidth and
stretches the firmware's timeouts by four to match.

## Known limitations

Stated plainly rather than hidden.

- **Above about 24 dB the link does not close.** The equaliser is 16 FFE and 8
  DFE taps against a channel whose normalised ISI at 30 dB is 1.78 UI with a
  tail still at 5% of the cursor twelve symbols out. Lengthening the equaliser
  further is the obvious next step; widening the CTLE's peaking range was tried
  and made every operating point *worse*, because the pole and zero move with
  the step size and the shape of the boost matters more than its size.
- **16 to 24 dB is marginal** -- the link comes up at every reference offset
  but the residual error rate is 1e-4 to 7e-4, so some of those points miss the
  KP4 limit. The errors are bursts rather than a raised floor (see above), and
  the front-end search finds a clean point at neighbouring losses, so this is a
  search-coverage and burst-length problem rather than a hard capability limit.
- **DFE error propagation produces bursts longer than KP4 can correct.** Real,
  correctly modelled, and the reason erasure decoding is implemented.
- **`cdr_ppm()` carries a constant offset of about -6 ppm.** It tracks with
  unity slope across -200 to +120 ppm; the offset is residual Mueller-Muller DC
  absorbed by the integrator. A real part counts phase-interpolator rollovers
  over a known interval instead of trusting a loop-internal value.
- The published `.s4p` is synthesised from physics, not measured. The reader
  takes a real one unchanged.
- Crosstalk is modelled from the off-diagonal S-parameters with
  nearest-neighbour weights, not from a full 16-port coupled extraction.

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
| ppm drift applied outside the CDR''s phase wrap | The drift is added even while the CDR is disabled during AGC, but the phase is only wrapped inside `cdr_update()`. It ran away unbounded; negative offsets drove it below zero, `cdr_sample()` clamped to index 0, and every symbol in the block read the same sample. Negative ppm failed 100% of the time. |
| AGC limit-cycled on single-block measurements | One block of mean-\|y\| is a noisy estimate. The loop stepped, overshot, stepped back, and never accumulated the consecutive in-band blocks that declare convergence -- it hunted between VGA 27 and 32 for the full timeout. Fixed with a single-pole IIR on the measurement and a deadband wider than one gain code (0.476 dB = 5.6% = ~154 units at this target). |
| **A pre-FEC BER counter that could only ever read zero** | `REG_ERR_CNT` is only incremented during training; telemetry only read it in `LS_UP`, where training is off. The headline result of the whole project was structurally incapable of being non-zero, and it hid everything below. |
| **The CDR was tapped off the equaliser output** | Mueller-Muller's entire output is `h(+1) - h(-1)`, which is exactly what the FFE and DFE exist to null. Two loops driven from one node, and the tap loop -- with 24 degrees of freedom against the timing loop's one -- wins. The detector's peak output measured under 0.005 where a working one gives 0.1 to 0.5. It was not inverted so much as absent. |
| Mueller-Muller sign inverted **again**, and the header disagreed with the .c | `cdr.h` documented `a[n-1]*y[n] - a[n]*y[n-1]`; `cdr.c` implemented the negation. Neither comment mentioned the other. |
| A lock detector that reported LOCKED while the phase swept the whole UI | Three separate reasons, and all three had to be closed: a uniformly sweeping phase also gives mean(e) ~ 0; the "frequency estimate has stopped moving" test is *guaranteed* to pass when the anti-windup clamp engages, because a clamped integrator has stopped by definition; and the wrap condition the comment claimed as independent was never in the predicate at all. |
| Loop gains sized for a detector gain of one | `zeta = kp/(2*sqrt(ki))` is the `Kd = 1` special case. At the real detector gain the loop was badly underdamped, not critically damped as the comment claimed. |
| Amplitude normaliser quadratic where the error is linear | `e /= (amp^2 + 0.55)` peaks at an amplitude of 1.11 and falls away either side -- gain varied 1.9x over the range the AGC actually visits, which is the opposite of the amplitude independence it was there to provide. At the AGC's own target it evaluated to 0.994: a divide by one. |
| The DFE fed back its own decisions during data-aided training | While the eye is closed those decisions are wrong a third of the time, so the tap gradients correlate against noise and the loop sits at its initial spike reporting convergence. |
| The training reference was not delayed by the pipeline latency | Comparing a decision against the symbol that produced it three positions later gives 1.0 bit errors per symbol -- a confident, stable 50% BER. |
| Convergence threshold not scaled when the tap count tripled | A budget of 3 codes summed over 24 taps demands each tap hold to an eighth of a code, which dither never achieves. Convergence was never declared and bring-up retried forever against a loop that had settled. |
| **The channel forgot its own memory at every block boundary** | `channel_apply` restarted the convolution from zero each call, so the first 48 UI of every block were convolved against silence. That put a 2.4e-4 floor on the training BER -- *exactly* the KP4 limit -- and made bring-up reject a receiver that was working. A model that manufactures errors at the specification limit is worse than no model. |
| Bring-up declared UP without ever measuring the error rate | Every test asked whether a loop had stopped moving; none asked whether the answer was any good. A CDR can hold a rock-steady phase at the wrong point in the eye and the taps will then converge to the best filter for that wrong phase. Measured at 12 dB: every loop converged, every status bit green, pre-FEC BER 7.6e-2. |
| Verification that was not verifying the thing that would run | Data-aided verification hands the DFE perfect feedback, so it never propagates an error. Decision-directed it does. Measured at 12 dB: zero errors data-aided, 7.7e-2 decision-directed -- a factor of a thousand between the number measured and the number shipped. |
| `build.bat` labels broken by mixed line endings | `.gitattributes` pins `*.bat` to CRLF, but a working copy with LF-only lines makes `cmd` seek to the wrong byte offset and lose its `goto` targets. The build silently ran stale binaries. |
| **Firmware crossed its own seam, twice** | `fw_bringup.c` called the hardware-side backend to raise `STAT_AGC_CONV`, and `fw_adapt.c` read-modify-wrote `REG_ADAPT_STAT`, which the map declares RO -- using a bit constant from a *different* register's namespace, so it meant the right thing only by numerical accident. Both wrote values that **nothing read**, which is why nothing ever failed. The seam is the central claim of the project and it was only ever violated to produce dead data. CI now greps `src/fw_*.c` for it, because a claim that lives in a header is a claim that drifts. |
| The HAL header made three statements its own code contradicts | It said `hal_field_set` was "wrapped in a critical section" -- it is not, and the suite's best test proves it is not, by firing an ISR at the read-modify-write and watching the update vanish. It said the ISR path used `hal_lane_push/pop` -- it uses a direct save/restore. And it named `test_lane_window_isr()` as the proof, a function that does not exist. A comment that states a guarantee is a specification; when the code does not honour it, the comment has become a defect, and one that advertises where to look. |
| `hal_lane_push` / `hal_lane_pop`: dead, and broken | Zero callers. Also wrong: `push(1); push(2); pop();` leaves the window on lane 2. It was an uncalled, incorrect implementation of the exact hazard the paragraph above it describes, advertised in the header as the mitigation. Deleted. |
| Every crosstalk aggressor shared one overlap-add tail | The crosstalk filter carries state between blocks, and that state lived on the *victim's* channel. Running three neighbours through it in turn meant the residue left by neighbour -3 at the end of a block was emitted at the start of the next one scaled by neighbour +1's coupling weight. Superposition *inside* a block was exact; only the boundary term was wrong -- which no BER number would ever show. Fixed by using linearity: sum the aggressors' waveforms first and filter once, which is the same arithmetic with one state variable instead of N, and one FFT pass instead of N. |
| The most-commented feature in the macro was arithmetically inert | Twenty lines explaining that a supervisor servicing 2 of 8 lanes divides every control loop's bandwidth by four -- attached to code where `service` was assigned `n_lanes`, making the scale factor identically 1 and `MACRO_SERVICE` unreferenced. The mechanism was real and the explanation was right; it was simply never exercised. Now a parameter, with the eight-lane case runnable both ways. |
| `channel_pulse_response()` used the streaming convolution | So a single-shot measurement started from whatever inter-block state the channel was carrying, and then left its own tail behind for the next real block. `hw_lane_init()` calls it to seed the training-reference delay, so the pollution landed on the first block of every link. |
| A metric that could not fail, still printing after being "replaced" | The README said the vacuous pre-FEC counter had been replaced by a real measurement. The real measurement was added; the old one was left in, still printing a confident `0.000e+00` two lines below it. Removed, and the telemetry field with it, because a number that cannot be non-zero is worse than no number -- it gets trusted. |
| The Touchstone reader trusted the file | A duplicated frequency row -- routine in a stitched or concatenated sweep -- makes the interpolation divide by exactly zero, and the resulting NaN propagates into every tap of the impulse response and from there into every sample, BER and lock decision. There was not one finiteness check anywhere in the project, so the run completed and printed numbers. A stray non-numeric token was worse: the parser abandoned the rest of the line but kept its partial record, so everything after it shifted by one field and what had been an imaginary part became the next point's frequency. It still returned success. All of it is external input and it is now validated, with tests that feed the parser each malformation. |

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

- **A longer equaliser.** 16 FFE and 8 DFE taps close 24 dB; 30 dB needs more,
  and the register aperture already has room for it.
- **Close the search-coverage gap at 16 dB.** The front-end operating point is
  inferred from the AGC's converged gain code, which is a good estimate and not
  a perfect one. A second refinement pass, or adapting the CTLE code directly
  against the error counter, would remove the retries.
- **An LDPC inner code**, to get the soft-decision gain that `fec_probe` shows
  a hard code fed reliability flags cannot deliver.
- **Error-propagation mitigation in the DFE** -- burst detection feeding the
  erasure decoder, which the codec already supports and nothing currently
  drives.

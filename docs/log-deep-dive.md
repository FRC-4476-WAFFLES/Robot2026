# Log deep dive — every match we have

An audit of all 113 `.wpilog` files from the 2026 season, looking for bugs and
things worth changing. Written overnight on 2026-09-07/08.

**Read the confidence markers.** Every claim here is one of:

| Marker | Meaning |
|---|---|
| **Measured** | Comes straight out of the logs. The number is what it is. |
| **Attributed** | Measured, and tied to a specific line of code or commit. |
| **Suspected** | Consistent with the evidence, not proven. Says what would settle it. |
| **Cleared** | Looked like a problem, measured, and is not one. Recorded so nobody re-derives it. |

**Two codebases are involved.** The logs were produced by `main` as it stood at
each event. The code being audited is `offseason`, which has diverged. Where a
finding applies to only one, it says so.

---

## Scope and method

| | |
|---|---|
| Logs analysed | 113 (106 readable; 7 are zero-byte stubs) |
| FMS matches | 25 — 10 at ONWEL, 15 at Houston/Milstein |
| Non-match logs | 81 — pits, practice field, bench |
| Total | 4.0 GB of `.wpilog` |

**Events.** `EventName` in the logs gives ONWEL (Apr 11–13), ONCMP1 (Apr 21–23)
and MILSTEIN (Apr 29 – May 2).

**The ONCMP1 logs are not matches.** All ten have `FMSAttached = false`. They are
practice/pit sessions dated *after* the Ontario event, which the commit messages
place around Apr 17–19. Any earlier statement in this repo treating them as
Ontario match data is wrong; they are used here only where a non-match log is
still valid evidence (code behaviour, current draw), never for per-match rates.

**Tooling.** Everything came from a streaming WPILOG reader written for this,
emitting one JSON summary per log, plus targeted single-purpose passes. The
scripts live in the session scratchpad rather than the repo — they are analysis,
not robot code. `LogReview` gained one change (see Corrections).

**Ball counting.** There is no ball sensor in the log, so balls are counted as
peaks in flywheel stator current. The speed dip is the obvious signal and the
wrong one: flywheel mass was added on 16 April, so the same ball costs less speed
afterwards and a fixed threshold would "find" fewer balls purely from the inertia
change. Current does not have that problem — a ball takes fixed momentum out of
the wheel whatever the wheel weighs. The detector's one free knob (peak
prominence) was calibrated against scouted teleop scores across ten Milstein
matches: correlation r = 0.76–0.82 at every setting, and the setting where points
per detected ball lands below 1.0 (as it must, since not every ball scores)
agrees with the independently measured 75–88% in-tolerance rate.

---

## Headline findings, ranked

| # | Finding | Confidence | Where |
|---|---|---|---|
| 1 | Brownouts are almost entirely the drivetrain; the shooter draws ~0 A at the median brownout | **Measured** | [Power](#1-power-the-drivetrain-is-the-brownout-and-nothing-else-is) |
| 2 | The feeder's 25 A supply cap costs ~13 s of match time per match | **Attributed** | [Fire rate](#8-fire-rate-what-we-can-actually-do) |
| 2b | Half of Houston's fire windows are closed by the turret, not the driver | **Measured** | [Fire rate](#lever-2-fewer-longer-fire-windows) |
| 2c | The turret's motion profile is computed every loop and discarded | **Attributed** | [Fire rate](#lever-3-the-turrets-motion-profile-is-computed-and-discarded) |
| 3 | `IGNORE_SINGLE_TAG` throws away nearly all of the frame camera | **Measured** | [Vision](#2-vision-one-constant-is-eating-a-whole-camera) |
| 3b | The bump high-trust window causes 88% of all pose jumps over a metre, up to 9.34 m | **Measured** | [Vision](#24-the-bump-high-trust-window-causes-multi-metre-teleports) |
| 4 | Vision "dropouts" are roboRIO loop stalls, not cameras | **Attributed** | [Vision](#22-the-dropouts-are-the-loop-not-the-cameras) |
| 5 | Half of every loop overrun is outside user code, logging and GC | **Measured**, cause unknown | [Loop](#3-loop-timing) |
| 6 | Six `periodic()` methods return early inside an `EpochTimer` block | **Attributed** | [Bugs](#4-bugs-found-by-reading-the-code) |
| 7 | `ChassisSpeeds.discretize` uses a hardcoded 0.02 s against a 21 ms median loop | **Attributed** | [Bugs](#4-bugs-found-by-reading-the-code) |
| 8 | The indexer agitation cycle in `shootCommand` is a no-op | **Attributed** | [Bugs](#4-bugs-found-by-reading-the-code) |
| 9 | The hood zeroes *after* enable, every match, during the start of auto | **Measured** | [Bugs](#4-bugs-found-by-reading-the-code) |
| 10 | RIO CAN bus hits 100% utilization in every match | **Measured**, impact unproven | [CAN](#5-can-bus) |

---

## 1. Power: the drivetrain IS the brownout, and nothing else is

**Measured**, across 159 brownouts in 25 FMS matches.

**All 159 happened in teleop. Zero in autonomous.**

Current at the instant of each brownout:

| Source | median | p90 | max |
|---|---|---|---|
| PDH total | 188 A | 416 A | 492 A |
| **Drive, 4 modules summed** | **176 A** | 271 A | 378 A |
| Flywheel | **0 A** | 38 A | 68 A |
| Feeder | **0 A** | 29 A | 60 A |
| Spindexer | 3 A | 53 A | 79 A |

At the median brownout the drivetrain is drawing 176 A of a 188 A total and the
entire shooter is drawing 3 A. The drive supply limit is 45 A per module
(`ModuleIOTalonFX.java:104`), so four modules cap at 180 A — the median brownout
sits **exactly on that cap**.

This is the single most important number in this document, because it says the
offseason `PowerManager` is aimed at the right subsystem, and it says the
shooter-side current limits added at Ontario were aimed at the wrong one.

Brownout threshold is lowered to 6.5 V in `Robot.java:118`. Battery minimum per
match is 6.1–7.2 V, so matches are genuinely reaching the floor.

Brownouts per match: ONWEL median 6, Houston median 2. Worst single matches:
ONWEL q5 19, ONWEL q31 19, Houston q4 20, Houston q44 12.

---

## 2. Vision: one constant is eating a whole camera

### 2.1 `IGNORE_SINGLE_TAG`

**Measured**, time-weighted (the log records on change, so counting records
counts transitions, not time — an earlier pass of mine got this wrong and
reported a meaningless 50/50).

`VisionConstants.IGNORE_SINGLE_TAG = true`. In `TagCamera.java:233` it discards
every single-tag frame, and in `TagCamera.java:314` it also short-circuits
`calculateGyroEstimate()` — the fallback built for exactly the single-tag case.

Share of enabled time each camera spent with N tags in view, and how often it
produced a usable estimate:

| match | frame: 1 tag | frame: 2+ tags | frame MEGATAG | turret MEGATAG |
|---|---|---|---|---|
| ONWEL p3 | 32.9% | 4.6% | **3.7%** | 47.9% |
| ONWEL p4 | 48.2% | 2.8% | **2.3%** | 28.4% |
| ONCMP1 00-55 | **96.4%** | 1.9% | **1.2%** | 63.9% |
| ONCMP1 01-16 | **97.0%** | 2.8% | **2.4%** | 80.7% |
| Houston q76 | 36.0% | 2.3% | **2.0%** | 33.7% |
| Houston q87 | 37.7% | 12.9% | **3.7%** | 51.5% |
| Houston q117 | 61.3% | 4.0% | **1.7%** | 37.2% |
| Houston e6 | 60.2% | 3.4% | **2.0%** | 34.5% |

**`limelight-frame` produces a usable pose 1.2–3.7% of the time.** It is
carrying almost none of the localisation load. The turret camera loses 11–64% of
its time the same way.

The two filters I expected to matter do not:

| filter | threshold | share of enabled time exceeded |
|---|---|---|
| `MAX_TURRET_YAW_RATE_ROTATIONS` | 2 rot/s | 0.21 – 1.18% |
| `MAX_YAW_RATE_RADS` | 5 rad/s | 0.00 – 0.30% |

One detail worth noting: turret velocity p99 is 2.00–2.10 rot/s — the threshold
sits exactly at the turret's own top speed, so it rejects precisely when the
turret is slewing to a new target. Small in total, concentrated at the worst
moment.

**Dead code today.** Everything under the single-tag branch —
`isAmbiguityAcceptable`, `MIN_TAG_AREA_SINGLE_TAG`, `isYawDifferenceAcceptable`,
and all of `calculateGyroEstimate` — is unreachable while the flag is true.

**Not a recommendation to just flip it.** The flag was presumably set because
single-tag estimates were bad. But the guards written to make single-tag safe
are sitting right there unused, and the cost of the blanket rule is now
quantified. Worth an explicit decision rather than an inherited default.

### 2.2 The dropouts are the loop, not the cameras

**Attributed.** In the last Houston match (e6) there were 15 stretches totalling
38 s of the 140 s match (27%) with no accepted estimate from either camera.

Of that, only ~5 s is the cameras going away: 19 brief simultaneous
`IsAlive = false` drops of 0.1–0.4 s each. Two cameras on separate IPs do not
fail in lockstep — and every one of those drops coincides with a loop overrun.

`LimelightIO.java:35`:

```java
inputs.isAlive = (Timer.getFPGATimestamp() - lastHeartbeatTime) < VisionConstants.LL_HEARTBEAT_MIN_FREQ;
```

`LL_HEARTBEAT_MIN_FREQ` is 0.5 s. This is wall-clock staleness of a value the
robot code only samples when it runs, so **any loop stall over 0.5 s marks both
cameras dead**. It conflates "the camera is gone" with "we were late".

The other ~33 s is section 2.1 — cameras alive, tag in view, estimate discarded.

Vision is also not special here. It is the only thing with a liveness timeout,
so it is the only thing that announces the stall. In the same match:

| signal | median gap | worst gap |
|---|---|---|
| gyro | 28 ms | 2163 ms |
| pose estimate | 23 ms | 12609 ms |
| turret camera | 23 ms | 2855 ms |
| flywheel | 26 ms | — |

### 2.3 Stale inputs on the early-return path

**Attributed**, latent. `LimelightIO.updateInputs` returns early on `!isAlive`
(line 38) and `!canSeeTag` (line 47) **without clearing** `inputs.canSeeTag`,
`inputs.megatagResult`, `inputs.rawFiducials` or `inputs.fiducialArrayLength`.
The inputs object persists between loops, so those keep last-good values.

Harmless today because `TagCamera` checks `isAlive` and `canSeeTag` in that
order before touching anything else. It is a trap for the next reader, and it
violates the repo's own rule that inputs are the source of truth.

---

## 2.4 The bump high-trust window causes multi-metre teleports

**Measured**, across all 25 FMS matches plus practice — 6503 s of enabled time.

`Vision.java:163` deliberately multiplies the standard deviations of the first
few estimates after a bump crossing by 0.1:

```java
if (highTrustEstimatesLeft > 0) {
  chosenDeviations = chosenDeviations.times(0.1); // Essentially teleport to the first few things we see
  highTrustEstimatesLeft--;
}
```

The comment is accurate. It teleports.

| pose jump size | total | of those, with the boost active |
|---|---|---|
| > 0.25 m | 1574 | 554 (35%) |
| > 0.5 m | 195 | **162 (83%)** |
| > 1.0 m | 60 | **53 (88%)** |
| > 2.0 m | 19 | **17 (89%)** |

The boost is active **12.4% of enabled time** but accounts for 88% of every
correction over a metre. The largest observed are **9.34 m** and **6.96 m** —
more than half the field, in one loop.

**Two constants are compounding.** `BUMP_HIGH_TRUST_ESTIMATES = 5` counts
*accepted* estimates, and `highTrustEstimatesLeft--` only runs when one is
actually fused. Section 2.1 shows accepted estimates are scarce, so a window
meant to cover "the next 5 things we see" stretches out to an eighth of the
match. Fixing `IGNORE_SINGLE_TAG` would shorten this window as a side effect;
leaving it alone keeps the window wide open.

**There is no bound on a single correction.** Any estimate passing
`isValidPose` and `isValidStdevs` is fused however far it moves the robot.

This is also the best available explanation for the two failed autonomous
routines (section 7): ONWEL q8 took a **1.42 m** correction 3.4 s into auto and
scored 3 balls against a median of 55; Houston q4 took four corrections of
0.33-0.43 m in the first 6 s and scored 5.

Cheapest guard: reject, or heavily de-weight, any correction that moves the pose
more than some bound in one step while odometry is otherwise healthy — and do
not apply the 10x boost during autonomous at all.

---

## 3. Loop timing

**Measured**, median across FMS matches:

| | ONWEL | Houston |
|---|---|---|
| cycle p50 | 17.9 ms | 19.8 ms |
| cycle p99 | 66.2 ms | 75.9 ms |
| loops over 40 ms | 660 | 1080 |
| loops over 100 ms | 36 | 44 |
| user code p50 | 14.4 ms | 15.8 ms |
| logging p50 | 3.2 ms | 3.7 ms |
| unaccounted p99 | 32.9 ms | 36.8 ms |

The loop is over its 20 ms budget for a meaningful fraction of every match, and
it got worse between events.

**Half the overrun is unexplained.** In the e6 match, of 206 in-match overruns
over 60 ms, the median was 74 ms — of which user code is 27 ms and logging is
10 ms. The remaining ~37 ms is not user code, not logging, and not GC.

Ruled out as causes (**cleared**, all measured):

| Suspect | Evidence against |
|---|---|
| Garbage collection | 0 collections during the e6 match; 1 of 206 overruns was GC-related, at the buzzer |
| CAN saturation | utilization 0.37 at overruns vs 0.38 normal; 0 receive errors, 0 bus-off |
| Battery / brownout | 10.30 V at overruns vs 10.31 V at normal loops |
| Shooting | 4.1% overrun rate while shooting vs 5.1% while not |
| CPU temperature | 43–47 °C |
| AdvantageScope disconnects | 32 in-match disconnects, but the timing correlation with camera drops is **not distinguishable from chance** |

Note the last one: AdvantageScope *was* connected over the field radio during
matches and thrashing (32 connect/disconnect cycles in 140 s), alongside Elastic.
That is worth stopping on its own principle, but the data does not support
blaming it for the stalls, and I am not going to claim it does.

GC does show up elsewhere: `gcMaxMS` is 236–333 ms in **every** match log. Those
pauses land outside the enabled window in the matches examined, but a 300 ms
stop-the-world pause is one bad roll away from landing inside one.

---

## 4. Bugs found by reading the code

All line numbers are `offseason` unless stated.

### 4.1 Early `return` inside an `EpochTimer` block — 6 sites

**Attributed.** `EpochTimer.BeginEpoch(name)` … `EndEpoch(name)` brackets each
subsystem's `periodic`, and `EndEpoch` is what writes
`RealOutputs/RobotState/Timing (ms)/<name>`. These return without reaching it:

| File | Line | Condition |
|---|---|---|
| `Flywheel.java` | 71 | `!robotEnabled()` |
| `Indexer.java` | 62 | `!robotEnabled()` |
| `Intake.java` | 68 | `!robotEnabled()` |
| `Hood.java` | 46 | `!hoodZeroed` |
| `Turret.java` | 69 | `!robotEnabled()` |
| `Turret.java` | 74 | `state == TurretState.BRAKE` |

Effect: the timing output silently stops updating on those loops. Most are
disabled-only, but **`Turret` BRAKE happens during matches**, so `Timing (ms)/
Turret` goes stale mid-match and the profiling data is quietly wrong exactly
where someone would look.

Fix is `try/finally` around the body, or moving `EndEpoch` into a `finally`.

### 4.2 `ChassisSpeeds.discretize` with a hardcoded period

**Attributed.** `Drive.java:271`:

```java
ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
```

`discretize` compensates the coupling between translating and rotating over one
loop period. The measured loop is p50 21 ms, p90 32–44 ms, p99 66 ms. It is
systematically under-compensating, and worst exactly when the robot is busiest —
which is when you are most likely to be translating and rotating together.

Fix: pass the measured period, or at minimum `CodeConstants.PERIODIC_LOOP_TIME`
so there is one place to change it.

### 4.3 The indexer agitation cycle does nothing

**Attributed.** `ShooterCommands.java:26-33`:

```java
if (agitationTimer.get() % AGITATION_CYCLE_TIME < 1.2) {
  RobotContainer.indexer.runIndexer(IndexerState.RUN);
} else {
  RobotContainer.indexer.runIndexer(IndexerState.RUNSLOW);
}
```

The two states are identical, and were at both events:

```java
RUN(15, 40),    RUNSLOW(15, 40),     // ONWEL
RUN(13.5, 65),  RUNSLOW(13.5, 65),   // champs
```

Both branches do the same thing. Either `RUNSLOW` was meant to be slower and the
constants drifted together, or the whole cycle is leftover. Right now it is a
timer, a modulo and a branch that compute nothing.

### 4.4 Hood zeroes after enable, every match

**Measured** across all 57 champs logs. `Hood.periodic` drives `-0.1` duty until
a stall (`torqueCurrent < -35` debounced 0.1 s) and returns early until zeroed.
Motors do not move while disabled, so zeroing cannot complete until the match
starts.

In every single match the hood zeroed 0.1–0.6 s **after** enable. That window is
the start of autonomous, and during it the hood is being driven into its stop
rather than tracking a setpoint.

There is also no timeout and no alert: if the stall is never detected, the hood
drives down forever and `periodic` returns early every loop, with nothing
reported. (**Cleared**: the two logs that never zeroed were both 0.1 s pit
enables, not a real failure.)

### 4.5 Missing-sample defaults to reject

**Suspected**, low impact. `TagCamera.java:225` and `:259` both do:

```java
...getTurretVelocityTimestamp(t).orElse(Double.POSITIVE_INFINITY)) > THRESHOLD
```

An empty buffer lookup becomes `+Inf`, which fails the check, so a *missing*
sample silently rejects the estimate. The buffers hold
`TELEMETRY_LOOKBACK_TIME = 1 s`, and `getSample` returns empty only for
timestamps older than the buffer, so this should be rare. But during a 0.8 s loop
stall the buffer stops being written, and the reject is silent either way.
Worth logging the branch before deciding it does not matter.

### 4.6 `aimToTag()` is `aimToHub()`

**Attributed**, cosmetic. `ShotPlanner.java` — both aim at
`Hub.topCenterPoint`. The real difference is that `TARGET_TAG` suppresses the
shoot-on-move lead (`ShotPlanner.java:115`). The name says the aim point differs;
it does not. Two identical bodies that must stay identical is a future bug.

### 4.7 `farFromStart` is evaluated once and goes stale

**Attributed**, minor. `Adaptable.java:254` / `AdaptableSwitch.java:146` set the
alert at auto *generation* time; only `InvalidateCache()` clears it. Reposition
the robot after generating and the error stays up while being false.

**Cleared, importantly**: this alert appears in 39 logs but was **never live at
the moment autonomous started** in any of the 25 FMS matches. It is pre-match
placement noise, not a real fault. Do not go chasing it.

---

## 5. CAN bus

**Measured.** RIO CAN utilization: p50 0.36–0.38, **p99 1.00 in every match**,
with 4.1–4.4% of samples at or above 0.9.

Error counters are clean in the matches examined (0 receive errors, 0 transmit
errors, 0 bus-off, `TxFull` unchanged), and utilization does not correlate with
loop overruns. So this is **not currently proven to cost anything** — but a bus
that saturates several percent of the time has no headroom left for a new device
or a higher status-frame rate.

Logged alerts across all logs: `CAN fault detected` in 26, `RIO CAN bus error`
in 17, `CANivore bus error` in 3.

---

## 6. Pose and odometry

**Measured.** Vision corrections that move the pose more than 25 cm in under
half a second:

| | ONWEL | Houston |
|---|---|---|
| per match, median | 16 | 28 |
| worst match | 57 (q12) | 87 (e3 2nd) |

During **autonomous** specifically, 1–8 corrections per match, worst single jump
0.28–1.42 m. A 1.4 m teleport mid-path (ONWEL q8) is enough to derail a
trajectory follower.

Related, **attributed**: `Drive.java:258` pushes the pose history sample as

```java
RobotContainer.state.updateOdometry(Timer.getTimestamp(), getPose(), ...)
```

— stamped with *now*, while the pose it carries was integrated from odometry
samples up to a loop old. That buffer is what vision latency compensation reads
(`getPoseAtTimestamp`), so there is a small systematic bias in every correction,
and a large one during a stall when a single sample gets stamped "now".

---

## 7. Per-match table

FMS matches only. `>100` is loops over 100 ms; `minV` is the lowest battery
voltage seen; `b/s` is balls per second while the fire command was open.

| log | event | match | enabled | p99 ms | >100 | brownouts | min V | balls | b/s | pose jumps |
|---|---|---|---|---|---|---|---|---|---|---|
| p9 | Houston | p9 | 71 | 55 | 32 | 0 | 7.17 | 72 | 4.44 | 16 |
| q4 | Houston | q4 | 161 | 64 | 36 | **20** | 6.10 | 288 | 4.66 | 13 |
| q14 | Houston | q14 | 161 | 76 | 44 | 3 | 6.42 | 231 | 4.34 | 21 |
| q26 | Houston | q26 | 160 | 64 | 34 | 0 | 7.14 | 115 | 4.42 | 21 |
| q44 | Houston | q44 | 161 | 76 | 44 | **12** | 6.25 | 259 | 4.41 | 41 |
| q59 | Houston | q59 | 161 | 70 | 41 | 1 | 6.43 | 308 | 4.62 | 18 |
| q68 | Houston | q68 | 161 | 76 | 40 | 2 | 6.67 | 191 | 4.27 | 29 |
| q76 | Houston | q76 | 161 | 66 | 38 | 2 | 6.39 | 266 | 4.49 | 20 |
| q87 | Houston | q87 | 161 | 79 | 58 | 0 | 6.82 | 245 | 4.36 | 29 |
| q105 | Houston | q105 | 160 | 67 | 38 | 7 | 6.32 | 285 | 4.27 | 29 |
| q117 | Houston | q117 | 161 | 88 | 53 | 0 | 6.90 | 223 | 4.04 | 23 |
| e3 (1) | Houston | e3 | 160 | 69 | 54 | 0 | 6.83 | 225 | 4.29 | 38 |
| e3 (2) | Houston | e3 | 155 | 77 | 88 | 1 | 6.56 | 112 | 3.95 | **87** |
| e3 (3) | Houston | e3 | 161 | 91 | 77 | 8 | 6.18 | 202 | 4.12 | 28 |
| e6 | Houston | e6 | 161 | **97** | **85** | 5 | 6.43 | 188 | **3.59** | 41 |
| p3 | ONWEL | p3 | 160 | 67 | 58 | 6 | 6.44 | 171 | 5.13 | 37 |
| p4 | ONWEL | p4 | 160 | 99 | **114** | 9 | 6.35 | 306 | 5.29 | 28 |
| 14-14-26 | ONWEL | — | 161 | 89 | 94 | **35** | 6.28 | 317 | 5.18 | 11 |
| q5 | ONWEL | q5 | 160 | 69 | 50 | **19** | 6.44 | 299 | 5.46 | 7 |
| q8 | ONWEL | q8 | 144 | 57 | 23 | 0 | 6.99 | 210 | 5.28 | 15 |
| q12 | ONWEL | q12 | 140 | 63 | 39 | 5 | 6.84 | 47 | 4.50 | **57** |
| q15 | ONWEL | q15 | 163 | 70 | 28 | 1 | 7.03 | 203 | 5.69 | 17 |
| q20 | ONWEL | q20 | 163 | 58 | 32 | 1 | 6.83 | 146 | 5.74 | 22 |
| q24 | ONWEL | q24 | 161 | 65 | 32 | 3 | 6.78 | 222 | 5.55 | 13 |
| q31 | ONWEL | q31 | 161 | 64 | 33 | **19** | 6.43 | 307 | 6.29 | 14 |

Outliers worth a look: **q12** scored 47 balls with 57 pose jumps — the worst
localisation match we have. **e6** is the slowest firing and the worst loop
timing. **e3 (2)** has 87 pose jumps and 88 loop overruns over 100 ms.

---

## 8. Fire rate: what we can actually do

### What changed, and what it cost

**Attributed**, with a dated commit.

| | ONWEL (10 matches) | Houston (15 matches) |
|---|---|---|
| Balls per match | 216 | **225** |
| Balls/s while firing | 5.38 | 4.34 (**−19%**) |
| Ball-to-ball gap | 0.129 s → 7.73/s | 0.183 s → 5.46/s (**−29%**) |
| Fire windows per match | 19 | **36** |
| Trigger open per match | 40 s | **53 s** |

The cause is `a55caae` "End of day 2 oncmp", 2026-04-18, which added to
`IndexerIOTalonFX.configureFeederMotors()`:

```java
.withSupplyCurrentLimit(25)
.withSupplyCurrentLimitEnable(true)
```

Before it, the feeder had **no supply limit at all** (stator 120 A only).

Measured feeder supply current during firing:

| | ONWEL | Houston |
|---|---|---|
| p50 | 17.9 A | **24.2 A** |
| p90 | 30.8 A | **26.8 A** |
| max | 79.4 A | 33.2 A |
| speed vs goal | 37.8 / 40 = **94%** | 42.3 / 65 = **66%** |

The Houston distribution is clamped tight around the limit — a saturated
limiter. The feeder can still spin fast when empty (p95 reaches its 65 rps goal)
but lacks the torque to push a ball through consistently, which is why its
velocity swings 23–64 rps at Houston against a steady 35.7–39.9 at ONWEL.

**The honest cost is time, not points.** Balls per match went slightly *up*
(216 → 225) because the drive team compensated by holding the trigger 33% longer.
The cap cost roughly **13 seconds of match time per match** — 13 seconds not
spent intaking, repositioning, or defending.

### Lever 1: the feeder supply cap

**The case for removing it.**

The cap was almost certainly added to fight brownouts. Section 1 says it is
aimed at the wrong subsystem: **at the median brownout the feeder draws 0 A.**
Even at the 90th percentile it is 29 A against the drivetrain's 271 A.

**Do this together with capping the drivetrain, not alone.** Removing the feeder
cap without capping the drivetrain gives back the brownouts that motivated it.
`PowerManager` on `offseason` already does the drivetrain half and has never been
on a robot — that is the thing to bring up first.

### Lever 2: fewer, longer fire windows

**Measured**, across the qualification matches of each event.

The rate while the trigger is open (4.34/s) is well below the sustained
ball-to-ball rate (5.46/s). The difference is dead time at the edges of each fire
window:

| | windows | trigger open | startup to 1st ball | tail after last ball |
|---|---|---|---|---|
| ONWEL quals | 128 | 255 s | 25 s (**10%**) | 11 s (4%) |
| Houston quals | 333 | 548 s | 67 s (**12%**) | 37 s (**7%**) |

Houston opens the trigger **2.6x more often** for only 2.1x the trigger-open
time, so it pays the per-window startup cost far more often. That is roughly
**9.5 s per match** thrown away purely on window edges.

So: what closes the windows?

| reason the window closed | ONWEL | Houston |
|---|---|---|
| driver released (or other) | 72 (59%) | 164 (50%) |
| **turret dropped out of `atGoal`** | 44 (36%) | **159 (49%)** |
| hub shift closed | 7 (6%) | 2 (1%) |

**Half of Houston's fire windows are closed by the turret, not the driver.**
`canFire()` includes `turret.atGoal()` with no debounce, so a momentary excursion
kills the command and the next one pays the startup latency again.

`TurretConstants.POSITION_TOLERANCE` is **12 degrees** — already wide — and
`atGoal()` compares against `latestGoalState`, which `Turret.java:105` sets to
the *final* goal rather than the motion profile's current state. So dropping out
means the turret is genuinely lagging its target by over 12 degrees, which is the
tracking problem, not a tolerance problem.

Cheapest fix is a falling debounce on the `turret.atGoal()` term inside
`canFire()`, exactly as was done for the flywheel. That stops the command
churning without loosening the aim requirement.

### Lever 3: the turret's motion profile is computed and discarded

**Attributed.** `Turret.java:96-105`:

```java
profileState = profile.calculate(CodeConstants.PERIODIC_LOOP_TIME, profileState, goalState);
...
// runSetpoint(profileState.position, profileState.velocity);
runSetpoint(goalState.position, goalState.velocity);
```

The trapezoidal profile is evaluated every loop, logged under
`Turret/MotionProfile/*`, and then not used — the raw goal is commanded instead.
Measured turret velocity p99 is 2.00-2.10 rot/s across every match, i.e. the
turret is saturating its own top speed rather than following a profile.

Whether commanding the goal directly is deliberate (the commented-out line
suggests it was a decision) or a leftover, right now the profile is pure cost and
the `Turret/MotionProfile/*` outputs describe motion that never happens. Either
use it or delete it — and note that lever 2 above is about the turret not
keeping up, which is exactly what a profile would inform.

### Everything, ranked

| # | Change | Expected gain | Confidence | Risk / precondition |
|---|---|---|---|---|
| 1 | Cap the **drivetrain** while shooting (`PowerManager`) | fewer brownouts, no fire-rate cost | **High** | Built on `offseason`, never run on a robot. Do this first. |
| 2 | Raise/remove the feeder supply cap (25 A) | toward 5.4 balls/s, ~13 s/match back | **High** — direct A/B across events | Adds ≤60 A peak to a bus whose problem is 176 A of drivetrain. Needs 1 first. |
| 3 | Falling debounce on `turret.atGoal()` in `canFire()` | up to ~9.5 s/match of window edges | **Medium-High** — 49% of windows measured | Must not loosen the actual aim requirement |
| 4 | Use or delete the turret motion profile | clarity; possibly better tracking | **Medium** | Changing what is commanded needs field time |
| 5 | Make `RUNSLOW` slower, or delete the branch | removes a no-op | **High** | None |
| 6 | Re-check the feeder velocity goal after 2 | 65 rps is only reachable with the torque to get there | Medium | Needs a real robot |

Items 1 and 2 are the current-limit story. Item 3 is independent of it and is
roughly the same size — worth doing regardless of what happens with power.

### What is *not* the limiter

**Cleared**, all measured at the moment of each ball:

| Candidate | Evidence |
|---|---|
| Flywheel recovery | 2.3 rps below goal at ONWEL vs 2.6–3.4 at Houston, and the 75th percentile is *better* at Houston (4.2–4.9 vs 5.9–6.5) |
| Flywheel mass | Recovery median 0.02 s at both events, against a 0.190 s ball gap. A ball takes fixed energy; the motor restores it at fixed power; recovery time is roughly mass-independent |
| Spindexer | Actual speed essentially unchanged: 13.2 rps (ONWEL) vs 13.1 (Houston) |
| The `atSetpoint` debounce | `canFire()` is byte-identical between events and hub shots bypass the flywheel check entirely (`shooterState == TARGET_HUB ? true : atSetpoint()`) |

The debounce *does* explain the passing gap moving 0.50 s → 0.72 s — a 0.25 s
debounce, almost exactly. It cannot touch hub shots.

---

## 9. Corrections to things stated earlier in this repo

Recorded so they are not believed twice.

| Claim | Correction |
|---|---|
| "Shot rate dropped 37%" | Contaminated by non-FMS practice logs. FMS matches only: **19%** on balls/s while firing, 29% on sustained gap. |
| "`logReview shots` gives balls per second" | It counts rising edges of `Commands/Fire shot` — how often the shooter is *let go*, not how often a ball leaves. Renamed and split by state in `LogReview` (commit `c53b303`). |
| "The Ontario logs are champs matches" | All ten have `FMSAttached = false`. They are practice sessions dated after the event. |
| "A feeder dragged to 65% of goal must be passing more material" | Sloppy. The goal went 40 → 65 rps *and* the gains changed, so more droop is expected at the same ball rate. |
| "AdvantageScope disconnects caused the camera dropouts" | Tested against a null model; **not distinguishable from chance**. |
| "Estimate types are 50/50 NONE/MEGATAG" | Counted transitions, not time. Time-weighted it is 96–99% NONE for the frame camera. |
| "GC pauses cause the loop stalls" | 0 collections during the match examined; 1 of 206 overruns was GC-related. |

---

## 10. What I could not determine

Stated plainly rather than guessed at.

- **What consumes the other half of every loop overrun.** Not user code, not
  logging, not GC, not CAN, not battery, not temperature, not shooting. It is
  below the level the robot code can see. Settling it needs RIO-side profiling,
  not another log pass.
- **Why the minimum ball spacing rose 37%** beyond the feeder cap being the
  obvious candidate. The cap is attributed by A/B across events and a dated
  commit, but no logged signal shows the ball actually taking longer to transit —
  spindexer speed is unchanged and feeder speed is *higher*. Ball compression
  against the heavier wheel and feeder slip are both consistent and neither is
  logged.
- **Whether the 100% CAN utilization spikes cost anything.** No error counters
  move and no correlation with overruns. Cannot rule it in or out.

---

## 11. Suggested next steps

In the order I would do them.

1. **Bring up `PowerManager` on blocks** (drivetrain capping). It is the fix for
   the one thing measurement says is actually hurting us, and it has never run on
   a robot. Watch `Power/Requested State` against `Power/Applied State`.
2. **Then raise the feeder supply cap** and re-measure balls/s. One variable at a
   time, and only after 1 is protecting the bus.
3. **Debounce `turret.atGoal()` in `canFire()`.** Independent of the power work,
   roughly the same size, and half of Houston's fire windows died to it.
4. **Fix the six `EpochTimer` early returns.** Ten minutes, and it makes the
   timing telemetry trustworthy — which matters because of finding 5.
5. **Bound a single vision correction, and drop the 10x bump boost in auto.**
   88% of every correction over a metre comes from that window, and it is the
   best explanation we have for the two autos that failed.
6. **Decide `IGNORE_SINGLE_TAG` deliberately.** The guards to make single-tag
   safe already exist and are dead. This is worth an experiment on the practice
   field, not a code change made blind.
7. **Separate "camera dead" from "we were late"** in `LimelightIO`. Compare
   heartbeat progression against loops actually run, rather than wall clock.
8. **Pass a real period to `ChassisSpeeds.discretize`.**
9. **Stop running AdvantageScope over the field radio during matches.** The data
   does not blame it for the stalls, but there is no upside and the client was
   thrashing 32 times in 140 seconds.

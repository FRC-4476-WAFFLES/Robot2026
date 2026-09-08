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

**A knock-on effect.** `Vision.combineEstimates` — the inverse-variance fusion
of both cameras, borrowed from 254 — only runs when *both* cameras produce an
estimate in the same loop. With the frame camera accepted 248 times against the
turret's 3086 in a median Houston match, that path can run at most ~8% of the
time. The most careful piece of code in the vision stack is bypassed by a
constant. (It is correct, for what it is worth: the latency compensation and the
NaN guards both check out.)

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
| ~~AdvantageScope disconnects~~ | **Retracted — this was the cause. See section 2.4.** |

Note the last one: AdvantageScope *was* connected over the field radio during
matches and thrashing (32 connect/disconnect cycles in 140 s), alongside Elastic.
That is worth stopping on its own principle, but the data does not support
blaming it for the stalls, and I am not going to claim it does.

### 2.4 The cameras never went away — the roboRIO stopped reading them

**Measured**, across all 40 milstein logs.

Both limelights get declared dead on the same loop. That cannot be two cameras
failing, and the log can prove it: the limelights are NetworkTables *clients* of
the roboRIO, so their real connection state is recorded independently of what
the vision code concluded.

In e6, **all 35** camera-dead events happen while that camera's NT client is
still connected. Across every match, limelight NT drops total **1**. The cameras
were there the whole time. `LimelightIO` computes `isAlive` from a heartbeat
robot code has to read, so anything that stops the loop reading NT marks every
camera dead at once.

What stops it is AdvantageScope attached over the field radio.

| | AdvantageScope attached | detached |
|---|---|---|
| e6 loop p50 | 27.3 ms | 21.2 ms |
| e6 loops over 60 ms | **8.0%** | 1.8% |
| e6 worst loop | 387 ms | 113 ms |
| e3 (15:06) loops over 60 ms | **10.5%** | 1.3% |
| e3 (14:34) loops over 60 ms | **7.6%** | 0.6% |

Same match, same robot, same battery, same radio — compared against itself with
the dashboard on and off. And the camera deaths are not merely correlated, they
are locked to the *reconnects*:

| match | camera deaths | within 1 s of a reconnect | chance would give |
|---|---|---|---|
| e3 (14:34) | 28 | **27 (96%)** | 0-5 |
| e3 (15:06) | 43 | **40 (93%)** | 8-18 |
| e6 | 35 | **35 (100%)** | 6-15 |

Median gap 0.55-0.58 s, which is `LL_HEARTBEAT_MIN_FREQ` exactly.

**When it started.** The dashboard's IP tells the story:

| match | AdvantageScope IP | connect/disconnect cycles | camera deaths |
|---|---|---|---|
| through q59 | 10.44.76.200 (static, pit tether) | 0 | 0 |
| q68 - q105 (May 1) | .24, .25, .26 (**DHCP, over the radio**) | 1 | 0 |
| q117 | .26 | 9 | 1 |
| e3, e3, e6 (May 2) | .27, .28, .29 | **47, 83, 117** | 28, 43, 35 |

Someone started leaving AdvantageScope connected over the field WiFi partway
through May 1, and it got worse every match through elims.

**It is the thrashing, not the attachment.** A *stable* AdvantageScope
connection over the radio costs loop time but kills nothing:

| match | attached | reconnects/s | over 60 ms attached | detached | camera deaths |
|---|---|---|---|---|---|
| q68 | 168 s | 0.01 | 3.0% | 0.5% | **0** |
| q105 | 167 s | 0.01 | 2.8% | 0.4% | **0** |
| e3 (14:00) | 167 s | 0.01 | 3.2% | 0.5% | **0** |
| q117 | 166 s | 0.05 | 5.0% | 0.8% | 1 |
| e3 (14:34) | 135 s | 0.35 | 7.6% | 0.6% | 28 |
| e3 (15:06) | 123 s | **0.68** | **10.5%** | 1.3% | **43** |
| e6 | 106 s | **0.73** | 8.0% | 1.8% | 35 |

**e6 was not the worst — e3 at 15:06 was**, on both loop overrun and camera
deaths. e6 had the highest reconnect *rate* and was the last match played. The
escalation belongs to May 2 elims generally, not to e6 specifically.

**Ruled out as the reason it started thrashing** (all measured):

| Suspect | Evidence against |
|---|---|
| We published more topics | 373 in q44, 386 at the worst. Essentially flat. |
| Attached for longer | Attached time went *down*, 168 s → 106 s. |
| ~~The radio degraded~~ | **Weak, do not rely on it.** DS and FMS attach exactly twice per match in every one of these and `CommsDisableCount` is 0, but DS control traffic is a few hundred bytes of UDP at 50 Hz and FMS prioritises it. A healthy DS link proves the prioritised low-bandwidth channel worked; it says nothing about bandwidth headroom for a bulk TCP stream. Field-side congestion is **not** ruled out. |

**Suspected, not measured — the mechanism.** NT4 is subscription-based, so
Elastic is cheap: it subscribes to the handful of topics it draws.
AdvantageScope subscribes broadly, and each reconnect re-announces topics and
re-sends values. That is consistent with everything above, but the log records
connection state and IP only — it has no bandwidth, signal strength or
subscription data, so this is inference from how NT4 works and not something
these logs demonstrate.

### Direction of causation

The correlation alone is ambiguous, because a loop stall would break the NT
connection *and* stale the heartbeat — which would make the dashboard a victim
rather than a cause, and unplugging it would fix nothing. The two stories differ
in ordering, and the ordering is measurable.

Loop health in 1-second windows around each event:

| | 2-1 s before | 1-0 s before | 0-1 s after | 1-2 s after | match avg |
|---|---|---|---|---|---|
| **reconnect**, e6 | 4% | 3% | **14%** | 6% | 4% |
| **reconnect**, e3 15:06 | 9% | 4% | **17%** | 9% | 5% |
| **reconnect**, e3 14:34 | 6% | 4% | **18%** | 11% | 2% |
| **disconnect**, e6 | **12%** | 4% | 5% | 6% | 4% |
| **disconnect**, e3 15:06 | **12%** | 7% | 8% | 8% | 5% |
| **disconnect**, e3 14:34 | **13%** | 4% | 7% | 12% | 2% |

(percent of loops over 60 ms)

Before a reconnect the loop is at or below the match average; immediately after
it is 3-9x worse. Before a *disconnect* the loop is already elevated. So the
cycle runs reconnect → stall → disconnect → reconnect, and it is self-sustaining
once started. The dashboard is not merely a bystander: the reconnect is what
costs the time.

62 distinct ephemeral source ports in e6 confirm each cycle is a genuine TCP
teardown and re-establish, not a stream hiccup. It does not say which end closed
it — either end closing, or a timeout, looks the same from here.

### What we still cannot attribute

**Whether the laptop or the field started it is unknown**, and the earlier claim
that it was laptop-side is withdrawn. What is measured: our topic count is flat
(373-386), attached duration went down not up, the reconnect precedes the stall,
and the cycle sustains itself. What is *not* measured: bandwidth, signal
strength, subscriptions, or which end dropped the connection. A laptop-side
trigger (position, machine, tabs open) and a field-side one (elims RF
congestion, the FMS bandwidth cap) both fit everything above.

The practical conclusion does not depend on resolving it. The reconnect cost is
measured, the cycle is self-sustaining, and a stable attachment is harmless — so
keep it off the field radio during matches and the question stays academic.

**The fix is not code.** Do not connect AdvantageScope over the field radio
during a match. Log to the USB stick and pull it afterwards, which is what
`WPILOGWriter` is already doing. Tethered in the pit (the .200 address) it
caused no problems in any log.

Worth noting for later, but not worth changing on its own: a false-dead camera
makes `LimelightIO.updateInputs` return early, so half a second of perfectly
good vision is discarded every time this fires — and section 9 is about what
vision dropouts cost us in autonomous. Raising the heartbeat threshold would
hide this rather than fix it.

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

## 7b. The shot counter was measuring itself

**Measured**, by counting balls off match video and comparing against what the
detector said for the same window.

Every balls-per-second figure in this document comes from one detector: peaks
in flywheel stator current, one peak per ball. It had never been checked
against a real count. It was tuned so the relative numbers tracked scouted
scores, which catches gross errors and says nothing about accuracy.

Five volleys, hand counted:

| volley | detector | actual | detected rate | **true rate** | error |
|---|---|---|---|---|---|
| Ontarios q75 | 12 | 11 | 2.86/s | 2.62/s | −8% |
| Houston q44 | 17 | **19** | 3.86/s | **4.32/s** | +12% |
| Niagara q5 | 16 | 16 | 4.21/s | **4.21/s** | 0% |
| Niagara q24 | 32 | **25** | 5.61/s | **4.39/s** | −22% |
| Niagara q8 | 27 | **21** | 5.87/s | **4.57/s** | −22% |

**The error is not noise — it tracks the detector's own output.** Above about
5 balls/s it invents peaks and overcounts by roughly a fifth; around 4 it is
accurate; below that it slightly undercounts. So a volley the detector thinks
is fast is reported faster still, and the spread between events is
manufactured by the instrument.

Correcting for it, the four full-hopper volleys land at **4.21, 4.32, 4.39 and
4.57 balls/s** — across two events, an 8% spread, against a 52% spread in what
the detector reported. Pooled: **81 balls in 18.5 s, 4.38 balls/s.**

The fifth ran at 2.6 balls/s with a near-empty hopper. That is the one real
source of variation found, and it is ball *supply*, not the shooter.

### What this retracts

**The event-to-event shot rate comparison, and everything resting on it.** A
season sweep gave Niagara 5.46 balls/s against Houston 3.46 and Ontarios 3.59,
which reads as a serious decline. Niagara's volleys are the fast ones, and fast
is exactly where the detector inflates. The gap is an artifact.

Note the per-match table in §7 was never wrong: it puts Houston at 4.27–4.66
b/s, which is what hand counting confirms. The damage was done by the
cross-event aggregate, which additionally divided by fire-command time
including windows where no ball ever came out.

### What survives, because it never depended on counting balls

| | Durham | Niagara | Ontarios | Houston |
|---|---|---|---|---|
| flywheel goal | 54.2 rps | 49.0 rps | 46.2 rps | 45.7 rps |
| inertia proxy (torque ÷ dω/dt) | 0.853 | **0.417** | 0.672 | 0.701 |
| recovery per ball | 136 ms | **54 ms** | 81 ms | 67 ms |
| feeder speed while firing | — | 38.0 rps | 37.5 rps | 40.5 rps |

The flywheel really did get heavier — the inertia proxy comes from torque
divided by angular acceleration during spin-up and involves no ball counting at
all. **But it never bound.** Recovery is 54–67 ms against a ball arriving every
~228 ms, so the wheel spends roughly 160 ms per ball already back at speed and
waiting. The feeder did not slow either; Houston's was the fastest measured.

There was no mechanism for a decline, which is the tell that should have been
followed before the decline was explained.

### For next time

- A rate derived from a detector needs the detector checked against reality
  before the rate is compared across configurations. Ours changes behaviour
  with flywheel mass, which is the exact variable under study.
- Pick validation windows at random. Choosing "countable" windows means
  choosing slow ones, and rate was the measurement — the first two counts were
  biased this way and had to be thrown out.
- Ask what the mechanism would be. Recovery time against ball spacing was
  measurable from the start and rules the story out in one line.

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

### Correction: a debounce on `turret.atGoal()` is the wrong fix

I suggested one. The data says no.

**Measured**, across every atGoal dropout while enabled (ONWEL 1102, Houston
1635; 73 and 246 of them ended a fire window):

| | duration p50 | peak error p50 | peak error p75 |
|---|---|---|---|
| all dropouts | 0.09-0.11 s | 21-23 deg | 44-55 deg |
| the ones that killed a shot | 0.11-0.20 s | 22-35 deg | **336-340 deg** |

The turret is genuinely off target, not flickering. At 3 m, 20 degrees is
**1.09 m** of lateral error — wider than the goal. Debouncing would let those
shots out and they would miss.

And more than a quarter of the shot-killing dropouts have a peak error around
340 degrees, which is not an aiming error at all — that is the turret
**unwrapping**, swinging the long way round through its 380 degree range. Those
last 0.6 s or more.

So the fire windows are being closed for two real reasons, and both need the
turret fixed rather than the gate loosened:

- the turret being outrun while tracking (see the discarded motion profile
  above), and
- unwrap events, which is the `getSmartUnwrapAngle` item already in the backlog.

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
| 2 | ~~Raise/remove the feeder supply cap (25 A)~~ | ~~toward 5.4 balls/s~~ **— see §7b, the 5.4 was a counting artifact** | ~~High~~ **Low** | The cross-event A/B this rested on does not survive hand counting. The cap may still be worth raising, but not for this reason and not at this priority. |
| 3 | Falling debounce on `turret.atGoal()` in `canFire()` | up to ~9.5 s/match of window edges | **Medium-High** — 49% of windows measured | Must not loosen the actual aim requirement |
| 4 | Use or delete the turret motion profile | clarity; possibly better tracking | **Medium** | Changing what is commanded needs field time |
| 5 | Make `RUNSLOW` slower, or delete the branch | removes a no-op | **High** | None |
| 6 | Re-check the feeder velocity goal after 2 | 65 rps is only reachable with the torque to get there | Medium | Needs a real robot |

Items 1 and 2 are the current-limit story. Item 3 is independent of it and is
roughly the same size — worth doing regardless of what happens with power.

### The flywheel is bus-voltage limited, not current limited

**Measured.** This overturns a claim recorded earlier in the backlog and changes
what raising the fire rate means.

Flywheel state while firing, bucketed by how far below goal the wheel was:

| deficit | motor V | **duty cycle** | stator A | bus V |
|---|---|---|---|---|
| at setpoint | 5.8 | 0.59 | 9 A | 10.02 |
| 1-3 rps | 6.7 | 0.75 | 32 A | 9.27 |
| 3-6 rps | 6.9 | 0.83 | 46 A | 8.85 |
| **6-10 rps** | 7.4 | **1.00** | 38 A | **8.25** |
| **10-20 rps** | 7.0 | **1.00** | 34 A | **7.79** |

Once the wheel is more than about 6 rps down, **duty cycle is pinned at 1.00**.
The motor is flat out. It is not being held back by a current limit — measured
stator peaks at 119 A against a 120 A limit in **0.03%** of samples, and during
deep dips the 90th percentile is only 62-74 A.

It cannot pull more current because there is no voltage left: the bus is at
**7.8-8.3 V** during exactly the moments recovery is needed, against 10.0 V at
setpoint, and at 45 rps most of what remains is spent on back-EMF.

**This corrects the backlog.** "During a 20 rps dip the controller asks for
roughly 420 A against a 120 A stator limit — saturated more than threefold" was
arithmetic on the gains, not a measurement. The achieved current never gets near
the limit, so raising current ceilings buys nothing. What buys recovery is bus
volts.

Consequences, in order:

1. **Capping the drivetrain while shooting is the fire-rate fix**, not just the
   brownout fix. The drivetrain is what pulls the bus to 8 V. `PowerManagerState`
   already says this in its `SHOOTING_FAR` comment — "what actually helps is the
   bus voltage itself" — and it is now directly evidenced.
2. **Battery selection matters as much as any code change.** Pack resistance
   varies 11.2-15.0 mOhm across matches, worth 1.14 V; against an 8 V bus that is
   a 14% change in the voltage available for recovery.
3. **Raising flywheel current limits does nothing.** They are never reached.
4. **Feeding faster than the wheel can service does nothing.** See the ceiling
   below.

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

## 12. Corrections to things stated earlier in this repo

Recorded so they are not believed twice.

| Claim | Correction |
|---|---|
| ~~"Shot rate dropped 37%"~~ | Twice wrong. First contaminated by non-FMS practice logs (FMS only: 19%). Then **retracted entirely** — hand counting five volleys off match video shows the rate never dropped. See §7b. |
| "`logReview shots` gives balls per second" | It counts rising edges of `Commands/Fire shot` — how often the shooter is *let go*, not how often a ball leaves. Renamed and split by state in `LogReview` (commit `c53b303`). |
| "The Ontario logs are champs matches" | All ten have `FMSAttached = false`. They are practice sessions dated after the event. |
| "A feeder dragged to 65% of goal must be passing more material" | Sloppy. The goal went 40 → 65 rps *and* the gains changed, so more droop is expected at the same ball rate. |
| ~~"AdvantageScope disconnects caused the camera dropouts"~~ | **Retracted. It did.** The first test correlated against *disconnects*; the event that costs time is the *reconnect*, which re-announces every topic. Re-tested against reconnects: 96-100% of camera deaths land within 1 s, against a null model of 0-18. See section 2.4. |
| "Estimate types are 50/50 NONE/MEGATAG" | Counted transitions, not time. Time-weighted it is 96–99% NONE for the frame camera. |
| "GC pauses cause the loop stalls" | 0 collections during the match examined; 1 of 206 overruns was GC-related. |

---

## 9. Why autonomous fails at the bump — four matches, one cause

**Measured**, on the four the drive team reported: q44, q59, q76 and e6
(`2026mil_sf6m1`). Their account was right, and it is a *pose* failure that
happens at the bump rather than a bump failure.

**The mechanism.** The bump costs about a third of wheel travel (section 9c), so
odometry over-reports how far the robot has gone. The path follower acts on that
wrong pose. It then does one of two things depending on what the wrong pose
implies:

### It turns early and drives into the hub — q59, q76

| q59 | pose | hub gap | speed | drive current |
|---|---|---|---|---|
| t=7.52 | (6.89, 4.67) | 1.69 m | 0.72 | **345 A** |
| t=7.81 | (6.76, 4.72) | 1.56 m | 0.42 | **348 A** |
| **t=8.11** | **(5.80, 3.71)** | **0.59 m** | 0.09 | 166 A |

A **1.39 m pose jump in 0.3 s**. Before it the robot is pushing 345-348 A at
0.4 m/s — shoving against something immovable — while odometry places it 1.6 m
clear of the hub. Vision then corrects it to 0.59 m away, which for a robot with
a 0.5 m half-diagonal is contact.

q76 is the same shape: 257 A at 0.08 m/s at (6.47, 4.13), then a **0.87 m jump**
to (5.81, 3.57), hub gap 0.61 m.

The robot drove into the hub and odometry did not know. "It didn't cross far
enough into the middle of the field" is exactly right — the turn was commanded
against a pose that thought it had.

### It gives up crossing and tries to spin on the ramp — q44, e6

Tilted well above the flat-ground baseline, barely moving, commanded to rotate
hard:

| match | t | match clock | pose | tilt | speed | commanded omega |
|---|---|---|---|---|---|---|
| **q44** | 8.9 | **12** | (13.56, 5.40) | 22.2° | **0.05** | **+5.79** |
| q44 | 9.2 | 12 | (13.11, 5.60) | 14.7° | 0.43 | **+11.55** |
| q59 | 10.7 | 10 | (3.02, 2.66) | 11.0° | **0.04** | -6.70 |
| q76 | 10.5 | 10 | (2.96, 2.66) | 13.4° | **0.09** | -4.83 |
| e6 | 9.2-10.9 | 11-12 | (13.54, 2.67) | 11-18° | **0.02-0.09** | -6.77 to +2.05 |

q44 commands **11.55 rad/s — 660 degrees per second** — while sitting still at
22 degrees of tilt. The follower believes it has arrived at the waypoint and
only heading is left, so it stops asking for translation and asks for rotation
instead. It has not arrived; it is short, on the ramp.

### What we could see, and threw away

**Measured**, decoding the megatag results that were rejected.

q59, the 1.3 seconds before the robot drove into the hub. The turret camera had
tag 19, then tag 20, continuously — about 45 frames:

| | measured | the gate behind the flag | verdict |
|---|---|---|---|
| distance | 0.98 - 1.12 m | — | close |
| `avgTagArea` | **1.5 - 2.3** | `MIN_TAG_AREA_SINGLE_TAG` = 1.0 | **passes** |
| ambiguity | **0.02 - 0.09** | `AMBIGUITY_THRESHOLD` = 0.7 | **passes easily** |
| tag count | 1 | `IGNORE_SINGLE_TAG` | **blocked** |

**Every one of those frames would have passed the existing single-tag quality
guards.** They are unreachable code, because the flag short-circuits first.

What those frames were worth, against the next accepted fix at (5.80, 3.70):

| t | estimate | error vs odometry | **error of the estimate** |
|---|---|---|---|
| 7.0 | (5.91, 3.41) | 1.62 m | 0.31 m |
| **7.5** | **(5.80, 3.67)** | **1.47 m** | **0.03 m** |

A single-tag fix **3 centimetres** from truth while odometry was **1.47 m**
wrong, six tenths of a second before the collision.

q59's second window is the same story at smaller scale — single-tag estimates
0.25 to 0.29 m from truth while odometry was about 1.0 m off.

**A caveat on the area gate.** Across whole matches, `MIN_TAG_AREA_SINGLE_TAG`
= 1.0 would pass only 1.5% of the frame camera's single-tag frames and 17% of
the turret's, because the median area is 0.33 and 0.67. That sounds fatal until
you notice which frames pass: the close ones. In the window where the robot was
actually lost, areas were 1.5 - 2.3. The threshold self-selects for short range,
which is where a single tag is trustworthy and where being lost hurts most.

**~~e6 is a different failure and should not be lumped in.~~ Retracted — e6 is
the same failure.** The claim above was that the frame camera's estimate sat
byte-identical for five seconds, so the Limelight was frozen rather than being
filtered. That was an artifact of how WPILOG stores data: records are written
only when a value *changes*, so a camera that sees nothing and a camera that is
frozen both leave the same silence in the log. Counting identical payloads
counts nothing, because identical payloads are never written twice.

Separating the two properly — stale means `isAlive` and `canSeeTag` both true
with no new `MegatagResult` — e6's longest genuine stale spell is **0.64 s**,
not five seconds. e6 does have far more staleness than its neighbours (39
spells, 16.9 s total on the turret, against 1-2 spells under 0.6 s in q44, q59
and q76), but it is spread across the match in half-second slices that line up
with loop overruns, and none of it lands in the failure window.

Both limelights are also declared dead on the *same loop* twice during e6 auto
(t=2.8 and t=11.6). Two cameras failing simultaneously is not two cameras
failing — `isAlive` is derived from a heartbeat that robot code has to read, so
any loop stall past `LL_HEARTBEAT_MIN_FREQ` (0.5 s) marks every camera dead at
once. e6 ran 26.2% of its loops over 30 ms, against 14.2% in q44. That is a
roboRIO symptom being reported as a camera symptom, and it is worth fixing
separately, but it is not what lost the auto.

### Why vision does not rescue it

In e6 the robot sat motionless for **5.2 seconds** with the pose going stale to
**6.3 seconds** old, then corrected 0.55 m. Throughout that time the frame
camera had **exactly one tag in view continuously** and every frame was
discarded by `IGNORE_SINGLE_TAG`.

That is the case for accepting single-tag estimates at reduced confidence. It is
not that the tags were missing.

**Confirmed by replay.** Running the e6 log through the pose estimator both ways
(see the `frc-log-replay` skill):

| blind window | length | baseline error | single-tag error |
|---|---|---|---|
| t=1.5 s | 1.6 s | 1.42 m | 1.06 m |
| **t=7.2 s** | **6.3 s** | **0.92 m** | **0.08 m** |

The second row is the one that lost the auto. Through it the baseline holds
(13.54, 2.66) while the gyro reads 17° of tilt — a robot at 17° is on a ramp,
and the far bump ends at x=12.459, so the pose is at least 1.08 m past the
nearest surface that could tilt it. Single-tag holds x≈12.26, which is *on* the
ramp and physically coherent with the tilt.

It takes almost nothing to do it: 346 accepted fixes against the baseline's 338.
Eight extra corrections, landing in the one window where there were none.

### What this costs, per match

| | genuine stall | worst pose staleness | correction when vision returned |
|---|---|---|---|
| q59 | 1.9 s | 3.1 s | 0.69 m |
| e6 | 4.0 s | **6.3 s** | 0.55 m |
| q76 | — | — | 0.91 m (at the end of auto) |

---

## 9z. Earlier framing of this section, corrected

**Measured**, on e6 (`2026mil_sf6m1`), the match the drive team reported.

The robot sits motionless for **5.2 seconds**, commanded to rotate but not
translate, then the pose snaps **0.71 m backwards** the instant vision returns:

| t | x | onBump | tilt | speed | cmd vx | cmd omega | vision age |
|---|---|---|---|---|---|---|---|
| 8.53 | 13.44 | YES | 9.6 | 0.16 | -0.04 | 0.51 | 1.34 s |
| 10.70 | 13.54 | YES | 17.6 | **0.06** | 0.04 | 2.20 | 3.51 s |
| 12.36 | 13.54 | YES | 11.3 | **0.05** | -0.07 | -0.44 | 5.17 s |
| 13.28 | 13.54 | YES | 22.7 | 0.07 | 0.06 | -1.60 | **6.08 s** |
| 13.70 | **12.83** | YES | 16.2 | 0.35 | -0.89 | -0.80 | 0.19 s |

The chain:

1. The bump costs about a third of wheel travel (section 9c), so odometry
   **over-reports** how far the robot has come.
2. The path follower therefore believes it has **arrived**, and stops commanding
   translation — `cmd vx` is ~0 for the whole stall while `cmd omega` swings.
   It only rotates.
3. It is really 0.7 m short, still against the ramp.
4. Vision does not correct it, for 6 seconds.

**Step 4 is the fixable one, and it is not that the tags were missing.**

| camera | tags in view during the stall | Z solve | why it was rejected |
|---|---|---|---|
| **frame** | **1, continuously** | **0.148 m, stable** | **`IGNORE_SINGLE_TAG`** |
| turret | 1-2 | 0.57 - 2.32 m | `MAX_Z_ERROR`, correctly — those solves are garbage |

The frame camera had a good, geometrically consistent single-tag fix for the
entire stall and every frame was discarded. This is the concrete case for
accepting single-tag estimates at a reduced confidence rather than not at all.

**A second problem sits behind it.** The frame camera's Z baseline is 0.148 m and
`MAX_Z_ERROR` is 0.20, leaving 0.05 m of margin. The ramp is 0.165 m tall, so a
robot genuinely on the bump reads about 0.31 m and is rejected — visible at
t = 13.57 and 13.99 where frame Z reaches 0.242 and is dropped exactly as the
robot reaches the ramp. The check that is meant to reject bad solves also
rejects good ones taken on the one piece of geometry that matters.

### Correction: `onBump` is not mainly firing when it should not

An earlier version of this section claimed 16% of autonomous was spent at half
acceleration because `onBump` was falsely set. **That was wrong**, and wrong by a
circularity worth recording: it classified "falsely on the bump" by asking
whether the *logged pose* was inside the ramp bands, when the pose is odometry
and odometry is precisely what fails there. A robot over-reporting its travel
reads as past the ramp while still on it, and got scored as a false positive when
`onBump` was right and the pose was wrong.

Re-run using only moments where an accepted vision pose landed within 0.3 s, so
the pose is anchored to something absolute:

| | agree | false positive | false negative |
|---|---|---|---|
| autonomous | 90% | **3%** | **6%** |
| whole match | 94% | 2% | 3% |

False negatives — on the ramp, `onBump` says no — are **twice as common** as
false positives. The drive team's read was right and mine was not.

Note what this cannot see: **40% of autonomous has no fresh vision anchor at
all**, and that is exactly where the failures live. The honest position is that
`onBump` is roughly right when the pose is trustworthy, and that nobody, robot
or analyst, knows where the robot is the rest of the time.

The gyro problems below are still real — they were measured from the tilt signal
directly — but they are a smaller effect than claimed, and they are not the
reason autonomous fails at the bump.

---

## 9a. The gyro reads 7 degrees flat, and cannot tell braking from a ramp

**Measured.** A contributing problem rather than the main one — see the
correction above.

`GyroIOPigeon2.getTiltMagnitude()`:

```java
double gz = pigeon.getGravityVectorZ().getValueAsDouble();
double tiltRad = Math.acos(MathUtil.clamp(gz, -1.0, 1.0));
```

Two problems, both measured across every match.

**It reads 7 degrees while stationary and level.** Across 12,708 samples with the
robot still, flat, and nowhere near a ramp, the median tip angle is **7.0
degrees** and the 90th percentile is 7.4. `ON_BUMP_TILT` is **9.5**, so three
quarters of the budget is gone before the robot moves. `acos` is
ill-conditioned near flat — its slope goes to infinity as gz approaches 1 — so a
0.75% error in the gravity vector becomes 7 degrees of apparent tilt.

**And the gravity vector is an accelerometer, which cannot tell gravity from
braking.** Sampled away from any real ramp:

| acceleration | median tilt | p90 tilt | over the 9.5 degree threshold |
|---|---|---|---|
| 0.0 - 0.5 m/s² | 7.0° | 7.5° | 2.2% |
| 1.5 - 3.0 m/s² | 7.1° | 9.1° | 9.4% |
| 5.0 - 8.0 m/s² | 7.1° | 10.5° | 11.6% |
| 8.0+ m/s² | 7.1° | 13.5° | **17.0%** |

### What it costs

`StateOrchestrator.determineOnBump` is `(X in band) OR (not level)`, so a false
tilt reading sets `onBump` anywhere on the field. That does three things:

- `DriveToPose` drops the autopilot acceleration limit from
  `AUTO_MAX_ACCEL` 15 to `AUTO_MAX_ACCEL_BUMP` 7 — **halved** — in autonomous
- the shooter is forced from `TARGET_HUB` to `TARGET_TAG`
- `ShotPlanner` stops leading the shot

Measured over the four reported autos:

| match | auto | onBump | of which false | share of auto |
|---|---|---|---|---|
| q44 | 20.2 s | 4.7 s | 3.6 s | **18%** |
| q59 | 20.6 s | 2.9 s | 2.0 s | 10% |
| q76 | 20.0 s | 2.6 s | 1.7 s | 9% |
| **e6** (`sf6m1`) | 21.1 s | 6.9 s | **5.7 s** | **27%** |

**16% of autonomous, across those four matches, is spent driving at half
acceleration because the robot believes it is on a ramp it is nowhere near.**
That is "undershot our setpoints" and "took us forever to start shooting".

### A debounce does not fix it — measured

The obvious guard fails. Episode durations are nearly identical:

| | n | p50 |
|---|---|---|
| genuinely on a bump | 390 | 0.392 s |
| nowhere near a bump | 409 | 0.369 s |

A 0.20 s debounce keeps 93% of real crossings and still lets **81%** of the
false ones through. Duration carries no information here, so no debounce
separates them.

### What would

- **Use the Pigeon's fused pitch and roll** rather than the raw gravity vector.
  They are gyro-integrated and accelerometer-corrected slowly, so linear
  acceleration should not move them, and `hypot(pitch, roll)` avoids the `acos`
  conditioning problem entirely. **They were not logged**, so this cannot be
  evaluated against the season's data — they are logged now, which is the only
  change made here.
- **Calibrate the 7 degree offset out**, whatever its source.
- Consider requiring the tilt path to agree with position being *near* a ramp.
  That weakens the case it exists for — catching a bad pose — so it is a
  judgement call rather than an obvious win.

Nothing about `onBump` was changed. The evidence is strong but the fix depends on
sensor behaviour that no log can confirm.

---

## 9c. Measured wheel slip

**Measured** across 25 FMS matches, comparing wheel travel against displacement
between accepted vision poses — the only independent ground truth available,
since `FieldPose` is itself vision-corrected and would show zero slip against the
wheels by construction.

| surface | n | p25 | median | p75 |
|---|---|---|---|---|
| carpet | 1228 | -2.2% | **6.0%** | 20.4% |
| bump | 191 | 13.4% | **32.4%** | 47.9% |

**The bump costs about a third of wheel travel.** Carpet's 6% sits inside the
measurement noise — the 10th percentile is *negative*, meaning vision said the
robot moved further than the wheels turned — so treat carpet slip as small and
not resolvable by this method rather than as 6%.

A first version of this measurement paired only *consecutive* accepted vision
poses and then demanded a 0.35 s gap between them. Since accepted poses arrive
about every 30 ms, that discarded everything except pairs straddling a vision
dropout, sampling exactly the periods when vision was failing. The medians barely
moved once fixed, but the sample went from 174 to 1228.

---

## 9d. Simulation limitations worth knowing

Recorded because a simulation you trust past its limits is worse than none.

- **The robot is a circle**, radius 0.505 m (its half-diagonal). Right for
  clipping a corner, too fat for threading a slot: the real 0.762 m robot fits
  the tower's 0.858 m climbing gap and the circle does not, by about 9 cm.
  A rectangle would fix it.
- **The drivetrain still spikes for a loop or two** at the instant full stick is
  applied. Steady state is now sane (25 A per module at a 45 A limit, bus holding
  9.06 V), but assertions should use current rather than voltage until this is
  gone, because voltage saturates against the pack's 4 V floor and silently
  compares nothing.
- **Simulated cameras see fewer tags than real ones.** Sampled across 640
  placements and headings, the simulated frame camera sees no tag 71% of the
  time against 34-62% on the real robot. Single-tag (16%) and multi-tag (13%)
  views both occur, so `IGNORE_SINGLE_TAG` *is* evaluable in simulation —
  an earlier note in this session claiming otherwise was wrong — but vision
  availability is pessimistic, so a sim result is a lower bound.
- **maple-sim's arena cannot cross the bump.** Recorded here because it is the
  reason not to adopt it: `Arena2026Rebuilt` models the ramps as a solid
  impassable obstacle, or omits them. Its tower and trench geometry is better
  than what this repo had, and has been adopted; its drivetrain and field
  handling would cost the bump, the fitted battery and flywheel models, and the
  IO layer that makes replay work.

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

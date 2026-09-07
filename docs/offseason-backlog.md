# Offseason backlog

Candidate improvements, where each came from, and what evidence supports it.

Most entries came from reading two public codebases against ours:

- [frc1678/C2026-Public](https://github.com/frc1678/C2026-Public) — same game, no AdvantageKit, strong shooting and vision filtering
- [team581/frc-2026](https://github.com/team581/frc-2026) — multi-robot monorepo, state-machine architecture, unusually good docs

**Do not treat "they do X" as a reason on its own.** Several of their ideas were
measured against our logs and rejected, and two of their most interesting files
turned out to be dead code in their own repo. Each item below records why it is
here.

| Status | Meaning |
|---|---|
| **Open** | Worth doing, not started |
| **Investigate** | Read-only work that decides whether a change is worth making |
| **Done** | Landed on `offseason` |
| **Rejected** | Considered and deliberately not done — the reason is recorded so it is not re-derived |

---

## CRITICAL — power

Not a backlog item. Measured on the 2026 ONWEL event logs across the 10 matches
in them:

```
./gradlew logReview --args="power    <dir>"     per-match voltage, brownouts, peak draw
./gradlew logReview --args="channels <dir>"     what the current is actually going to
```

| Measure | Value |
|---|---|
| Minimum battery voltage | **6.28 – 7.03 V** (our configured brownout threshold is 6.5 V) |
| Brownout samples | **63 across 10 matches** |
| Share of brownouts in the final 50 s | **65 %** |
| Average voltage while enabled | ~10 V, declining only ~0.7 V across a match |
| Peak current draw | **242 – 472 A** (see the q5 caveat below) |

**The end-of-match symptom is real, and the cause is peak current, not
capacity.** Average voltage falls only about 0.7 V from start to end, so the
battery is not running empty. What happens is that spikes sag the pack to
6.3–7.0 V, and as state of charge drops the pack's internal resistance rises, so
the same spike sags deeper. That is why brownouts cluster late. It also matches
the physics: at ~15 mΩ internal resistance a 400 A spike is a ~6 V sag, which is
the sag we measure.

Peak draw tracks brownouts:

| Match | Peak | Brownout samples |
|---|---|---|
| q8 | 250 A | 0 |
| q15 | 246 A | 1 |
| q20 | 318 A | 1 |
| q12 | 288 A | 5 |
| p3 | 242 A | 6 |
| q24 | 420 A | 3 |
| p4 | 472 A | 9 |
| q31 | 378 A | 19 |
| 14-14-26 | 414 A | 35 |

### Where the current goes

Every Talon logs its own supply current in its `TalonFXIOData` struct, so this
does not depend on knowing PDH wiring. Means are over all enabled samples
(including while the mechanism is idle), so read the peak column for headroom
and the "spike" column for what a mechanism contributes when draw is high:

| Motor | Mean | While draw is high | Peak | Configured supply limit |
|---|---|---|---|---|
| Drive M0–M3 *(estimated)* | 11 – 15 A | 23 – 34 A | 117 – 206 A | 45 A |
| Flywheel 0 | 11.5 A | 20.7 A | **90.1 A** | 60 A |
| Flywheel 1 | 11.3 A | 16.9 A | **95.5 A** | 60 A |
| Intake 0 | 6.9 A | 17.5 A | **84.2 A** | **none** |
| Intake 1 | 5.5 A | 13.5 A | **75.4 A** | **none** |
| Feeder 0 | 13.8 A | 11.4 A | 84.5 A | 25 A |
| Feeder 1 | 14.0 A | 10.8 A | 86.0 A | 25 A |
| Indexer 0 | 9.5 A | 8.2 A | 64.1 A | none |
| Indexer 1 | 6.5 A | 5.3 A | 53.3 A | none |
| Turn M0–M3 *(estimated)* | 1.7 A | ~2 A | 85 – 105 A | none |
| Expander | 1.6 A | 1.0 A | 70.7 A | none |
| Hood | 0.2 A | 0.4 A | 23.9 A | none |
| Turret | **not logged** | | | none |
| Climber | **never writes a value while enabled** — the mechanism does not exist | | | |

Roughly: **drive is about half of a spike, the mechanisms the other half**, with
flywheel and intake the two biggest mechanism contributors.

**Two logging gaps.** `ModuleIOTalonFX` logs stator current only, so the drive
numbers above are derived as `stator × appliedVolts ÷ busVoltage` and agree with
the PDH channels to within about 25%. `TurretIOTalonFX` logs no motor data at
all — the turret's draw is simply unknown.

**Supply limits are exceeded where set, and missing where not.** Stator limits
are set nearly everywhere (120 A), but a stator limit does not bound what the
battery supplies: a motor stalled at 20% duty pulls 120 A stator and only ~24 A
from the pack. Supply current is what sags the battery, and it is the limit we
mostly do not set. Hood, turret and the indexer
rollers have none. Intake's is commented out **deliberately** — see below.

### Do not put the intake supply limit back

`IntakeIOTalonFX` has `.withSupplyCurrentLimit(35)` commented out. It was removed
because the intake bogged down while intaking, especially in autos. The logs say
that was the right call:

| Commanded duty | Samples | Mean supply | Mean stator | Mean speed |
|---|---|---|---|---|
| 0.9 – 1.0 (free) | 29 977 | **14.0 A** | 15.3 A | 24.7 rps |
| 0.6 – 0.9 (loaded) | 676 | **54 – 64 A** | 74 – 87 A | 9 – 15 rps |

The intake is run open-loop at full duty cycle, so supply current is close to
stator current, and its draw peaks precisely when it is loaded. A 35 A supply
limit would have clamped it in exactly the regime where torque is needed, and
would barely have touched the free-spinning case that costs 14 A. Restoring it
would recreate the bogging. If a limit is wanted, it belongs somewhere near
70 A, and only alongside a state-based budget that raises it while intaking.

`./gradlew logReview --args="motor Intake/IntakeMotor <logs>"` reproduces this.

### The intake does recover — measured

The related theory was that the intake, once dragged down by a pile of balls,
recovers more slowly than the drivetrain moves, so the robot outruns it and
beaches. The ONWEL logs do not support it:

```
./gradlew logReview --args="bog IntakeMotor0 <logs>"
```

| | Intake 0 | Intake 1 |
|---|---|---|
| Free speed | 29.5 rps | 29.4 rps |
| Time below half free speed while commanded | **2.5 %** | 2.0 % |
| Bog episodes | 263 | 216 |
| Mean episode | **0.05 s** | 0.05 s |
| Longest episode | 0.52 s | 0.52 s |
| Robot speed while bogged | **1.34 m/s** | 1.40 m/s |
| Robot speed while intaking overall | 1.74 m/s | 1.74 m/s |

Bogs are real but brief — a typical one is one loop long, the worst half a
second. And the robot is moving *slower* while bogged, not faster, so the
drivetrain is not outrunning the intake. Whatever causes beaching, this is not
it. Caveat: these logs predate the current code.

### Why low voltage makes the shooter undershoot

The flywheel runs `VelocityTorqueCurrentFOC`, and a motor's achievable speed
scales with bus voltage. As the battery sags, a goal that was reachable stops
being reachable no matter what the controller asks for. Measured across the
ONWEL logs with `./gradlew logReview --args="shooter <dir>"`:

| Battery | Samples | Goal | **Shortfall** | Reported "at setpoint" |
|---|---|---|---|---|
| 12.0 – 12.5 V | 2855 | 48.8 rps | 0.14 rps | 98.4 % |
| 11.0 – 11.5 V | 3897 | 51.2 rps | 0.47 rps | 97.8 % |
| 10.0 – 10.5 V | 3845 | 52.7 rps | 1.62 rps | 95.3 % |
| 9.0 – 9.5 V | 4133 | 53.3 rps | 3.99 rps | 94.9 % |
| 8.0 – 8.5 V | 3161 | 55.3 rps | 6.67 rps | 91.2 % |
| **below 8.0 V** | 680 | 57.2 rps | **11.60 rps** | **71.0 %** |

Perfectly monotonic. Below 8 V the wheel is **20 % below its goal**.

**And the readiness gate does not catch it.** `Flywheel.atSetpoint()` compares
against `FlywheelConstants.RPM_RANGE`, which is **1200 RPM = 20 rps** — on a
~53 rps goal that is a **±38 % tolerance**. The wheel can be a fifth slow and
still report ready, which is exactly what the table above shows.

**For hub shots the flywheel check is bypassed entirely.** `RobotState.canFire()`
reads:

```java
.and(() -> shooterState == ShooterState.TARGET_HUB ? true : (RobotContainer.flywheel.atSetpoint()))
```

Whether that is deliberate for a close dump shot is a question for whoever wrote
it, but it means hub shots fire at any flywheel speed.

Note the goal column rises as voltage falls — the longest shots need the most
flywheel, and spinning the flywheel is itself one of the largest current draws.
The shooter is partly causing the sag that then starves it.

### Passing: fixed since the logs were taken

Worth recording so it is not re-derived. In the ONWEL logs every `TARGET_PASS`
shot commanded ~100.5 rps and delivered ~85, short by 15 rps at any battery
voltage — `FLYWHEEL_REDUCTION` is 1, so the flywheel turns at rotor speed and a
Kraken's 6000 RPM free speed is a hard 100 rps ceiling.

**Current code does not do this.** `ShotMapTest` evaluates the live map and it
clamps flat at 61.0 rps beyond 12.5 m. The fix landed after ONWEL. The test
stays as a regression guard: it fails if the map ever commands above the
flywheel's free speed again.

### How much a slow flywheel actually costs, validated against video

Near 3.5 m the hood map is flat (6.13 → 5.74 → 6.01 rotations), so the flywheel
map's local slope there *is* the true range sensitivity. `ShotMapTest` prints it:
**5.6 rps per metre at 3.5 m**.

Checked against event video for two shots in q31 (both at ~3.5 m, `shots` mode
gives the flywheel deficit and the match clock):

| Field clock | Deficit | Predicted short | Observed on video |
|---|---|---|---|
| 1:34 | 4.3 rps | 0.77 m | **0.6 – 0.7 m** |
| 1:26 | ~22 rps | 3.9 m | **3 – 4 m** |

Both within measurement error. The model holds.

**So `RPM_RANGE = 1200` (20 rps) permits a 3.6 m range error at 3.5 m.** For
0.3 m of accuracy the tolerance needs to be about **1.7 rps — roughly 100 RPM**,
a tenfold tightening.

**The method only works where the hood is flat.** `ShotMapTest` also prints the
slope across the whole map, and it collapses to 0.4 – 0.5 rps/m at 4.5 – 5.0 m
and goes *negative* past 7.5 m. Those are regions where the hood is doing the
work (11.35 → 17.46 rotations across 4.5 – 5.0 m), so the flywheel map's slope is
a calibration path rather than a sensitivity, and inverting it there is not
valid. Getting a tolerance for those distances needs either real projectile
physics or more video points.

### Where we shoot from, and where accuracy falls off

159 hub shots across the ONWEL logs, from `./gradlew logReview --args="shots <dir>"`.
"Within 4 rps" is the share of shots whose flywheel was close enough to land
inside about half a metre:

| Distance | Shots | Median deficit | Within 4 rps | Median battery |
|---|---|---|---|---|
| 1.0 – 1.5 m | 4 | 0.2 rps | 100 % | 12.16 V |
| 1.5 – 2.0 m | 28 | 1.7 rps | 79 % | 11.99 V |
| 2.0 – 2.5 m | 47 | 0.9 rps | 87 % | 11.73 V |
| 2.5 – 3.0 m | 23 | 1.4 rps | 78 % | 11.62 V |
| 3.0 – 3.5 m | 18 | 0.4 rps | 83 % | 11.07 V |
| **3.5 – 4.0 m** | 18 | 3.4 rps | **56 %** | 10.59 V |
| **4.0 – 4.5 m** | 11 | 9.8 rps | **18 %** | 7.52 V |
| **4.5 – 5.0 m** | 8 | 17.5 rps | **13 %** | 10.19 V |

Range is 1.30 m to 9.21 m, but 73 % of hub shots are inside 3.5 m.

**Accuracy falls off a cliff at exactly 3.5 m**, which matches what the drive team
reports independently. And the median battery voltage falls with distance, from
12.0 V close in to 7.5 V at 4 – 4.5 m. Longer shots need more flywheel, more
flywheel means more current, and the resulting sag is what starves the shot. The
shooter creates the conditions that ruin its own long shots.

**This is the metric to move.** If a power budget works, the cliff moves outward.

### Recovery is not a tuning problem

`kP` is 11 amps per rps, and `RECOVERY_FF_PER_RPS` adds another 10. During a
20 rps dip the controller asks for roughly 420 A against a 120 A stator limit —
saturated more than threefold. The gains are not doing anything during recovery;
the motor is simply flat out.

Two things follow. Raising the current ceiling **cannot** upset the tune, because
the tune only operates near setpoint where the demand is ~20 A. And there is
nothing to tune for recovery — the only lever is the ceiling. That removes the
need to simulate before changing it.

### The power manager, as built

Landed on `offseason`. `PowerManager` is a `VirtualSubsystem` holding a
`PowerManagerState` per thing the robot is doing, applied off the main loop
because a Phoenix config write blocks.

| State | Drive | Flywheel stator/supply | Intake | Feeder | Spindexer |
|---|---|---|---|---|---|
| `DEFAULT` | 45 | 120 / 60 | 90 | 25 | 90 |
| `SHOOTING` | 20 | 160 / 140 | 20 | 25 | 90 |
| `SHOOTING_FAR` | 10 | 160 / 140 | 20 | 25 | 90 |
| `SHOOTING_AND_INTAKING` | 15 | 160 / 140 | 50 | 25 | 90 |

All per motor, supply unless stated. `DEFAULT` reproduces what each subsystem
configures for itself, so it is a no-op.

**What is never cut.** The feeder, because a ball entering at an inconsistent
speed makes the shot inconsistent however well the flywheel is holding — and its
25 A limit was added deliberately after ONWEL to reduce sag with no observed
quality loss, so it is not raised either. The spindexer, because it governs shot
rate and has never had a limit. The intake below what it draws while intaking,
because a 35 A limit there used to make it bog.

**When it applies.** Three triggers were measured:

| Trigger | Median warning | Under 0.3 s |
|---|---|---|
| `RobotState/Shooting` | 0.00 s | 100 % |
| flywheel nearly at speed | 1.07 s | 19 % |
| flywheel goal set | 7.23 s | 1 % |

The two early ones would leave the drivetrain weak while driving around the
shooting zone spun up, which is when a defended robot most needs it. Firing wins
despite giving no warning, because **the cap is not preventing the dip** — that is
the ball taking energy out, and nothing electrical stops it — **it is holding the
bus up for the recovery afterwards**, and the wheel does not start dropping until
a median of 0.2 s after the command. The state is held 1 s after firing so a
burst does not thrash the bus.

A hard shove on the sticks, or `setTurboOverride`, hands the drivetrain back
immediately. The override has no control bound to it yet.

**Why a current limit is not the whole answer.** The flywheel's acceleration was
identified from the logs at **2.311 rps/s per amp of stator**, from 3918
accelerating samples. Re-running every measured dip through it:

| Dip depth | Dips | Measured | Model at today's ceiling | Model at 160 A |
|---|---|---|---|---|
| 3 – 8 rps | 1004 | 0.06 s | 0.03 s | 0.02 s |
| 8 – 13 rps | 1069 | 0.07 s | 0.06 s | 0.03 s |
| 13 – 18 rps | 67 | **0.24 s** | 0.09 s | 0.04 s |
| 18 – 23 rps | 17 | **0.59 s** | 0.12 s | 0.06 s |

The model matches measurement for dips up to 13 rps — 93 % of them — so raising
the ceiling roughly halves recovery there. It under-predicts deep dips by five
times, because those were **voltage limited, not current limited**: the wheel was
not getting the current it was already allowed. Raising a limit cannot help a
motor with no voltage to push through it. That is what the drivetrain cap is for,
and it is the more important half of the change.

### The readiness gate, as built

Landed on `offseason`. `Flywheel.atSetpoint()` no longer compares against a fixed
window. The tolerance is derived from an acceptable range error, because range
error is `2 * distance * speedError / speed` — the same speed error costs far
less range up close than it does far out, so one number is wrong at both ends.

| Distance | Goal | Tolerance | Range error |
|---|---|---|---|
| 1.5 m | 41.0 rps | 4.8 rps | 0.35 m |
| 2.5 m | 45.9 rps | 3.2 rps | 0.35 m |
| 3.5 m | 53.3 rps | 2.7 rps | 0.35 m |
| 4.5 m | 57.1 rps | 2.2 rps | 0.35 m |

`ACCEPTABLE_RANGE_ERROR` is 0.35 m and is the number to tune, because it means
something physical. The goal is about a metre across, so the whole budget is
0.5 m either side; turret aim, pose, hood and the shoot-on-move correction all
spend from the same metre, so the flywheel takes roughly two thirds and leaves
the rest. The old `RPM_RANGE` of 1200 permitted **2.6 m** of range error at
3.5 m.

**Opens on 0.25 s inside tolerance, closes only after 0.25 s outside it.** The
falling debounce is what makes it usable: a ball drags the wheel down a median of
7.8 rps on its way out, *after* it has gone, and without the debounce that shuts
the gate behind every shot. Replaying the logs, no falling debounce leaves the
gate open 9 % of the time a shot is wanted; 0.20 s takes it to 64 %, and past
0.30 s it stops improving and starts tolerating real sag.

**Hub only.** `RobotState.canFire()` used to read
`shooterState == TARGET_HUB ? true : atSetpoint()` — hub shots skipped the
flywheel check entirely, which is why the logs are full of hub shots fired 20 rps
slow. That is now reversed: hub gets the strict check, passing keeps the old
loose window through `atLooseSetpoint()`, because passing aims at a region of
floor rather than a goal.

### What a brownout actually looks like

13 excursions below 6.5 V while enabled, 46 % of them in the last 30 seconds.
Averaged over those events, at the worst point of each:

| Source | Draw | Share |
|---|---|---|
| Drivetrain, drive + steer | 140 A | **62 %** |
| Flywheel | 27 A | 13 % |
| Feeder | 22 A | 10 % |
| Intake | 20 A | 9 % |
| Spindexer | 12 A | 6 % |
| **Total accounted** | **224 A** | |

The whole robot is going at once and the drivetrain is most of it. Capping the
drivetrain at 10 A would have prevented 10 of the 13.

**But the power manager as built will not prevent them**, because it only caps
the drivetrain while shooting, and these happen while driving. That is a
deliberate choice, not an oversight — a voltage-triggered guard was considered
and rejected, because capping the drivetrain when the driver is asking for it is
the opposite of what a pinned robot needs. `TURBO` exists for that case.

### Two claims made during this analysis that turned out to be wrong

Recorded so they are not re-derived.

**"The flywheel cannot reach far-shot goals."** Wrong. The wheel was observed at
**87.8 rps at 10.8 V**, and the old passing bug commanding 100.5 rps gives
genuine saturation points: 69 rps at 7.75 V, 83 at 8.75 V, 88 at 10.75 V.
Far-shot goals of 57 – 65 rps are reachable at every voltage in the logs. The
original claim came from reading shots that were sitting *at* setpoint, which
says nothing about the ceiling. The real problem is headroom — a 57 rps goal
against an 83 rps ceiling leaves little torque for recovery, and the ceiling
falls with the bus.

**"There is excess resistance in the robot's wiring."** No evidence. Fitting
voltage against current gives 11 – 16 mΩ total, which is exactly what a healthy
battery (10 – 13 mΩ) plus cable, main breaker and SB50 (2 – 4 mΩ) should read.
An earlier figure near 25 mΩ came from pairing a voltage minimum with current
samples from a different moment. Comparing the PDH's voltage against the RIO's
to isolate the wiring does not work either: the difference comes out negative,
which is physically impossible, so one of the two sensors is unreliable. If this
needs settling it is a meter across the SB50 and the main breaker under load,
not a log.

### What to do next

1. **Get it on a robot.** Neither the power manager nor the gate has ever
   executed on hardware. First deploy on blocks: watch `Power/Requested State`
   against `Power/Applied State` to confirm the CAN writes land, and
   `Flywheel/Velocity Tolerance` and `Flywheel/At Setpoint` to watch the gate.
2. **Bind a control to `PowerManager.setTurboOverride`.** The state exists and
   nothing reaches it. Until then the only escape is a stick past 0.7 deflection,
   which returns the drivetrain but does not free the battery for it.
3. **Rank the batteries.** `logReview battery` fits internal resistance per
   match, and across ONWEL they ranged 11.2 to 15.0 mΩ — worth 1.14 V of sag at
   300 A, more than three times what the drivetrain cap buys. The last three
   matches of the day had the worst packs. This is the largest single lever and
   it is not a code change.
4. **Set the supply limits that are still missing.** Hood and turret have none.
   The intake's stays off deliberately.
5. **Reconsider the lowered brownout threshold.** `Robot.java` sets 6.5 V against
   a 6.75 V default. Worth knowing first what a RIO brownout actually costs this
   robot: it cuts PWM, DIO and the 5 V/6 V rails, but CAN motor controllers keep
   receiving commands, so on an all-CAN robot the answer may be "very little".

### What the whole thing is worth

Estimated from the logs, so treat as an order of magnitude:

| | |
|---|---|
| Power manager, accuracy | **+2 points** (about 3 shots in 184) |
| Power manager, brownouts | not modelled, and the caps do not fire during them |
| Gate | prevents the ~40 % of hub shots fired outside tolerance from going out |
| Battery selection | 1.14 V, more than three times the drivetrain cap |
| Flywheel current ceiling | ~0, the battery caps it at 112 A before 160 matters |

An earlier version of this section claimed the power manager was worth +9 points.
That was computed with a back-EMF model that went negative at low voltage and
got clamped, inflating the benefit roughly fivefold. The corrected model uses the
measured speed ceiling instead.

**The gate is the intervention that works.** The power manager makes the wait
shorter; it does not make the shots better.

### Tooling

Every number in this section is reproducible:

```
./gradlew logReview --args="power    <dir>"   voltage, brownouts, peak draw per match
./gradlew logReview --args="channels <dir>"   draw attributed to PDH channels
./gradlew logReview --args="motor  <name> <dir>"  one motor by commanded duty
./gradlew logReview --args="modes    <dir>"   supply current by what the robot is doing
./gradlew logReview --args="shots    <dir>"   one row per shot, keyed to the match clock
./gradlew logReview --args="shooter  <dir>"   flywheel shortfall against battery voltage
./gradlew logReview --args="gate     <dir>"   replay a proposed gate over the logs
./gradlew logReview --args="recovery <dir>"   identify acceleration per amp, re-run the dips
./gradlew logReview --args="leadtime <dir>"   how much warning each trigger gives
./gradlew logReview --args="battery  <dir>"   internal resistance per match
./gradlew logReview --args="brownout <dir>"   what was drawing when the bus collapsed
./gradlew logReview --args="ceiling  <dir>"   fastest the flywheel turns at each voltage
./gradlew logReview --args="energy   <dir>"   where a match's energy goes
./gradlew logReview --args="predict  <dir>"   estimate shots landed under different caps
./gradlew logReview --args="wiring   <dir>"   battery resistance against wiring resistance
```

`logReview` takes a list of paths as well as a directory, which matters because
**q5 is bad data** — for 29 s four PDH channels read exactly 71.875 A at once,
about 288 A of phantom draw, which is why its peak reads 508 A. No other match
shows it. Exclude it by passing the others individually.

---

## Highest value (everything else)

### Collision detection and recovery in autos — Open

Wrap each auto path so that being stuck is detected and recovered from, rather
than ground against for the rest of the match.

1678 detects a hit as three debounced conditions AND-ed: below the speed the
trajectory expects, far enough from the final pose that it isn't just
decelerating, and inside the zone where contact is likely. Recovery backs off at
a fixed velocity for 0.4 s, then drives to the trajectory's final pose.

**We already compute the ingredient and use it backwards.** `DriveToPose`
compares tracking error to `AUTO_MAX_TRACKING_ERROR`, and on exceeding it
*lowers its expectations* by adopting actual speeds. So a defender pinning us
produces no recovery at all.

Effort: ~1 day. Check the logs first for auto paths that stalled.

### Extract turret and shot-planning math into pure functions — Open

581 has 12 test files; every one tests pure math in `shared/`, with no HAL, no
simulation and no mocks. That is possible because their math takes every input as
a parameter:

```java
public static double getOptimalAngle(double target, double current,
                                     double minTurretAngle, double maxTurretAngle)
```

Ours does the same job inside `Turret.adjustSetpointForWrap(double)`, which reads
instance state, and inside `ShotPlanner`, which reads `RobotContainer` globals.
Neither can be called from a test.

This unlocks everything else in the shooting section. Effort: ~half a day.

### Goal-centric turret tolerance — Open

`TurretConstants.POSITION_TOLERANCE` is a fixed 12°, and `canFire()` gates
shooting on it. What that means in practice:

| Range | Lateral miss still counted as "aligned" |
|---|---|
| 1.5 m | 0.32 m |
| 3 m | 0.64 m |
| 7 m | **1.49 m** |
| 12.5 m | **2.66 m** |

The flywheel distance map runs to 12.5 m, so at the long end the turret can point
nearly three metres off and `canFire()` still says yes.

581's `getGoalCentricTurretTolerance` takes an acceptable miss **in metres at the
target** and converts it to an angle using current distance — tight when far,
forgiving when close. Roughly 10 lines. Effort: ~1 hour.

---

## Shooting

### Shoot-on-the-move iterations: 1 → 5 — Open

`ShotPlanner.aimAtField` contains `for (int i = 0; i < 1; i++)`. Compensated-goal
convergence is a fixed-point iteration; one pass is a first-order approximation
whose error grows with speed and range. 581 uses 5.

The loop also contains `previousTimeOfFlight != Double.NaN`, which is always true
in Java — a vestigial convergence check that is harmless at one iteration and
wrong the moment the bound is raised.

Effort: minutes. Strictly more accurate.

### Shoot-on-the-move drag model — Open

581 applies `effectiveTof = (1 - e^(-k·tof)) / k`, with `k` tunable. A ball does
not carry the robot's full velocity for its whole flight; using raw time of
flight over-leads the shot, worse at long range. Effort: hours plus a tuning
session.

### Separate radial from tangential velocity — Open

581 splits robot velocity into radial (toward the goal — changes distance, so
hood and flywheel) and tangential (perpendicular — changes lead angle). We lump
both into one `integratedVelocity`. More correct, and it makes the two
compensations independently tunable.

### Smart unwrap near hardstops — Open

Our turret range is ±190°, i.e. **380° total**, so some targets are reachable two
ways. `adjustSetpointForWrap` picks purely on which is closer right now. 581's
`getSmartUnwrapAngle` adds the missing case: if the chosen target lands within a
tolerance band of a hardstop, unwrap to the other side so the turret is not left
with no room to keep tracking. Relevant because we track continuously while
shooting on the move.

### Shot obstruction check — Open, low priority

581's `ObstructionCalculator` (tested, live call sites) decides whether the hub
structure is between the robot and the target. 1678 wrote the same thing and
never called it. Only worth it if blocked shots are actually costing points.

---

## Autonomous

### Auto-detect and recover from beaching — Open

Currently **both halves are missing**:

- Detection is manual — `Controls.beachButton` is the operator's right bumper.
  Someone has to notice and press it.
- Recovery does not exist — `aimBeached()` returns zeros, so we stop shooting and
  nothing else happens.

581 debounces `hypot(pitch, roll) > 5°` for detection, then uses
`atan2(pitch, roll)` to get the tilt direction and drives 1.5 m downhill to get
off.

We already have the inputs: `StateOrchestrator.determineOnBump()` reads
`drive.isLevelOnGround()` and `state.getLatestTilt()`. Effort: ~half a day.

### Gate post-bump shooting on pose stability — Open

1678's bump-crossing maneuver ends with `waitUntil(hasRecentVisionPoseUpdate())`
before shooting — don't fire on a pose the bump just wrecked.

`Vision.isPoseStable()` already exists and currently has no consumer. This is its
obvious first one.

### Explicit bump-crossing maneuver — Open, verify first

1678 treats crossing as a maneuver with distinct up and down phases, a commanded
heading, and **swapped vision standard deviations while crossing**. We solve the
same problem differently with `highTrustEstimatesLeft = 5` after `onGround`.

Only worth building if the logs show bump crossings costing us autos.

---

## Vision

### Measure pose jitter during alignment — Investigate

581, from field experience:

> Multiple Limelights could see the same AprilTag when aligning and output
> slightly different poses. This resulted in the robot's estimated pose
> jittering, which manifested as oscillating around the scoring pose. We updated
> the vision logic to only ingest poses from a single Limelight. Theoretically
> this makes vision worse, but in practice the stability greatly improved
> alignment consistency.

**We fuse two cameras** via `combineEstimates` with inverse-variance weighting.
Mathematically more careful; possibly worse in practice for the same reason.

`LogReview` can measure this from existing match logs. Read-only, and it could
invalidate a design we currently rely on.

---

## Code structure and tooling

### Units convention — Open

Adopt canonical units by convention rather than migrating to WPILib `Angle` /
`Distance` **types**. 581's version is six lines: meters for field positions,
degrees bounded to ±180 for rotations, inches for linear mechanisms with the drum
radius folded into `SensorToMechanismRatio`.

Then fix the known offenders: `Hood`'s `setpoint / 360`, `Flywheel`'s
`RPM_RANGE / 60.0`, and `HoodConstants.DistanceMap`, whose comment says
"rotations" while the values are degrees.

Deliberately **not** using the typed measures: this robot runs a fixed 100 MB
heap with `UseSerialGC`, and each measure operation allocates. The conventions
prevent the same bugs at no runtime cost.

### Give the simulation real physics — Open

`PowerAndGateSimTest` proves the power manager and the shot gate are structurally
sound: the background thread's CAN writes land, every state transition completes,
turbo reaches the motors and releases, and nothing deadlocks against the odometry
lock. What it cannot prove is that any of it *helps*, because the simulation has
no physics to help with.

**What is missing.** `FlywheelIOSim` is a second-order response driven straight
from the setpoint:

```java
simState = new SecondOrderSim(2.5, 1, 0, 0);
talonFXSim.setRotorVelocity(simState.Evaluate(setpointVel, ...));
```

No torque, no current limit, no load, no battery. The simulated wheel reaches any
goal at a fixed rate regardless of what the motor could actually do. Three things
follow, and each is a test we cannot write today:

- **A current limit does nothing in sim.** `SHOOTING` and `SHOOTING_FAR` produce
  identical behaviour, so the entire power manager is untestable beyond "the
  writes landed".
- **The gate closing on a wheel that cannot keep up is untestable.** The sim
  wheel chases any setpoint faster than the 0.25 s falling debounce expires, so
  the case that matters — a long shot on a sagged battery — never occurs.
- **There is no bus voltage**, so nothing browns out and nothing sags.

**Everything needed to build it has already been measured**, which is the reason
this is worth doing rather than guessing at constants:

| Parameter | Value | Where it came from |
|---|---|---|
| Flywheel acceleration | **2.311 rps/s per amp** of stator | fitted from 3918 accelerating samples, `logReview recovery` |
| Speed ceiling | 13.8 rps per volt below ~10 V, flattening near 88 rps | saturation points in `logReview ceiling` |
| Battery | 11 – 16 mΩ, open circuit 11.8 – 12.3 V | fitted from 63k samples, `logReview battery` |
| Energy per ball | median 7.8 rps of speed drop, 90th percentile 18 rps | `logReview shots` |
| Recovery time | 0.07 s for a normal dip, 0.59 s for a deep one | `logReview recovery` |

A model with those five numbers would reproduce the measured behaviour, and every
prediction in the power section above could then be tested before a robot exists
rather than after.

**Partly done.** `GyroIOSim` now exists and can be tilted, which unlocked the
behaviours that key off the gyro rather than the pose — arming the post-bump
vision recovery, forcing `TARGET_TAG` while tilted, and the turret's bump offset.
Those are deliberately gyro-driven because a crossing corrupts the pose and
cannot corrupt gravity, which had made them the one thing simulation could not
reach. `SimHarness.tiltOntoBump()` and `levelOut()` drive it.

`SimHarness.advanceClockOnly` was added alongside, for logic that has to be
driven directly: simulated vision derives its estimates from the drive's own
pose, so it agrees perfectly on every loop and resets any pose-agreement state
machine as fast as a test can set it.

**Also done.** `SimBattery` gives the simulation a pack fitted from the match
logs — 11.79 V open circuit, 15.5 mOhm, so every 100 A costs 1.55 V — and
publishes the result through `RoboRioSim` so the whole robot sees it.
`ModuleIOSim` now clamps to that voltage, reports its draw, and honours the
supply limit the power manager applies. `FlywheelIOSim` was replaced with a model
identified from the logs: 2.311 rps/s per amp, a ceiling of 13.8 rps per volt
capped at 88, and current available in proportion to the room between the wheel
and that ceiling. `SimHarness.takeShot()` removes the measured 7.8 rps a ball
costs.

`SimPhysicsTest` covers what none of this could show before: that current sags
the bus, that spinning up draws far more than sitting at speed, that a tired pack
lowers what the wheel can reach, that a ball costs speed, and that a flywheel
starved by the power manager recovers measurably slower than a fed one
(0.22 s against 0.14 s).

**Still missing.** The pieces below are worth doing if the simulation is ever
relied on further:

- **Vision that can disagree.** `SimVisionIO` derives its estimates from the
  drive's own pose, so it agrees perfectly by construction. No pose-agreement
  threshold can be evaluated in simulation, and the tests that exercise the
  lost-pose logic have to bypass the robot loop entirely with
  `SimHarness.advanceClockOnly`.
- **The other mechanisms.** Intake, indexer and feeder still write velocity
  straight through and load the battery not at all, so the parts of the budget
  that touch them are still inert.

**Shape of it.** A shared battery model that every IO layer draws from, so bus
voltage falls with total current the way it does on the robot; a flywheel model
that converts commanded current into acceleration against that voltage and honours
its own limits; and a ball disturbance that removes a calibrated amount of energy
on command. Each piece is small; the value is in them being connected, because
what makes the real robot hard is that the flywheel's own draw is what starves it.

**Do not do this to make the simulation look realistic.** Do it so that a change
to a current limit or a debounce can be judged without waiting for a field. That
is the only reason it is worth the effort.

### Generic motor subsystem base — Open

Homing is hand-rolled three times — `Intake`, `Hood`, `TurretIOTalonFX` — each
with its own thresholds. 1678's `ServoMotorSubsystem` takes a `ServoHomingConfig`
and does it once.

Do this **after** the units work, or the unit confusion gets baked into the new
abstraction. Must preserve our `@AutoLog` inputs contract; 1678's version has no
AdvantageKit, so it is a port rather than a copy.

### Subsystem bringup checklist — Open

581's `subsystem-bringup-checklist.md` is excellent and mostly portable: gains at
zero, `PositionVoltage` instead of `MotionMagicVoltage` so profiling doesn't hide
PID behaviour, verify motor direction by hand-turning and watching the log,
confirm gearing by moving the mechanism and comparing logged position. Belongs as
a project skill. Pays off next build season.

### Delete `AlignToPose` dead code — Open, needs a decision

115 lines of commented-out code, zero call sites, and the only reason Spotless
needs a per-file exclusion. `AutoAlignToPose` is also unreferenced. Not deleted
because it isn't mine to delete.

### Small cleanups — Open

- `formatter.xml` sets `comment.line_length` to 80 while code wraps at 120, which
  is what makes comment reflow dominate reformat diffs. Setting both to 120 would
  shrink future diffs.
- Flip Spotless `enforceCheck` to `true` once the team is used to the pre-commit
  hook.
- `AdaptableManager` and `RobotContainer` compare chooser names with `==`. It
  works by string-literal identity; use `.equals()` in anything new.
- The adaptable auto's sweep and attack option lists are duplicated by hand
  across the first- and second-pass choosers, and the labels have already drifted
  (`"Rotate Out"` vs `"No Sweep"` for the same segment).

---

## Verification outstanding

**Nothing on `offseason` has run on a robot.** In rough order of how much a first
deploy should watch them:

- **`PowerManager`** writes motor configurations over CAN from a background
  thread, which nothing in this repo did before. On blocks, confirm
  `Power/Applied State` follows `Power/Requested State` and that the divergence
  alert stays down.
- **The readiness gate** changes when the robot is willing to shoot. Expect it to
  hold fire more than before, especially past 3.5 m — that is the intent, but the
  drive team should see it before a match rather than during one.
- **The `Ports` migration**: enable, confirm every motor responds, glance at CAN
  utilization. `PortsTest` proves every ID and bus is unchanged, but that is a
  test, not a robot.
- **Turret motor logging** was uncommented, so `/Inputs/Turret/TurretMotor`
  should appear and should show sensible current while aiming.

---

## Done

| Item | Source | Notes |
|---|---|---|
| `Ports` enum pairing CAN ID with bus | 1678 | Frozen by `PortsTest`; drivetrain stays in generated `TunerConstants` |
| Spotless + reformat | 1678 | 20 files; vendor, generated, obsolete and `AlignToPose` excluded |
| Pre-commit formatting hook | 581 | Installs via Gradle plugin; stages only what Spotless changed |
| `src/test/` and a test suite | 1678 | Was zero tests |
| `SimHarness` — headless whole-robot tests | — | Boot, step, enable, press buttons |
| `SimLog` / `LogReview` | 581 pin a Claude skill for the same job | Read sim and real match logs |
| Vision pose agreement (`isPoseStable`) | 1678 | No consumer yet — see the post-bump gate above |
| Allocation and eager-init guidance in skills | 581 | |
| Origin stories in skills | 581 | Every rule should say what going without it cost |
| Fixed: auto-generation NPE, crossed indexer feeder handles, sim feeder handle | — | Found while reading |

---

## Rejected

Recorded so they are not re-derived.

| Item | Why not |
|---|---|
| **Debounced alignment completion** | `Autopilot.atTarget()` is a stateless predicate with no velocity term, so in principle it can finish while moving. Measured on the ONWEL logs: of 18 alignment ends, 8 stopped properly, 9 were interruptions mid-travel, and **one** ended at the target while still moving, at 0.33 m/s. Not worth guarding. Re-run `./gradlew logReview --args="align <dir>"` before revisiting. |
| **`TunableNumber` / live PID retuning** | Phoenix Tuner X already writes gains to motors directly, which covers the fast tuning loop. Note that robot code reapplies full configs at construction, so a Tuner value survives only until the next code restart. |
| **`ShotVerifier`** | 1678's version is a pre-shot viability gate, not shot-outcome feedback. Its two novel parts (range check, obstruction geometry) are dead code in their own repo, and the live parts — alignment and tilt — we already have in `canFire()`, checking the turret rather than the chassis. |
| **WPILib `Angle`/`Distance` measure types** | Allocation cost on a 100 MB `SerialGC` heap. Use unit conventions instead. |
| **Vendor-agnostic `CameraIO`** | Days of refactor to enable swapping Limelight for PhotonVision, which has never been needed. |
| **Per-axis alignment tolerances, richer alignment telemetry** | No mechanism needs one axis weighted differently, and `LogReview` answers the telemetry questions after the fact. |
| **Explicit-dt PID for replay determinism** | 1678's own convenience overload calls `getFPGATimestamp()`, the exact call our logging rules ban. |
| **Porting Trailblazer** | Our autopilot path following works and is competition-proven, and the adaptable system composes autos at runtime — 581 hand-writes every combination. The transferable idea is their tracker/follower separation, not the library. |
| **State machines instead of commands** | 581 rejects long-running commands after being burned by command bugs. The critique of `configureBindings()` lands, but converting is a rewrite, not a refactor. |
| **"Simulated vision is metres off"** | Retracted. It was measured with a harness missing `HAL.simPeriodicBefore/After` and against a stale logged pose, because WPILOG only records values on change. With both fixed, the fused pose tracks sim truth to ~5 cm. |

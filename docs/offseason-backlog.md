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
mostly do not set. Intake's is commented out in `IntakeIOTalonFX`; hood, turret,
climber and the indexer rollers have none.

### One caveat on the raw peaks

Match q5 is bad data. For 29 s of it, PDH channels 0, 1, 18 and 19 all read
exactly 71.875 A simultaneously — about 288 A of phantom draw, which is why its
"peak" reads 508 A. No other match shows it. `channels` prints `SUM of channels`
next to `TotalCurrent` for exactly this reason: when they disagree, the PDH
reported something impossible. Exclude q5 by passing the other logs
individually; `logReview` accepts a list of paths.

### What to do

1. **Power manager** — the 581 pattern: define each robot state as a complete
   supply-current budget across every subsystem, and reapply limits on transition
   from a background thread. Capping supply current attacks the peak directly.
   Limits today are set once in `configure*()` and never change.
2. **Set the supply limits that are missing, before anything clever.** Intake's
   is commented out; hood, turret and climber have none. This is a small change
   with a measured justification.
3. **Reconsider the lowered brownout threshold.** `Robot.java` sets 6.5 V, down
   from the 6.75 V default, "to give more headroom before outputs cut out". With
   minimums measured at 6.28 V that is masking sags rather than preventing them,
   and running the RIO closer to its limit has its own risks.

Re-run the measurement after any change — the tooling exists.

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

**Nothing on `offseason` has run on a robot.** The `Ports` migration is the one to
sanity-check on first deploy: enable, confirm every motor responds, and glance at
CAN utilization. `PortsTest` proves the ID and bus of every device is unchanged
from before the migration, but that is a test, not a robot.

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

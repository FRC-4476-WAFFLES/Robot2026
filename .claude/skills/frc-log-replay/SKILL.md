---
name: frc-log-replay
description: Use when asking "would this change have fixed that match?" — replaying a real AdvantageKit .wpilog through modified robot code to A/B a change against a match that already happened, including the setup that makes replay run at all and the limits of what the answer means.
---

# Replaying Match Logs Through New Code

AdvantageKit logs every input. Feed a log back in and robot code recomputes every
output from the same inputs — so a change to a filter, a threshold, or a
controller can be tested against a match that already happened, instead of
against a guess.

The catch is at the bottom of this page and it is not small. Read it before you
report a result.

## Running one

```bash
AKIT_LOG_PATH=/path/to/match.wpilog ./gradlew simulateJava
```

Writes `match_sim.wpilog` next to the input. Outputs land under `ReplayOutputs/`;
the original run's values are still there under `RealOutputs/`, in the same file.
That side-by-side is what makes an A/B possible.

Three things had to be true for this to work, and all three are already in the
repo — don't undo them:

| Where | What | Why |
|---|---|---|
| `Constants.simMode` | `AKIT_LOG_PATH` set → `Mode.REPLAY` | Nothing else picks the mode; the env var is the switch. |
| `build.gradle` | Sim GUI + DriverStation extensions registered only when `AKIT_LOG_PATH` is blank | The HAL sim extensions fight the replayed DriverStation and the run dies at startup. |
| `Robot.simulationPeriodic` | Returns immediately unless `Mode.SIM` | WPILib calls it in replay too. `RobotContainer.simState` is null there (NPE), and if it weren't, the physics would fight the logged inputs it is meant to be reproducing. |

## Validate before you believe anything

**Replay a log through unmodified code first and check `ReplayOutputs` matches
`RealOutputs`.** If they don't, the divergence is a determinism bug in robot
code, not evidence about your change — and any A/B on top of it is noise.

For this repo's pose work: median error 0.000 m, max 0.036 m against
`RobotState/FieldPose`. That is the bar.

Divergence means something reads outside the log — a `getFPGATimestamp()`, a
`Math.random()`, a hardware handle read directly instead of through `inputs`.
See the `frc-advantagekit-logging` skill.

## Designing the A/B

This is where the work actually is. The mechanics above are easy; the metric is
what gets it wrong.

Run the log twice — once unmodified (baseline), once with the change — and
compare the two `ReplayOutputs` against a truth anchor. For pose, the anchor is
`RealOutputs/Vision/<cam>/Megatag Accepted`: a correction the original run
trusted, which is the closest thing to ground truth a log contains.

**Measure only the windows where the baseline was actually failing.** Averaging
over a whole autonomous answers a question nobody asked. In q59 the change looked
worthless at −0.03 m averaged over 302 windows, and worth 1.25 m in the one
window that lost the match — because 95% of the match had vision every other
loop, and those windows swamped the six that mattered.

The pattern that worked: find gaps in the truth anchor longer than ~1.5 s, and
compare each run's pose at the *end* of the gap against the fix that then landed.
That is the moment the baseline had drifted furthest with nothing to correct it,
which is the moment the change exists for.

A metric that credits the baseline for a correction it just absorbed measures
nothing. If the baseline's median error comes out near 0.00 m, the sample is
dominated by windows where it was never in trouble — redesign it.

## What replay does not tell you

**Replay reproduces the estimate, not the trajectory.** Module positions come
from the log, so the robot follows exactly the path it followed on the field. A
change that makes the pose estimate correct does not, in replay, make the robot
drive anywhere different — the path follower's new output goes nowhere.

So replay answers "would the robot have *known* where it was" — which is worth a
lot when the failure was the robot not knowing where it was. It does not answer
"would it then have driven correctly." Say which one you measured.

Same shape for anything downstream of an actuator: a shooter-speed change replays
the *command*, not the balls.

## Related

- `frc-advantagekit-logging` — the determinism rules replay depends on.
- `docs/log-deep-dive.md` — the milstein bump/odometry findings this was built for.

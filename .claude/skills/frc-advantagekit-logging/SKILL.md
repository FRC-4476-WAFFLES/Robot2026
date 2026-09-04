---
name: frc-advantagekit-logging
description: Use when logging robot data, reading a timestamp or Driver Station value, adding a sensor or coprocessor input, using threads or Notifiers, or reading NetworkTables/SmartDashboard in this robot project — violations of these rules break log replay silently.
---

# AdvantageKit Logging & Replay

This robot logs every input and replays logs in the simulator to reproduce a match exactly. Replay only works if robot code is **deterministic**: given the same logged inputs, it must produce the same outputs. Nothing warns you when you break this. The code compiles, the robot drives, and the log is quietly useless for debugging the one match that mattered.

**Breaking the letter of these rules is breaking replay.** There is no "this one call is small enough."

## The three ways to log

| Use | For | Where |
|---|---|---|
| `@AutoLog` on an `IOInputs` class | Everything read **from hardware** | The `*IO` interface |
| `@AutoLogOutput(key = "...")` | A field or zero-arg getter you want logged every loop | Subsystem / `RobotState` |
| `Logger.recordOutput("...", value)` | A value computed inside a method | Anywhere |

```java
@AutoLogOutput(key = "Flywheel/Flywheel Goal Velocity")
private double flywheelGoalVelocity = 0;

Logger.recordOutput("Turret/Distance To Target", distanceToTarget);
```

Prefer `@AutoLogOutput` for state that persists between loops; it can't be forgotten on a new code path. Use `recordOutput` for intermediate values inside a computation.

### Key naming

Slash-separated, `Title Case With Spaces` for the leaf. Established roots — use these, don't invent siblings:

| Root | Contents |
|---|---|
| `Inputs/<Subsystem>` | **Reserved** for `Logger.processInputs`. Never `recordOutput` here. |
| `<Subsystem>/...` | Per-subsystem outputs: `Turret/`, `Flywheel/`, `Hood/`, `Drive/`, `Vision/`, `Climber/` |
| `RobotState/...` | Robot-wide state, poses, velocities, trajectories |
| `RobotState/Timing (ms)/...` | Written by `EpochTimer` — don't write here directly |
| `Commands/<name>` | Written by the `CommandScheduler` hooks in `Robot` — don't write here directly |

Put units in the key only when the value's unit isn't obvious from its name, and use the `(unit)` suffix form already in use — `RobotState/Timing (ms)/`. `Turret/Distance To Target` (meters, the repo default) needs no suffix; an elapsed-time or temperature reading earns one. Never abbreviate the leaf into camelCase — `At Setpoint`, not `atSetpoint`.

## The input rule

**Any value that comes from outside robot code must enter through an IO layer's `updateInputs`, be logged with `Logger.processInputs`, and be read from the `inputs` object.**

"Outside robot code" means motors, encoders, gyros, limit switches, cameras, coprocessors, LaserCAN, and NetworkTables.

```java
@Override
public void periodic() {
  io.updateInputs(inputs);
  Logger.processInputs("Inputs/Foo", inputs);
  double position = inputs.fooMotorData0.position();   // read inputs
  // NOT: io.getMotor().getPosition().getValueAsDouble()
}
```

Reading straight from `io` or from a hardware handle inside the subsystem bypasses the log. During replay that call returns nothing and the control logic diverges.

This applies to plain WPILib sensors as much as to motors. A `DigitalInput`, `Encoder`, or `AnalogInput` belongs in the IO implementation with a bare primitive on the inputs class — `public boolean beamBroken = false;` — following `GyroIO`'s `connected` / `tipAngle` fields. Don't wrap it in a record; there is no `DigitalInputData` and you shouldn't invent one.

Two consequences that bite:

- **Derived state from a sensor is not itself an input.** Edge counts, debounce state, and elapsed timers are computed in the subsystem from `inputs` and logged with `@AutoLogOutput`. Only the raw reading crosses the IO boundary.
- **Stub the sensor in the sim IO.** An unconnected DIO reads a constant, so a beam break never trips and the logic downstream of it can't be exercised in sim. Use `DIOSim`. Do **not** reach for `RobotContainer.simState` from a shared IO path — it is `null` in REAL and REPLAY and will NPE on the robot.
- **Put derived tracking above the disabled guard** if it should keep running while disabled. The guard `return`s, so anything below it stops during disable — which is wrong for a piece counter you want working while hand-feeding in the pit, and right for anything that commands hardware.

## Timestamps

```java
Timer.getTimestamp()          // ✅ deterministic, replayed from the log
Timer.getFPGATimestamp()      // ❌ real wall clock, breaks replay
```

AdvantageKit replaces `Timer.getTimestamp()` with the logged loop timestamp during replay. `getFPGATimestamp()` always returns the real FPGA clock.

**This repo currently has 13 `Timer.getFPGATimestamp()` calls across 7 files** (`RobotState`, `EpochTimer`, `WafflesUtilities`, `DeferredRefresher`, `LimelightIO`, `SimVisionIO`, `ModuleIOSim`). They are pre-existing debt, not the standard. Do not copy them into new code because the surrounding file uses them — `Drive.earlyPeriodic` and `Turret.periodic` use `Timer.getTimestamp()`, which is correct. Converting an existing call is a welcome fix, but changes odometry timebase alignment, so raise it rather than doing it silently as part of an unrelated change.

Subsystems are constructed in `RobotContainer`'s static initializer, which runs before `Logger.start()`. You therefore **cannot seed a timestamp field at construction**. Use a sentinel and initialize lazily:

```java
private double lastEventTimestamp = -1;   // not Timer.getTimestamp()
```

## Banned in robot control paths

| Don't | Why | Instead |
|---|---|---|
| `Timer.getFPGATimestamp()` | Real clock, not replayed | `Timer.getTimestamp()` |
| `Math.random()`, `new Random()` | Can't be reproduced | Seed explicitly, or make it an input |
| Reading `SmartDashboard` / NetworkTables as an input | Dashboard state isn't in the log | Route through an IO layer, or use `LoggedNetworkNumber` |
| `DriverStation` or `Timer` before `Logger.start()` | AdvantageKit isn't capturing yet | Do it after, in `RobotContainer` |
| Iterating a `HashMap` / `HashSet` where order affects output | Iteration order varies per run | `LinkedHashMap`, or sort first |
| Raw `new Thread` / `Notifier` touching subsystem state | Racy relative to the loop | Publish through a synchronized snapshot read once per loop |
| Vendor libraries that drive hardware directly (YAGSL, Phoenix swerve API) | Bypass the IO layer entirely | The repo's own `Module` / `ModuleIO` layer |

`SmartDashboard.getNumber` **is** used in `ShotPlanner.aimManual()` for hand tuning, gated behind `CodeConstants.MANUAL_SHOOTER_TUNING`. That's a deliberate tuning-only escape hatch. Don't extend the pattern to match logic.

`DeferredRefresher` uses a `Notifier` on purpose, for a blocking read that would otherwise overrun the loop. It hands results back through a `synchronized` `Optional` read once per loop. Copy that shape if you need another one; don't share mutable state across a thread boundary any other way.

## Adding an input

```
- [ ] Field added to the @AutoLog inputs class, initialized to a sane default (never left null)
- [ ] Populated in every IO implementation's updateInputs, including the sim one
- [ ] Read only via `inputs.<field>` in the subsystem
- [ ] Logger.processInputs called under an "Inputs/..." key
```

An uninitialized (null) input field throws during replay when AdvantageKit tries to deserialize it. Give every field a default — that's why the inputs classes construct `new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0)` rather than leaving it null.

## Rationalizations — all of these mean stop

| Thought | Reality |
|---|---|
| "It's just for a log message, not control" | Replay compares outputs. A diverging output is a broken replay. |
| "The file already uses `getFPGATimestamp`" | Existing debt isn't the standard. New code uses `getTimestamp()`. |
| "It's only in simulation" | Sim code runs in replay too. `ModuleIOSim` and `SimVisionIO` are the files this bites. |
| "It's temporary, for tuning" | Gate it behind a `CodeConstants` flag like `MANUAL_SHOOTER_TUNING`, or it isn't temporary. |
| "Reading the motor directly is one line shorter" | It's one line that removes the subsystem from replay. |
| "Nobody replays logs anyway" | Then the logging cost is already being paid for nothing. Replay is the reason the IO layer exists. |

## Verifying

Deploy or simulate, pull the `.wpilog`, and replay it. Outputs in the `ReplayOutputs` table should match `RealOutputs`. `./gradlew replayWatch` runs the watcher. Do this after adding a subsystem, not the week of an event.

## Related

- Structure of an IO layer: `frc-subsystem-pattern` skill.
- Formatting and naming: `frc-code-style` skill.

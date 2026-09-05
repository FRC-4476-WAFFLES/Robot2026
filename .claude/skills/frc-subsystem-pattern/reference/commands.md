# Commands Reference

## Contents

- The two shapes (there is no third)
- Static factory
- Builder class with `follow()`
- Composition idioms used in this repo
- Requirements and proxies

## The two shapes (there is no third)

**This repo contains zero `Command` subclasses.** `AlignToPose extends Command` exists only as a commented-out line. Do not write one — use one of these two shapes.

Both alignment files are also **dead**: `AlignToPose` is entirely commented out, and `AutoAlignToPose` has zero call sites. All real alignment goes through `DriveToPose`, reached via `DriveCommands.autoToTarget` / `autoToFieldPose`. Don't take either as a pattern to copy, and don't assume editing them changes robot behaviour.

| Shape | When | Examples |
|---|---|---|
| Static factory on a `*Commands` class | The command is a fixed composition | `ShooterCommands.shootCommand()`, `IntakeCommands.intakeCommand()`, `DriveCommands.joystickDrive()` |
| Plain class holding a `Command`, exposed via `follow()` | The command needs per-instance configuration or mutable cross-loop state | `DriveToPose`, `AutoPath`, `PassThroughTarget`, `WheelRadiusCharacterization` |

## Static factory

Lives in `commands/<area>/<Area>Commands.java`. Reaches subsystems through `RobotContainer`, so it takes no subsystem parameters — except `DriveCommands`, which takes `Drive` because it was ported from Mechanical Advantage.

```java
public static Command backoffIndexer() {
  return Commands.startEnd(
      () -> RobotContainer.indexer.runIndexer(IndexerState.REVERSE),
      () -> RobotContainer.indexer.stopIndexer(),
      RobotContainer.indexer)
      .withTimeout(0.25)
      .withName("Backoff Indexer");
}
```

Utility classes get a private constructor: `private DriveCommands() {}`.

## Builder class with `follow()`

The command is built **once in the constructor** and returned by `follow()`. Configuration is `withX()` methods returning `this`, applied before `follow()` is called.

```java
public class AutoPath {
  private int targetIndex = 0;            // mutable state across loops
  private boolean preciseFinish = false;
  private final Command cmd;

  public AutoPath(BlueRelativeTarget... targets) {
    this.targets = targets;
    cmd = followPath();                   // built once
  }

  public AutoPath withPreciseFinish() {
    preciseFinish = true;
    return this;
  }

  public Command follow() {
    return cmd;
  }

  private Command followPath() { ... }
}
```

Two consequences of building in the constructor:

- **`withX()` called after construction still works** only because the built command's lambdas read the field at execution time, not construction time. Keep configuration in fields the lambda closes over, never captured as a local.
- **State must be reset in `beforeStarting`**, since the same `Command` object is reused across schedulings:

```java
.beforeStarting(() -> {
  targetIndex = 0;
  onFinalTarget = false;
  startingPose = state.getPose();
});
```

Java requires captured locals to be effectively final, so mutable cross-loop values are either instance fields or a mutable object the lambda mutates in place. `DriveToPose.copyChassisSpeeds` exists purely for this — it writes into an existing `ChassisSpeeds` rather than reassigning the local. `DriveCommands` uses a small `private static class` holder (`WheelRadiusCharacterizationState`) for the same reason.

## Composition idioms used in this repo

```java
Commands.run(...)               // repeats until interrupted; the default
Commands.runOnce(...)           // one shot
Commands.startEnd(a, b)         // run a on start, b on end
Commands.runEnd(a, b)           // repeat a, run b on end
Commands.sequence(...)          // in order
Commands.parallel(...)          // all, until all finish
Commands.deadline(main, ...)    // all, until `main` finishes — the auto workhorse
Commands.repeatingSequence(...) // sequence, looped
Commands.waitUntil(...)         // gate
```

Decorators, in the order they're normally chained:

```java
.until(cond)          .withTimeout(s)      .onlyWhile(cond)
.beforeStarting(...)  .finallyDo(...)      .repeatedly()
.asProxy()            .withName("...")
```

**Always `.withName("...")`.** The name is what appears in the `Commands/` log table and in AdvantageScope. An unnamed composition logs as an opaque generated name.

`.asProxy()` is needed when a composed command requires a subsystem that the outer composition must not hold for its whole duration — see `ShooterCommands.shootCommand()`, where the wheel-lock sub-command is proxied so the repeating sequence can release `drive` between iterations.

## Requirements and proxies

Pass requirements as the trailing varargs of `Commands.run`/`runEnd`/`startEnd`:

```java
Commands.runEnd(
    () -> intake.setIntakeDutyCycle(OUTTAKE_DUTY_CYCLE),
    () -> intake.setIntakeDutyCycle(0),
    intake)                                  // requirement
```

Omit the requirement when the command only writes a goal that the subsystem's own `periodic()` applies and you want it to coexist with other writers — several shooter bindings in `configureBindings()` deliberately do this. That is a real decision, not an oversight: adding a requirement there would make the trigger bindings cancel each other.

**A `VirtualSubsystem` can never be a requirement.** It isn't a `SubsystemBase`. `Vision`, `Telemetry`, `MechanismPoses`, `StateOrchestrator`, and `Lights` cannot be passed to `Commands.run(...)`.

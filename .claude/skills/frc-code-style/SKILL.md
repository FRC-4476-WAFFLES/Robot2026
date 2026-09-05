---
name: frc-code-style
description: Use when writing, editing, or reviewing any Java file in this robot project — covers this repo's formatting, naming, import order, file headers, comment style, and code hygiene rules.
paths: src/main/java/**/*.java
---

# Java Style

Team 4476 WAFFLES robot code — WPILib + AdvantageKit; `build.gradle` names the season. Match the surrounding file; these are the repo-wide defaults when the file gives no signal.

## Formatting

| Rule | Value |
|---|---|
| Indent | 2 spaces. Never tabs — there are zero tab characters in `src/`. |
| Line width | 120 columns. Wrap, don't let it run. |
| Continuation indent | 4 spaces (two levels). |
| Braces | K&R — `{` on the same line, `} else {` on one line. |
| Final newline | Not required. |

Older files under `utils/` had drifted to 4-space indentation; Spotless has since pulled them back. If you see indentation drift again, run `./gradlew spotlessApply` rather than matching it.

**Run `./gradlew spotlessApply` before committing.** Spotless formats from the build using the same `formatter.xml` VS Code uses, so it works without an editor — which matters because format-on-save never runs for an agent writing files directly. `./gradlew spotlessCheck` reports violations without changing anything.

`enforceCheck` is `false`, so a formatting violation will not fail the build. That makes running `spotlessApply` your job, not the build's.

Do not hand-align code into columns; let the formatter wrap. Use `// spotless:off` … `// spotless:on` to protect a block that genuinely needs manual layout.

Four paths are excluded in `build.gradle` and must stay that way: vendor copies (`LimelightHelpers`, `utils/vendor/**`) so they stay diffable against upstream, generated files (`BuildConstants`, `TunerConstants`), `**/obsolete/**`, and `AlignToPose.java` — a dead file whose large block of commented-out code dominated the initial reformat diff for no benefit. If that dead code is ever deleted, drop the exclusion.

## File header

Every new file starts with the WPILib BSD header:

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
```

Files ported from Mechanical Advantage (`Drive.java`, `DriveCommands.java`, `Module*.java`) keep the Littleton Robotics header instead. Preserve whichever header a file already has.

Some existing files have no header. That's drift, not an exemption — most files, including most `*IO.java` files, do have one. Add the header to new files; don't go add it to old ones as a drive-by.

## Imports

Six groups, blank line between each, alphabetical within a group. This is what VS Code's `source.organizeImports` produces — run it rather than hand-sorting.

```java
import static edu.wpi.first.units.Units.Meters;   // 1. static

import java.util.Optional;                        // 2. java

import org.littletonrobotics.junction.Logger;     // 3. org

import com.ctre.phoenix6.hardware.TalonFX;        // 4. com

import edu.wpi.first.math.geometry.Pose2d;        // 5. edu + frc, merged
import frc.robot.RobotContainer;

import lombok.Getter;                             // 6. lombok
```

Never use wildcard imports except the `edu.wpi.first.units.Units.*` static members.

The imports nearly every subsystem needs, with their real paths:

```java
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;                          // RobotContainer.state, .drive, ...
import frc.robot.data.Constants.FlywheelConstants;        // nested classes imported individually
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;
import frc.robot.utils.lib.EpochTimer;
import frc.robot.utils.lib.WafflesUtilities;
import frc.robot.utils.lib.subsystems.ExpandedSubsystem;
```

`TalonFXIOData` is a **record**, so its accessors have no `get` prefix: `inputs.fooMotorData0.velocity()`, `.position()`, `.torqueCurrent()`.

## Naming

| Kind | Convention | Example |
|---|---|---|
| Class, enum, record | `PascalCase` | `ShotPlanner`, `TurretSetpoint` |
| Method | `camelCase` | `runSetpoint`, `atSetpoint` |
| Field, local | `camelCase` | `flywheelGoalVelocity` |
| Tuning constant in `Constants.java` | `SCREAMING_SNAKE` | `MOTOR_STATOR_CURRENT_LIMIT` |
| CAN device in `Ports` enum | `SCREAMING_SNAKE` | `FLYWHEEL_MOTOR_0`, `HOOD_MOTOR` |
| Enum member | `SCREAMING_SNAKE` | `TARGET_HUB`, `FULLY_AGITATING` |
| IO interface | `<Subsystem>IO` | `FlywheelIO` |
| IO implementation | `<Subsystem>IO<Backend>` | `FlywheelIOTalonFX`, `FlywheelIOSim` |
| Generated inputs | `<Subsystem>IOInputsAutoLogged` | never write this class by hand |

**The one deliberate exception:** static helpers in `utils/lib/` and `utils/hardware/` use `PascalCase` — `WafflesUtilities.Lerp`, `FlipIfRedAlliance`, `EpochTimer.BeginEpoch`, `PhoenixHelpers.RegisterStatusSignals`. When adding to one of those existing classes, match its neighbours. Everywhere else, `camelCase`.

One naming exception that is real and must not be "fixed": `Controls.java` uses the WPILib `kPrefix` form (`kLeftJoystickPort`). Keep that within `Controls`.

"At target" checks are named `atSetpoint()` — `Flywheel`, `Hood`, and `ConstrainedMechanism` have it. `Turret` has overloaded `atSetpoint(...)` taking explicit tolerances plus `atGoal()`, which checks against a motion-profile *goal state*. Don't invent a third name.

Log keys are `Subsystem/Title Case With Spaces` — `@AutoLogOutput(key = "Flywheel/At Setpoint")`, `Logger.processInputs("Inputs/Flywheel", inputs)`. Full key conventions are in the `frc-advantagekit-logging` skill.

## Comments

Javadoc on public methods whose purpose isn't obvious from the name, and on every class in `utils/lib/`. One-line `/** ... */` is normal and preferred over a three-line block for a short note.

```java
/**
 * Attempts the desired task more than once if failed, up to a limit
 * @param maxAttempts The max number of attempts
 * @param task A supplier for the config function
 */
```

Inline `//` comments explain **why**, not what. Physical/tuning facts are exactly the kind of comment worth keeping:

```java
// Lower brownout threshold from default (6.75V) to give more headroom before
// outputs cut out. Only applies on real hardware — RIO 1 ignores this call.
```

Do not add comments that restate the code.

## Hygiene

**A subsystem never holds a hardware handle.** No `new TalonFX(...)`, `DigitalInput`, `CANcoder`, or vendor sensor object inside a `SubsystemBase`. Hardware lives in a `*IO` implementation; the subsystem reads an `inputs` object. This is the single most common structural mistake in this repo's domain, and it silently removes the subsystem from log replay. If you are about to construct hardware outside a `*IO` class, stop and use the `frc-subsystem-pattern` skill.

**Constants go in `Constants.java`.** The decision rule:

| Value | Home |
|---|---|
| Something a driver/operator retunes between matches — current limit, setpoint, tolerance, distance map, gear reduction | `Constants.java`, in the matching nested class |
| PID/feedforward gains | Inline in the `*IOTalonFX` `configure*()` method. Every existing subsystem does this; the `*Constants.MOTOR_kP` fields are mostly stale leftovers. |
| A number only one method's algorithm cares about | `private static final` at the top of the owning class — `Flywheel.RECOVERY_FF_PER_RPS`, `DriveCommands.FF_RAMP_RATE` |

A bare magic number in the middle of a method is the thing to avoid. Existing tolerance constants are named for what they measure — `POSITION_TOLERANCE`, `AUTO_POSITION_TOLERANCE_VAGUE` — not bare `TOLERANCE`.

**Every motor gets a current limit.** `CurrentLimitsConfigs` with `withStatorCurrentLimit(...)` + `withStatorCurrentLimitEnable(true)` in the IO layer's `configure*()`, applied through `PhoenixHelpers.tryConfig(...)` so a CAN failure raises the config-error flag instead of passing silently.

**Don't leave commented-out code in new work.** The repo has a lot of it already (`AlignToPose.java` alone has 115 such lines). Don't add more, and don't delete existing blocks that aren't yours — several are parked tuning values.

**Warnings and errors go to the Driver Station**, not stdout, for anything an operator needs at an event:

```java
DriverStation.reportWarning("...", false);   // operator-visible
new Alert("AUTO WINNER NOT SET", AlertType.kError).set(true);  // dashboard
```

`System.out.println` is acceptable only for startup/characterization output that nobody reads mid-match.

**Don't touch `utils/obsolete/`, `subsystems/lights/obsolete/`, or `LimelightHelpers.java`.** The first two are dead code kept for reference; `LimelightHelpers` is a vendor file copied verbatim and must stay diffable against upstream.

**Only remove imports, fields, or methods that your own change made unused.** Pre-existing dead code stays.

## Related

- Adding or changing a subsystem, command, or auto: use the `frc-subsystem-pattern` skill.
- Anything that logs or reads a timestamp: use the `frc-advantagekit-logging` skill — its determinism rules are not optional.

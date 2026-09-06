---
name: frc-subsystem-pattern
description: Use when adding or restructuring a subsystem, mechanism, IO layer, command, trigger, constant, vision camera, or autonomous routine in this robot project, or when wiring new hardware into RobotContainer.
---

# Robot2026 Architecture Recipes

How this repo is put together, and the shape a new piece has to match.

## Layer map

```
Robot.robotPeriodic()
  └─ PhoenixHelpers.refreshAllSignals()      // one CAN refresh for every device
  └─ RobotContainer.state.updateEnabledState()
  └─ ExpandedSubsystemManager.RunEarlyPeriodic()   // Drive odometry, ConstrainedMechanism
  └─ CommandScheduler.run()                        // SubsystemBase.periodic(), commands
  └─ ExpandedSubsystemManager.RunLatePeriodic()    // virtual subsystems, then latePeriodic()
```

Anything ordering-sensitive belongs in `earlyPeriodic` (needs fresh sensor data before commands run) or `latePeriodic` (needs the setpoint commands just wrote). Plain `periodic()` is the default.

## Keep all code running all the time

Avoid work that only starts after some event — lazy initialisation, first-use construction, warmup that happens on the first press. It moves cost into the middle of a match, where a loop overrun is expensive, instead of into startup where nobody cares.

`RobotContainer`'s constructor calls `ShotPlanner.aimManual()` purely to force static initialisation of the shot planner's splines before the match rather than on the first shot. Do the same for anything expensive and lazily built.

## Which base class

| Base | Use for | Gets |
|---|---|---|
| `SubsystemBase` | Ordinary mechanism a command can `require` | `periodic()` |
| `ExpandedSubsystem` | Same, but needs early or late periodic | `+ earlyPeriodic()`, `latePeriodic()` |
| `VirtualSubsystem` | Software-only: vision, telemetry, state logic, mechanism poses. **Cannot be required by a command.** | `periodic()` (runs before other late periodics), `earlyPeriodic()`, `latePeriodic()` |
| `ConstrainedMechanism` | Mechanism whose setpoint must be clamped by named, logged constraints | `applySetpoint()`, `applyConstraints()`, `runConstraint()` |

Registration is automatic — each base class registers itself with `ExpandedSubsystemManager` in its constructor. Don't register by hand.

## The subsystem trio

Every hardware subsystem is three (or four) files in its own package. Standalone mechanisms sit at the top level — `subsystems/drive/`, `subsystems/vision/` and so on. Mechanisms that are part of a larger assembly nest one level under it, as the current robot's shooter does with `subsystems/shooter/flywheel/`, `shooter/hood/`, `shooter/turret/`. Put a new mechanism under an area folder only if it's genuinely part of that assembly; list `subsystems/` to see what this season's robot actually has.

**1. `FooIO.java`** — the interface. Inputs class carries data only; every method is a `default` no-op so `new FooIO() {}` is a valid null implementation for replay.

```java
package frc.robot.subsystems.foo;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface FooIO {
  @AutoLog
  class FooIOInputs {
    public TalonFXIOData fooMotorData0 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(FooIOInputs inputs) {}

  default void runDutyCycle(double speed) {}

  default void runFooVelocity(double velocity) {}
}
```

Reuse `TalonFXIOData` **for motor data** rather than loose `double` fields — it keeps position/velocity/voltage/current/temp consistent and is what `TalonFXIO.getSignalData()` returns. Give every motor its own field, including followers (`flywheelMotorData0` / `flywheelMotorData1`), or the follower vanishes from the logs.

**Non-motor sensors use bare primitives**, following `GyroIO`:

```java
@AutoLog
class FooIOInputs {
  public TalonFXIOData fooMotorData0 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  public boolean beamBroken = false;      // DigitalInput
  public double rangeMillimeters = 9999;  // LaserCAN
  public Rotation2d angle = Rotation2d.kZero;
}
```

There is no `DigitalInputData` record and you should not invent one. Every field needs a non-null default — a null input throws during replay deserialization.

**2. `FooIOTalonFX.java`** — real hardware. Motors are `TalonFXIO` (not raw `TalonFX`), declared `protected` so the sim class can reach them. Control request objects are constructed once as fields, never per-loop. Configuration lives in `private void configureFooMotor()` and goes through `PhoenixHelpers.tryConfig`.

```java
public class FooIOTalonFX implements FooIO {
  // Hardware Components
  protected final TalonFXIO foo0;

  // Control Objects
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  public FooIOTalonFX() {
    foo0 = new TalonFXIO(Ports.FOO_MOTOR_0);
    configureFooMotor();
  }

  @Override
  public void updateInputs(FooIOInputs inputs) {
    inputs.fooMotorData0 = foo0.getSignalData();
  }

  @Override
  public void runFooVelocity(double velocity) {
    foo0.setControl(velocityRequest.withVelocity(velocity));
  }

  private void configureFooMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Constants.FooConstants.MOTOR_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    configs.Feedback.SensorToMechanismRatio = PhysicalConstants.FOO_REDUCTION;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 11;
    slot0Configs.kS = 5.5;
    configs.Slot0 = slot0Configs;

    PhoenixHelpers.tryConfig(() -> foo0.getConfigurator().apply(configs));
  }
}
```

Using `TalonFXIO` is what registers the device's status signals for the single grouped `refreshAllSignals()` call and sets update frequencies. A raw `TalonFX` silently costs CAN bandwidth and won't be refreshed.

### CAN devices go in the `Ports` enum

Every CAN device is one entry in `data/Ports.java`, pairing its ID with the bus it lives on:

```java
HOOD_MOTOR(23, Bus.RIO),
TURRET_MOTOR(25, Bus.CANIVORE),
```

Construct with the port, never a bare ID — `new TalonFXIO(Ports.HOOD_MOTOR)`, `new CANcoderIO(Ports.TURRET_ENCODER_0, 250)`. CAN IDs are only unique *per bus*, so an ID alone doesn't identify a device; passing the port makes it impossible to omit the bus. The raw `(int)` and `(int, CANBus)` constructors still exist for vendor code but should not be used for new devices.

The bus is not just addressing — it sets the signal refresh rate (`FD_CAN_FREQUENCY` 100 Hz on the CANivore vs `BASE_CAN_FREQUENCY` 50 Hz on the RIO), so getting it wrong silently halves your data rate even if the device responds.

Drivetrain devices are the exception: they live in `TunerConstants` (IDs 1-12, CANivore) because that file is generated by CTRE Tuner X and loses hand edits. `PortsTest` guards the rest against duplicate IDs on a bus.

**Robot code owns motor configuration.** Every `configure*()` applies a full `TalonFXConfiguration`, gains included, at construction — so anything written to a motor with Phoenix Tuner X is overwritten the next time robot code starts. Tuner is the right tool for the fast tuning loop, but a value only survives once it is copied back into the `configure*()` method.

Gains that live in software rather than on the motor are outside Tuner's reach entirely and need a redeploy: `DriveCommands.ANGLE_KP` / `ANGLE_KD` (used by joystick-at-angle and `DriveToPose`), `SlewRateLimiter` rates, `Turret`'s `TrapezoidProfile` constraints, and the autopilot velocity/acceleration/jerk limits.

Stator limits are mandatory. Add a supply limit as well for anything that can stall or draw hard against a game piece — `Flywheel`, `Intake`, and the `Indexer` feeder do; `Hood`, `Turret`, and `Climber` are stator-only. Match the mechanism, don't cargo-cult either shape.

### Follower motors

A second motor mechanically coupled to the first is a `Follower` request, constructed once as a field. The repo is split 2/2: `Indexer` (both pairs) uses `Aligned`, `Intake` and `Flywheel` use `Opposed`.

**`MotorAlignmentValue` is about rotor direction relative to the leader, not output direction.** "Both wheels spin the same way" is a different question from "both rotors spin the same way" whenever there's an odd gear stage or an opposed mounting between them. Two motors facing each other across a shooter wheel need `Opposed` to drive the wheel one way. Confirm against the mechanism, and if you can't, say so rather than guessing — this is the one mistake in this file that destroys hardware.

```java
protected final TalonFXIO foo0;
protected final TalonFXIO foo1;
private final Follower followerRequest;

public FooIOTalonFX() {
  foo0 = new TalonFXIO(Ports.FOO_MOTOR_0);
  foo1 = new TalonFXIO(Ports.FOO_MOTOR_1);
  followerRequest = new Follower(foo0.getDeviceID(), MotorAlignmentValue.Aligned);
  configureFooMotor();
}

@Override
public void updateInputs(FooIOInputs inputs) {
  inputs.fooMotorData0 = foo0.getSignalData();
  inputs.fooMotorData1 = foo1.getSignalData();   // follower still gets logged
}

@Override
public void runFooVelocity(double velocity) {
  foo0.setControl(velocityRequest.withVelocity(velocity));
  foo1.setControl(followerRequest);              // re-sent every loop
}
```

Both motors get the same `TalonFXConfiguration`, applied with **two separate** `tryConfig` calls:

```java
PhoenixHelpers.tryConfig(() -> foo0.getConfigurator().apply(configs));
PhoenixHelpers.tryConfig(() -> foo1.getConfigurator().apply(configs));
```

### Units

`configs.Feedback.SensorToMechanismRatio = PhysicalConstants.FOO_REDUCTION` means **every position and velocity the subsystem sees is in mechanism units** — mechanism rotations and rotations/sec, not motor units. Pick tolerances and setpoints accordingly. `PhysicalConstants` reductions are motor rotations per mechanism rotation.

**3. `FooIOSim.java`** — `extends FooIOTalonFX`, overrides `updateInputs` to push simulated state into the real Phoenix sim object, then calls `super`. This is the repo's convention (see `FlywheelIOSim`, `HoodIOSim`) — the sim reuses the real configuration and control path so tuning transfers.

`SecondOrderSim(frequency, damping, response, startingPosition)` — a critically damped velocity mechanism is roughly `(2.5, 1, 0, 0)`. `Evaluate(setpoint, dt)` returns a `Vector<N2>`; index it with `.get(0)` for velocity. Build it in the constructor, not as a field initializer.

`foo0.getSimState()` works because `TalonFXIO extends TalonFX` — it is the Phoenix sim handle, not a wrapper API of our own.

For a non-motor sensor, the sim IO must stub it or the sim reads an unconnected port forever (a DIO reads constant `true`, so a beam break never trips and downstream logic can't be tested). Use WPILib's `DIOSim` / `EncoderSim`.

**Do not reach for `RobotContainer.simState` from an IO layer.** It is `null` in REAL and REPLAY modes:

```java
public static SimState simState = (Constants.getMode() == Mode.SIM) ? new SimState() : null;
```

An `*IOSim` class only ever runs in SIM so it is safe there, but a shared IO path that touches it will NPE on the robot.

```java
public class FooIOSim extends FooIOTalonFX {
  private SecondOrderSim simState;
  private double setpointVel = 0;

  public FooIOSim() {
    simState = new SecondOrderSim(2.5, 1, 0, 0);
  }

  @Override
  public void updateInputs(FooIOInputs inputs) {
    var simResult = simState.Evaluate(setpointVel, CodeConstants.PERIODIC_LOOP_TIME);
    foo0.getSimState().setRotorVelocity(simResult.get(0) * PhysicalConstants.FOO_REDUCTION);
    super.updateInputs(inputs);
  }

  @Override
  public void runFooVelocity(double velocity) {
    setpointVel = velocity;
    super.runFooVelocity(velocity);
  }
}
```

**4. `Foo.java`** — the subsystem. Holds the goal, not the command. `periodic()` reads inputs, guards on disabled, writes the output.

```java
public class Foo extends SubsystemBase {
  private final FooIO io;
  private final FooIOInputsAutoLogged inputs = new FooIOInputsAutoLogged();

  @AutoLogOutput(key = "Foo/Goal Velocity")
  private double goalVelocity = 0;

  public Foo(FooIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    EpochTimer.BeginEpoch("Foo");
    {
      io.updateInputs(inputs);
      Logger.processInputs("Inputs/Foo", inputs);

      if (!RobotContainer.state.robotEnabled()) {
        io.runFooVelocity(0);
        return;
      }

      io.runFooVelocity(goalVelocity);
    }
    EpochTimer.EndEpoch("Foo");
  }

  // Public API
  public void runSetpoint(double velocity) {
    goalVelocity = velocity;
  }

  @AutoLogOutput(key = "Foo/At Setpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.fooMotorData0.velocity() - goalVelocity) < FooConstants.VELOCITY_TOLERANCE;
  }
}
```

`TalonFXIOData` is a record — accessors are `velocity()`, `position()`, `torqueCurrent()`, with no `get` prefix.

For anything a firing sequence gates on, prefer the debounced form `Flywheel` uses — it stops the check chattering across the tolerance boundary:

```java
private final Trigger atSetpointTrigger = new Trigger(
    () -> Math.abs(inputs.fooMotorData0.velocity() - goalVelocity) < FooConstants.TOLERANCE)
    .debounce(0.25);
```

Four things that are load-bearing here:

- The subsystem reads `inputs`, never `io` directly. That is what makes replay work.
- `EpochTimer.BeginEpoch`/`EndEpoch` wrap the body with a bare `{ }` block. Every subsystem does this; it publishes per-subsystem loop time to `RobotState/Timing (ms)/`.
- The disabled guard zeroes the output and returns early. Without it a stale goal is re-applied the instant the robot enables.
- **Known quirk:** that `return` sits inside the `EpochTimer` block, so `EndEpoch` is skipped on disabled loops and no timing sample is recorded until the robot enables. `EpochTimer` tolerates this — the next `BeginEpoch` just overwrites `lastTime`, nothing leaks or throws. Five subsystems do it (`Flywheel`, `Hood`, `Intake`, `Indexer`, `Turret`). Match the convention; don't quietly "fix" it in an unrelated change, and don't extend it to a guard that needs the timing sample.
- "Public API" is a comment convention marking the section commands call. Setters store a goal; the periodic applies it.

## Wiring into RobotContainer

Subsystems are `public static final` fields assigned in the static initializer, switched on `Constants.getMode()`. All three branches must be filled — `REPLAY` uses anonymous null IO.

```java
public static final Foo foo;

static {
  switch (Constants.getMode()) {
    case REAL:
      foo = new Foo(new FooIOTalonFX());
      // ... other subsystems, one blank line between each

      break;

    case SIM:
      foo = new Foo(new FooIOSim());

      break;

    default:
      // Replayed robot, disable IO implementations
      foo = new Foo(new FooIO() {});

      break;
  }
}
```

Virtual subsystems are constructed after the switch, in dependency order, because they read from the hardware subsystems.

## State and triggers

Persistent robot state lives in `RobotState`, not in subsystems. Add a field with `@AutoLogOutput`, use Lombok `@Getter`/`@Setter` for plain accessors, and expose behavior as a `Trigger` factory:

```java
@Getter
@Setter
@AutoLogOutput(key = "RobotState/Outtake Desired")
private boolean outtakeDesired = false;

public Trigger shouldOuttake() {
  return new Trigger(() -> !isIntaking && !isShooting && outtakeDesired);
}
```

Bind that trigger to behavior in `RobotContainer.configureBindings()`, not inside the subsystem:

```java
state.shouldOuttake().whileTrue(
    Commands.runEnd(
        () -> intake.setIntakeDutyCycle(IntakeConstants.OUTTAKE_DUTY_CYCLE),
        () -> intake.setIntakeDutyCycle(0),
        intake).withName("Outtake"));
```

`StateOrchestrator` (a `VirtualSubsystem`) derives state from field position each loop. Position-driven mode changes belong there, not in `configureBindings`.

Always `.withName("...")` a composed command — the name is what appears in the `Commands/` log table and in AdvantageScope.

## Commands

Commands are static factories on a `*Commands` class in `commands/<area>/`, returning composed `Commands.*` builders. They reach subsystems through `RobotContainer`, so they take no subsystem parameters except where a WPILib API forces it (`DriveCommands` takes `Drive`).

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

**Never write a `Command` subclass.** This repo has none — the one attempt (`AlignToPose`) is commented out. When a command needs per-instance configuration or state that survives across loops, use a plain class that builds its `Command` in the constructor and exposes it via `follow()`, like `DriveToPose`, `AutoPath`, and `PassThroughTarget`.

Full command reference — the two shapes, composition idioms, requirements and proxies: [reference/commands.md](reference/commands.md)

## Constants

Add a nested `public static class FooConstants` in `data/Constants.java`.

| Value | Home | Naming |
|---|---|---|
| Any CAN device (ID + bus) | `data/Ports.java` enum | `SCREAMING_SNAKE` — `FLYWHEEL_MOTOR_0` |
| Gear reduction | `PhysicalConstants` | `SCREAMING_SNAKE` |
| Feature flag / loop tuning | `CodeConstants` | `SCREAMING_SNAKE` |
| Mechanism limits, tolerances, setpoints | `FooConstants` | `SCREAMING_SNAKE` |
| PID / feedforward gains | **Inline** in `configure*()` | n/a |
| DIO port | see below | — |

There is no DIO convention yet. `Constants` has an empty `DigitalOutputs` class under a `/* Digital Ports */` comment; it's misnamed for sensor inputs. Add a sibling `DigitalInputs` class rather than putting an input port in `DigitalOutputs`, and use SCREAMING_SNAKE like the `Ports` enum.

Named positions are enums with a value and a getter:

```java
public enum ExpanderPosition {
  STOWED(0.0),
  EXTENDED(97.0);

  private final double degrees;

  ExpanderPosition(double degrees) {
    this.degrees = degrees;
  }

  public double getDegrees() {
    return degrees;
  }
}
```

Distance-to-output lookups are `NodePoint[]` arrays consumed by `SplineMonotone1D` (see `FlywheelConstants.DistanceMap`), not `if`/`else` ladders.

## Autonomous

An auto is a `SequentialCommandGroup` in `autos/`, with field positions as `BlueRelativeTarget` fields and `AutoUtils.resetOdometry(start)` first:

```java
public class Preload extends SequentialCommandGroup {
  BlueRelativeTarget start = new BlueRelativeTarget(3.450, 4, Rotation2d.fromDegrees(90));
  BlueRelativeTarget end = new BlueRelativeTarget(2.5, 4, Rotation2d.fromDegrees(90));

  public Preload() {
    addCommands(
        AutoUtils.resetOdometry(start),
        DriveCommands.autoToTarget(end),
        ShooterCommands.shootAutoCommand(5));
  }
}
```

Register it in `RobotContainer.configureCommandChoosers()`. Never hardcode red-alliance coordinates — `BlueRelativeTarget` flips at read time.

There is a second, larger auto system in `autos/adaptable/` that assembles a routine at runtime from dashboard-chosen segments. Its caching contract is easy to get wrong (a chooser without `.onChange(InvalidateCache)` silently does nothing). Adding a segment, adding a chooser option, or writing a new variation: [reference/autos.md](reference/autos.md)

PathPlanner autos are **disabled** (`CodeConstants.USE_PATHPLANNER_AUTOS = false`). Don't add `.path` files expecting them to run.

## Checklist for a new subsystem

```
- [ ] Device added to the Ports enum with its bus; PortsTest updated and passing
- [ ] FooIO.java: @AutoLog inputs, all methods `default` no-op, every field non-null default
- [ ] One inputs field per motor INCLUDING followers; sensors as bare primitives
- [ ] FooIOTalonFX.java: TalonFXIO fields (correct CAN bus), request objects as fields
- [ ] Follower: MotorAlignmentValue Aligned vs Opposed confirmed against the gearbox
- [ ] Stator current limit set and enabled; supply limit too if it can stall
- [ ] configure*() applied per motor via PhoenixHelpers.tryConfig
- [ ] FooIOSim.java extends FooIOTalonFX, calls super; non-motor sensors stubbed
- [ ] Foo.java: reads `inputs` not `io`, EpochTimer block, disabled guard, @AutoLogOutput on goals
- [ ] Tolerances in mechanism units (SensorToMechanismRatio is applied)
- [ ] Reduction in PhysicalConstants, tuning in FooConstants
- [ ] RobotContainer: public static final field + all three switch branches incl. REPLAY
- [ ] Triggers in RobotState, bindings in configureBindings(), .withName() on compositions
- [ ] Builds: ./gradlew build

`FooIOInputsAutoLogged` does not exist until the annotation processor runs. The editor will show it as unresolved until the first successful build after you add the `@AutoLog` class — build before believing the error.
```

## Reference files

Load the one that matches the task; each is self-contained.

| File | Covers |
|---|---|
| [reference/commands.md](reference/commands.md) | The two command shapes, composition idioms, requirements, `asProxy`, effectively-final workarounds |
| [reference/autos.md](reference/autos.md) | `BlueRelativeTarget`, `AutoPath`, the adaptable system, segments and choosers, the cache-invalidation contract |
| [reference/vision.md](reference/vision.md) | `TagCamera`, moving-camera transforms, estimate fusion, the single-fuse rule, adding a camera |

## Related

- Formatting, naming, imports: `frc-code-style` skill.
- Logging keys and the replay determinism rules: `frc-advantagekit-logging` skill.

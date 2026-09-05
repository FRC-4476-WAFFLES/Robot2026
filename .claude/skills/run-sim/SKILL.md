---
name: run-sim
description: Use when running this robot project's simulator or its unit tests — launching simulateJava, connecting AdvantageScope, writing or running JUnit tests, or diagnosing MsvcRuntimeException, UnsatisfiedLinkError, or a JVM crash in msvcp140.dll.
---

# Running the Sim and Tests

## The one rule that makes everything work

**Every Gradle command must run on the WPILib JDK.** `gradle.properties` already does this, so plain `./gradlew` works.

```bash
./gradlew <task>
```

WPILib's JNI libraries check the MSVC runtime bundled *inside the JDK*, not the system Visual C++ redistributable. Most JDKs ship an old one. VS Code's WPILib extension passes this automatically, which is why sim works from the extension and fails from a terminal.

Symptoms of the wrong JDK, all the same cause:

| Symptom | Where |
|---|---|
| `edu.wpi.first.util.MsvcRuntimeException: Invalid MSVC Runtime Detected` | `simulateJava` |
| JVM hard-crashes, `EXCEPTION_ACCESS_VIOLATION` in `msvcp140.dll`, writes `hs_err_pid*.log` | tests that touch HAL |
| `UnsatisfiedLinkError: no CTRE_PhoenixTools in java.library.path` | tests touching CTRE without HAL (see below) |

Installing the system VC++ redistributable does **not** fix this. Only the JDK matters. The project targets Java 17 and the WPILib JDK is Temurin 17; a system Java 21 is wrong on both counts.

This is already handled — `gradle.properties` at the repo root sets `org.gradle.java.home` to the WPILib JDK, so plain `./gradlew` works. If you are not on Windows, change that one line to `<your home>/wpilib/2026/jdk`. If any of the symptoms above appear, that file is the first thing to check.

## Running the simulator

```bash
./gradlew simulateJava
```

**This blocks until the sim is closed.** Run it in the background and poll the log rather than waiting on it.

Startup is working when you see:

```
HAL Extensions: Attempting to load: halsim_gui
********** Robot program starting **********
NT: Listening on NT3 port 1735, NT4 port 5810
Logger initialized. Robot program starting...
********** Robot program startup complete **********
```

Normal noise, not errors:

- `Joystick Button N on port M not available` — repeats forever with no controller plugged in
- `simulationPeriodic(): 0.000516s` — loop timing, printed by the sim GUI

### Connecting AdvantageScope

This is the team's normal way to see what the robot is doing, and it needs no setup: **File → Connect to Simulator**.

Sim publishes over **NT4**, deliberately the same as the real robot, so connecting works identically at a desk and at an event. `RLOGServer` also runs on port 5800 if you prefer it — set AdvantageScope's live source to **RLOG Server** — but NT4 is the default path and the one to reach for.

`assets/AdvantageScopeLayouts/` holds saved layouts, and `assets/AdvantageScopeAssets/Robot_Leo` is the 3D robot model — point AdvantageScope's asset folder at these to get the field and mechanism visualisation. `MechanismPoses` publishes to `RobotState/MechanismPoses` for the 3D view.

Every sim run and every `SimHarness` test already writes a `.wpilog` to `simlogs/` (gitignored). Open one in AdvantageScope directly, feed it to `./gradlew replayWatch`, or read it in a test with `SimLog` — see below.

## Running tests

```bash
./gradlew test
./gradlew test --tests 'frc.robot.data.*'
```

Tests live in `src/test/java/`, mirroring the main package layout. **`deploy` does not depend on `test`**, so tests never slow a deploy down.

### Two kinds of test

**Pure logic — no HAL.** Anything that doesn't touch hardware or CTRE. These run on any JDK and are fast. This is where most value is: shot planning math, spline interpolation, `AutoPath.ShouldAdvanceToNextTarget`, field-coordinate flipping, `Ports` invariants.

**HAL / simulation.** Anything touching a CTRE object, `CANBus`, a motor, or WPILib hardware classes. These need the WPILib JDK **and** HAL initialized before the first CTRE call:

```java
public class MySimTest {
  @BeforeAll
  static void setupHAL() {
    assertTrue(HAL.initialize(500, 0), "HAL initialization failed");
  }

  @AfterAll
  static void teardownHAL() {
    HAL.shutdown();
  }
}
```

Skip `HAL.initialize` and Phoenix looks for the real native library instead of the `_Sim` one — `build/jni/release` contains only `CTRE_PhoenixTools_Sim.dll`, so you get `UnsatisfiedLinkError`. The error names the non-sim library, which is the giveaway.

### Whole-robot tests: `SimHarness`

`src/test/java/frc/robot/SimHarness.java` boots the **entire robot** headlessly — every subsystem, every sim IO layer, AdvantageKit logging — and steps it a loop at a time. This is how a change gets verified without a robot, a GUI, or a driver station.

```java
public class MyFeatureTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  @Test
  void flywheelReachesItsSetpoint() {
    SimHarness.enableTeleop();

    RobotContainer.flywheel.runSetpoint(40.0);
    SimHarness.stepSeconds(3.0);
    assertTrue(RobotContainer.flywheel.atSetpoint());

    RobotContainer.flywheel.runSetpoint(0.0);
    SimHarness.disable();
  }
}
```

API: `boot()`, `step(loops)`, `stepSeconds(s)`, `enableTeleop()`, `enableAutonomous()`, `disable()`. `RobotBootSimTest` is the worked example.

### Pressing buttons: covering `configureBindings()`

The harness attaches simulated controllers at boot, so trigger bindings can be exercised without a human on a joystick. This is the only way to cover `RobotContainer.configureBindings()`.

```java
SimHarness.enableTeleop();
SimHarness.releaseAllControls();

SimHarness.tapButton(SimHarness.RIGHT_JOYSTICK, 2, 2);   // press, hold 2 loops, release
SimHarness.step(2);
assertTrue(RobotContainer.state.isManualMode());

SimHarness.setAxis(SimHarness.LEFT_JOYSTICK, 0, 0.75);   // 0 = X, 1 = Y
SimHarness.setPov(SimHarness.OPERATOR, 0);               // D-pad up; -1 releases
```

Ports are `LEFT_JOYSTICK`, `RIGHT_JOYSTICK`, `OPERATOR`, taken from `Controls` so they cannot drift. Button numbers are 1-based, matching `CommandJoystick.button(n)` and `XboxController.Button.kA.value`. On the operator's Xbox controller the triggers are **axes** 2 and 3, not buttons. `ControlsSimTest` is the worked example.

Three things to get right:

- **Enable first.** Commands do not run while disabled, so a binding will not fire even though the trigger flips. Call `enableTeleop()` before pressing anything.
- **Call `releaseAllControls()` at the start of each test.** Controller state persists across test methods within a class.
- **Step at least one loop after a press.** Triggers are polled by the `CommandScheduler`, so nothing happens until the next loop. `setButton`/`setAxis`/`setPov` each step once for you; give a command more loops if it needs them.

The harness declares 16 buttons, 6 axes and 1 POV per port at boot. Without those counts the driver station rejects every read with `Joystick Button N on port M not available` and no binding can ever fire.

### Reading the log back: `SimLog`

Sim runs write a `.wpilog` to `simlogs/`. `SimLog` reads it, so a test can assert on **anything the robot logged** — not just what a subsystem exposes through a public getter. This is the tool for verifying internals.

```java
SimHarness.shutdown();          // flush and close the log
SimLog log = SimLog.openLatest();

log.fields();                            // every field name, sorted
log.fieldsMatching("Flywheel");          // discover what exists
log.maxDouble("/RealOutputs/Flywheel/Flywheel Goal Velocity");
log.everTrue("/RealOutputs/Flywheel/At Setpoint");
```

Field names are the AdvantageScope keys **with a leading slash** — `/Inputs/...` for IO layer data, `/RealOutputs/...` for `recordOutput` and `@AutoLogOutput`. Use `fieldsMatching` to discover rather than guessing.

`SimLog.openInAdvantageScope(path)` launches AdvantageScope on a log for a human to look at. Use it to hand over a result, not to verify one — an agent should assert on values, not on a rendered graph.

Four constraints, each learned the hard way:

- **One robot per JVM.** `RobotContainer`'s subsystems are `static final`, so a robot is built once and cannot be rebuilt after `shutdown()`. `build.gradle` sets `forkEvery = 1` so each test class gets a fresh JVM. Within a class, state carries between methods — leave the robot disabled when you finish.
- **`@AutoLogOutput` fields need `AutoLogOutputManager.addObject(robot)`.** `LoggedRobot` does this inside `loopFunc`, which the harness bypasses, so `SimHarness.boot()` does it explicitly. Without it those fields are silently absent from the log while `recordOutput` fields still appear — which looks like the annotation is broken rather than the harness.
- **Loops run in real time — a 3-second wait takes 3 seconds.** Prefer asserting that a subsystem reached a state over waiting a fixed settling time.
- **Never use `SimHooks.pauseTiming()` / `stepTiming()` here.** `PhoenixOdometryThread` waits on CAN signals; with the sim clock paused that wait never returns while it holds `Drive.odometryLock`, so `Drive.earlyPeriodic()` deadlocks and the test hangs until the build times out.
- **The harness must drive AdvantageKit's loop hooks**, which it does by reflection because they are package-private. Calling `robot.robotPeriodic()` alone leaves `Timer.getTimestamp()` frozen at zero, so every debouncer, timer and motion profile stalls silently — `atSetpoint()` on a debounced trigger simply never becomes true. If AdvantageKit renames those hooks the harness throws a clear error rather than quietly freezing time again.

## Reviewing real match logs

Simulation cannot tell you what the robot actually did at an event. `LogReview`
streams a real `.wpilog` and answers questions about it:

```bash
./gradlew logReview --args="fields C:/path/to/match.wpilog"    # what did that code log?
./gradlew logReview --args="align  C:/path/to/logdir"          # how did alignments end?
```

Pass **Windows-style paths**; a Git Bash `/c/...` path reaches Java as `C:\c\...` and fails.

Three things it exists to encode:

- **Stream, don't load.** A match log is 50-100MB. `SimLog` builds a map of boxed values and is fine for a short sim run but will thrash on a match log — `LogReview` keeps only the entry IDs it needs.
- **Structs are raw bytes.** `Pose2d` and `ChassisSpeeds` serialise as consecutive little-endian doubles (x, y, theta / vx, vy, omega). Decode them directly; `SimLog` skips struct fields entirely.
- **Logs are often truncated.** A robot that loses power mid-write leaves a partial final record, and `DataLogReader` throws part-way through iteration. Catch it and keep what you read, rather than losing the whole file.

Field names change between seasons and between deploys. Always run `fields` on an
actual log before assuming a key exists — older logs here predate
`RobotState/Autopilot/Active` entirely.

## Simulation lies about some things

Sim is not the robot. Known divergences:

- **`CANBus.isNetworkFD()` returns `true` for every bus in simulation**, including the RIO's, which is physically CAN 2.0. This is why `Ports.isCANFD()` declares bus type rather than querying it. `CanBusSimTest` asserts the lie still exists, so the workaround can be removed if CTRE fixes it.
- **An unconnected `DigitalInput` reads a constant**, so a beam break never trips. Stub sensors with `DIOSim` — see the `frc-advantagekit-logging` skill.
- **`RobotContainer.simState` is null outside SIM mode.** Never touch it from a shared IO path.

A passing sim test is evidence, not proof. Say so when reporting results.

## Checklist for "does this change work?"

```
- [ ] ./gradlew build           (compiles, runs tests)
- [ ] ./gradlew test            (all green)
- [ ] a SimHarness test exercises the changed behaviour, or SimLog asserts on the logged field
- [ ] simulateJava reaches "startup complete" with no exception
- [ ] AdvantageScope shows the field/values the change should affect
- [ ] State plainly that this was not run on hardware
```

All of these run on the WPILib JDK via `gradle.properties`.

## Related

- Logging keys, determinism, sensor stubs: `frc-advantagekit-logging`
- Subsystem and IO structure: `frc-subsystem-pattern`

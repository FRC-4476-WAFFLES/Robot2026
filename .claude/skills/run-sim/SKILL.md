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

This is the team's normal way to see what the robot is doing, and it needs one setting changed from the default.

`Robot.java`'s SIM branch adds **`RLOGServer`** and leaves `NT4Publisher` commented out. AdvantageScope's default live source is NetworkTables 4, so **File → Connect to Simulator** with default settings connects to NT4 on 5810 and shows only plain dashboard values — none of the AdvantageKit `Inputs/` or `RealOutputs/` fields.

Two ways to fix, pick one:

| Option | How | Trade-off |
|---|---|---|
| Point AdvantageScope at RLOG | AdvantageScope preferences → **Live Source: RLOG Server** (port 5800), then Connect to Simulator | No code change; matches what the robot already publishes |
| Publish NT4 in sim too | Uncomment `Logger.addDataReceiver(new NT4Publisher())` in `Robot.java`'s SIM case | AdvantageScope works with defaults; slightly more sim overhead |

`assets/AdvantageScopeLayouts/` holds saved layouts, and `assets/AdvantageScopeAssets/Robot_Leo` is the 3D robot model — point AdvantageScope's asset folder at these to get the field and mechanism visualisation. `MechanismPoses` publishes to `RobotState/MechanismPoses` for the 3D view.

To capture a sim run for later replay instead of watching live, uncomment `WPILOGWriter("simlogs/")` in the SIM branch, then open the `.wpilog` in AdvantageScope or feed it to `./gradlew replayWatch`.

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

Four constraints, each learned the hard way:

- **One robot per JVM.** `RobotContainer`'s subsystems are `static final`, so the robot initializes once and every test class in a run shares it. `boot()` is idempotent. Put the robot into the state you need at the start of each test; never assume defaults. Leave it disabled when you finish.
- **Loops run in real time — a 3-second wait takes 3 seconds.** Prefer asserting that a subsystem reached a state over waiting a fixed settling time.
- **Never use `SimHooks.pauseTiming()` / `stepTiming()` here.** `PhoenixOdometryThread` waits on CAN signals; with the sim clock paused that wait never returns while it holds `Drive.odometryLock`, so `Drive.earlyPeriodic()` deadlocks and the test hangs until the build times out.
- **The harness must drive AdvantageKit's loop hooks**, which it does by reflection because they are package-private. Calling `robot.robotPeriodic()` alone leaves `Timer.getTimestamp()` frozen at zero, so every debouncer, timer and motion profile stalls silently — `atSetpoint()` on a debounced trigger simply never becomes true. If AdvantageKit renames those hooks the harness throws a clear error rather than quietly freezing time again.

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
- [ ] a SimHarness test exercises the changed behaviour, if it is testable
- [ ] simulateJava reaches "startup complete" with no exception
- [ ] AdvantageScope shows the field/values the change should affect
- [ ] State plainly that this was not run on hardware
```

All of these run on the WPILib JDK via `gradle.properties`.

## Related

- Logging keys, determinism, sensor stubs: `frc-advantagekit-logging`
- Subsystem and IO structure: `frc-subsystem-pattern`

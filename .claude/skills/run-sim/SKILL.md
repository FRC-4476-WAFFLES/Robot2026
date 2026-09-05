---
name: run-sim
description: Use when running this robot project's simulator or its unit tests — launching simulateJava, connecting AdvantageScope, writing or running JUnit tests, or diagnosing MsvcRuntimeException, UnsatisfiedLinkError, or a JVM crash in msvcp140.dll.
---

# Running the Sim and Tests

## The one rule that makes everything work

**Every Gradle command must run on the WPILib JDK.**

```bash
./gradlew -Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk" <task>
```

WPILib's JNI libraries check the MSVC runtime bundled *inside the JDK*, not the system Visual C++ redistributable. Most JDKs ship an old one. VS Code's WPILib extension passes this automatically, which is why sim works from the extension and fails from a terminal.

Symptoms of the wrong JDK, all the same cause:

| Symptom | Where |
|---|---|
| `edu.wpi.first.util.MsvcRuntimeException: Invalid MSVC Runtime Detected` | `simulateJava` |
| JVM hard-crashes, `EXCEPTION_ACCESS_VIOLATION` in `msvcp140.dll`, writes `hs_err_pid*.log` | tests that touch HAL |
| `UnsatisfiedLinkError: no CTRE_PhoenixTools in java.library.path` | tests touching CTRE without HAL (see below) |

Installing the system VC++ redistributable does **not** fix this. Only the JDK matters. The project targets Java 17 and the WPILib JDK is Temurin 17; a system Java 21 is wrong on both counts.

To avoid typing the flag, add `gradle.properties` at the repo root:

```properties
org.gradle.java.home=C:/Users/Public/wpilib/2026/jdk
```

That file is machine-specific — it breaks anyone whose WPILib lives elsewhere or who isn't on Windows. It is not committed. Decide per machine.

## Running the simulator

```bash
./gradlew -Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk" simulateJava
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

This is the team's normal way to see what the robot is doing. In AdvantageScope: **File → Connect to Simulator**. No configuration; it finds NT4 on localhost:5810.

`Robot.java` sends `RLOGServer` in SIM mode rather than `NT4Publisher`, so AdvantageKit outputs arrive over RLOG. Fields appear under `RealOutputs/` and `Inputs/`. `simgui.json` and the `assets/AdvantageScopeLayouts` folder hold saved layouts.

To log a sim run to a file for later replay, uncomment the `WPILOGWriter("simlogs/")` line in `Robot.java`'s SIM branch.

## Running tests

```bash
./gradlew -Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk" test
./gradlew -Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk" test --tests 'frc.robot.data.*'
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
- [ ] simulateJava reaches "startup complete" with no exception
- [ ] AdvantageScope shows the field/values the change should affect
- [ ] State plainly that this was not run on hardware
```

Both the build and the sim need the JDK flag.

## Related

- Logging keys, determinism, sensor stubs: `frc-advantagekit-logging`
- Subsystem and IO structure: `frc-subsystem-pattern`

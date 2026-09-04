# Vision Reference

## Contents

- Structure
- The IO layer and why it looks unusual
- `TagCamera` and the moving-camera transform
- Estimate fusion and the single-fuse rule
- Validation and standard deviations
- Adding a camera
- Gotchas

## Structure

```
subsystems/vision/
  Vision.java        VirtualSubsystem — fuses estimates, feeds the pose estimator
  TagCamera.java     Per-camera logic: reads inputs, builds a TagPoseEstimate
  VisionIO.java      @AutoLog inputs mirroring the Limelight result structs
  LimelightIO.java   Real hardware, via LimelightHelpers
  SimVisionIO.java   Synthesises tag observations from a pose supplier
utils/vision/
  LimelightHelpers.java   Vendor file, copied verbatim — do not edit
  VisionHelpers.java      Pose/stddev validation predicates
```

`Vision` is a `VirtualSubsystem`, so it runs in `earlyPeriodic` — before commands — and **cannot be required by a command**.

## The IO layer and why it looks unusual

`VisionIOInputs` mirrors the Limelight structs as **records declared in the interface**, because `LimelightHelpers.PoseEstimate` isn't loggable:

```java
public record PoseEstimateRecord(
    Pose2d pose, double timestampSeconds, double latency, int tagCount,
    double tagSpan, double avgTagDist, double avgTagArea, boolean isMegaTag2) {}

public record RawFiducialRecord(
    int id, double txnc, double tync, double ta,
    double distToCamera, double distToRobot, double ambiguity) {}
```

Two things that look wrong but are deliberate:

- **`rawFiducials` is a fixed 32-length array with a separate `fiducialArrayLength`.** AdvantageKit needs a fixed-size, non-null array to serialize. The constructor pre-fills all 32 entries. Read only up to `fiducialArrayLength`; the rest is stale garbage.
- **`updateInputs` takes a second parameter**, `DoubleFunction<Transform3d> cameraOffset`. This is the one IO method in the repo that takes anything besides the inputs object, because the turret camera's robot-relative transform depends on the turret angle *at the frame's timestamp*.

## `TagCamera` and the moving-camera transform

Each `TagCamera` is constructed with an IO, a name, and a `DoubleFunction<Transform3d>` giving the camera's robot-space transform at a timestamp:

```java
frameCamera = new TagCamera(frameIO, LIMELIGHT_NAME_FRAME,
    (timestamp) -> PhysicalConstants.ROBOT_TO_FRAME_CAMERA_PARTIAL);   // fixed
turretCamera = new TagCamera(turretIO, LIMELIGHT_NAME_TURRET,
    this::calculateTurretCamPose);                                     // time-varying
```

`calculateTurretCamPose` looks the turret angle up out of `RobotState`'s interpolating buffer at the frame timestamp — not the current angle. A turret spinning at speed moves the camera several degrees per frame of latency, so using the current angle puts the pose estimate in the wrong place.

### The `_PARTIAL` transforms

Pitch is applied Limelight-side (in the Megatag config), so robot code uses a variant with roll and pitch zeroed and **the full translation kept**:

```java
public static final Transform3d ROBOT_TO_FRAME_CAMERA_PARTIAL = new Transform3d(
    ROBOT_TO_FRAME_CAMERA.getTranslation(),                        // full translation, Z included
    new Rotation3d(0, 0, ROBOT_TO_FRAME_CAMERA.getRotation().getZ()));  // yaw only
```

Height is *also* configured Limelight-side, so keeping Z here double-applies it on the real robot. That is deliberate and documented in `Constants`: *"Should technically not include height on real robot but for sim it's easier to debug this way."* Copy the existing shape for a new camera. Zeroing Z to be "correct" changes real-robot pose output and is not a drive-by fix.

**Pitch sign convention is unsettled.** Both existing cameras use negative degrees; nothing states whether that means pitched down. It's currently inert — `_PARTIAL` drops pitch and `SimVisionIO` is passed `Transform3d.kZero` — so a wrong sign is documentation-only today. Don't assume; if it starts mattering, measure.

## Estimate fusion and the single-fuse rule

`TagCamera.update(boolean isTurret)` returns `Optional<TagPoseEstimate>`. The flag gates **only** the `MAX_TURRET_YAW_RATE_ROTATIONS` check — pass `true` for a turret-mounted camera, `false` for anything bolted to the frame.

`Vision.earlyPeriodic` gets at most one estimate per camera, then:

- exactly one present → use it
- both present → `combineEstimates` (inverse-variance weighted, ported from 254)
- neither → nothing

**Only ever fuse one measurement per loop into the pose estimator.** Adding both cameras separately "double taps" the Kalman filter and overweights vision against odometry. `combineEstimates` exists specifically to avoid that; don't call `addVisionMeasurement` twice.

`combineEstimates` latency-compensates the older estimate onto the newer one's timestamp using the odometry delta between them, then weights by `1/variance` per axis. Heading is only fused when both headings have real variance — `LARGE_VARIANCE` (1e7) is the sentinel for "this estimate has no usable heading", used instead of `Double.MAX_VALUE` to stay clear of overflow in the arithmetic.

## Validation and standard deviations

Every estimate passes `VisionHelpers.isValidPose` and `isValidStdevs` before fusing. Thresholds live in `VisionConstants` — ambiguity, minimum tag area, max Z error, max yaw rate, minimum distance from the field origin.

`VisionConstants.USE_AUTOMATIC_STANDARD_DEVIATIONS` selects between the Limelight's reported std devs and hand-computed ones. `IGNORE_SINGLE_TAG` drops single-tag estimates entirely.

The `highTrustEstimatesLeft` mechanism multiplies std devs by 0.1 for the first 5 estimates after coming off the bump — effectively teleporting the pose back to truth after the bump wrecks odometry. It's driven by a `Trigger` on `!onBump`.

Vision is ignored entirely in manual mode while enabled.

## Adding a camera

1. Add the name to `VisionConstants` and the mounting transform to `PhysicalConstants`, plus a `_PARTIAL` variant matching the existing shape.
2. Add a `TagCamera` field in `Vision`, constructed with the IO, name, and offset function. Pass `update(false)` for a frame-rigid camera.
3. Add the `VisionIO` parameter to the `Vision` constructor and wire **all four** construction sites in `RobotContainer` — REAL, SIM-with-vision-sim, SIM-without (the `USE_VISION_SIMULATION` else branch), and REPLAY. `SimVisionIO(name, robotToCamera, poseSupplier)` is passed `Transform3d.kZero` for the transform at every existing call site, because `updateInputs` overwrites it each loop.
4. **Update `Telemetry.checkVisionFault()`**, which enumerates `frameCamera` and `turretCamera` by hand. Skip this and the new camera's heartbeat fault silently never reaches the dashboard. `Lights` goes through `limelightsSeeTag()`, so update that too.
5. Extend the fusion logic. With three cameras the pairwise `combineEstimates` no longer covers it — fold over a fixed-order list, don't add another special case. Fixed order matters for replay determinism, since the latency compensation is applied sequentially and isn't exactly order-independent.

**Folding hazard:** `combineEstimates` computes fused heading stddev as `sqrt(1/(1/varA + 1/varB))`. Two estimates both carrying the `LARGE_VARIANCE` (1e7) "no usable heading" sentinel fuse to ≈2236, not 1e7 — the sentinel is destroyed by the arithmetic and the pose estimator receives what looks like a confident heading. This is already wrong with two cameras and gets monotonically worse with three. If you add a camera, propagate the sentinel when every input carries it. Related: `numTags` sums across cameras, so a tag seen by two double-counts (currently unread downstream, but a trap).

## Gotchas

- **Don't flush NetworkTables per camera.** There's a commented-out block explaining it: NT flush is rate-limited to once per 10ms, and flushing per-Limelight causes loop overruns. Left as a warning, not dead code to delete.
- **`LimelightHelpers.java` is vendor code**, 1677 lines, copied verbatim. Never reformat or refactor it — it must stay diffable against upstream. Its `PascalCase` static methods are its own convention, not the team's.
- **Limelights are throttled while disabled** (`LIMELIGHT_DISABLED_THROTTLE = 120` frames) to stop them overheating on the cart.
- **A Limelight is "disconnected" by heartbeat staleness**, not by NT connection — `LL_HEARTBEAT_MIN_FREQ`.
- **Vision timestamps come from the camera**, in the FPGA timebase, and get converted where needed via `WafflesUtilities.currentTimeToFPGA`. Don't substitute `Timer.getTimestamp()` for a frame timestamp — they answer different questions.

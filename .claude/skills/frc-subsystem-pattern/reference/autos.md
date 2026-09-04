# Autonomous Reference

## Contents

- Two auto systems, and which to use
- Field coordinates: `BlueRelativeTarget`
- Static autos (`SequentialCommandGroup`)
- Path following: `AutoPath`
- The adaptable system: `AdaptableBase`, `AutoSegment`, choosers
- Adding an option to an existing adaptable auto
- Writing a new adaptable variation
- Gotchas

## Two auto systems, and which to use

| System | Where | Use for |
|---|---|---|
| Static | `autos/Preload.java`, `Left.java`, `LeftDepot.java`, `LeftGreedy.java`, `Greedy.java` | A fixed routine you name and pick from the dashboard |
| Adaptable | `autos/adaptable/` | A routine assembled at runtime from dashboard-selected segments |

PathPlanner autos exist in the build but are **off** — `CodeConstants.USE_PATHPLANNER_AUTOS = false`. `AutoBuilder` is still configured in `Drive` because PathPlanner's pathfinding and logging callbacks are used. Don't add `.path` files expecting them to run.

Both register in `RobotContainer.configureCommandChoosers()`.

## Field coordinates: `BlueRelativeTarget`

**Every field coordinate in an auto is blue-alliance-relative.** `BlueRelativeTarget` flips to red on read, caching the flipped `APTarget` and invalidating when the alliance changes. Never write red coordinates and never call `FlipIfRedAlliance` yourself on an auto target.

```java
new BlueRelativeTarget(7.6, 6.64, Rotation2d.fromDegrees(-35))
    .withEntryAngle(Rotation2d.fromDegrees(-90))  // approach heading; forces autopilot over pure pursuit
    .withExitVelocity(4)                          // don't stop at this point
    .withMaxVelocity(1.75)                        // cap for this leg
    .withMaxRotationRate(3)
```

| Method | Effect | Invalidates cache |
|---|---|---|
| `withEntryAngle` | Constrains approach direction. Also forces the autopilot path instead of pure pursuit. | yes |
| `withExitVelocity` | Non-zero means drive through, not stop | yes |
| `withTarget` | Replace the pose | yes |
| `withMaxVelocity` | Per-leg speed cap | **no** |
| `withMaxRotationRate` | Per-leg turn-rate cap | **no** |
| `withMirroring(bool)` | Returns a left/right mirrored **clone** (not alliance flip) | n/a |

`withMirroring(true)` returns `getMirrored()`, a clone. `withMirroring(false)` returns `this` — the *same object*. Never mutate a target you got from `withMirroring`; the `static final` originals in `Adaptable` are shared across every generation.

**Mirroring is across Y**: `fieldSizeY - y`, with the rotation and entry angle negated. X is untouched, and it is not an alliance flip. A target already near the field's Y centreline (~4.03 m) barely moves when mirrored — so "mirror this auto to the other side" is a no-op for anything starting mid-field. Check the Y coordinate before assuming mirroring gives you a second auto.

## Static autos

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

`AutoUtils.resetOdometry(start)` first, always — there's a `BlueRelativeTarget` overload as well as the `Pose2d` one. It no-ops when `CodeConstants.RESET_ODOMETRY_AUTO_START` is false, so don't guard it yourself.

Register it in `RobotContainer.configureCommandChoosers()`: an import plus one `addOption`. Labels are spaced title case, not class names — `"Left Depot"`, `"Left Greedy"`.

```java
autoChooser.addOption("Right Preload", new RightPreload());
```

## Path following: `AutoPath`

`AutoPath` chains targets into one continuous drive, advancing when the robot is within `AUTO_POSITION_TOLERANCE_VAGUE` **or** has overshot (dot-product check in `ShouldAdvanceToNextTarget`). That overshoot test is why a path doesn't stall when it cuts a corner.

```java
new AutoPath(targets.toArray(new BlueRelativeTarget[0]))
    .withMirroring(pathMirrored)
    .withPreciseFinish()      // final target uses autopilot's atTarget, not the vague tolerance
    .follow();
```

Without `withPreciseFinish()` the path ends on the same loose tolerance used mid-path — fine when the next command re-aims, wrong when the next thing is a shot.

## The adaptable system

Four pieces:

| Piece | Role |
|---|---|
| `AdaptableBase` | Holds `cmd`, caches it, `run()` schedules it wrapped in `onlyWhile(autonomousEnabled)` |
| `AutoSegment` | A named list of `BlueRelativeTarget`s — a reusable chunk of path |
| `*Chooser` | Dashboard-backed selection, built on `LoggedDashboardChooser` |
| `AdaptableManager` | Picks the variation, regenerates while disabled, swaps Elastic tabs |

**The caching contract is the thing to get right.** `AdaptableManager.periodic()` regenerates the command while disabled so generation cost never lands during the match. Any chooser whose value affects the generated path **must** carry `.onChange(() -> InvalidateCache())`, or the dashboard change silently won't take effect.

```java
private final AutoNetworkNumber autoDelay = new AutoNetworkNumber(autoClass + "/Auto Delay", 0)
    .onChange(() -> InvalidateCache());
```

Chooser types:

```java
new AutoSegmentChooser(autoClass + "/First Sweep")     // picks an AutoSegment; getTargets() -> Optional<List<...>>
    .addOption("Normal", new NormalSweep())
    .onChange(() -> InvalidateCache());

new AutoDropdownChooser<Boolean>(autoClass + "/Side")  // picks any value; get() may return null
    .addOption("Left", false)
    .addOption("Right", true)
    .onChange(() -> InvalidateCache());

new AutoNetworkNumber(autoClass + "/Shoot Timeout", 9) // a tunable double; getAsDouble()
    .onChange(() -> InvalidateCache());
```

The first `addOption` becomes the default automatically (`GenericAutoDropdownChooser` counts options). `AutoSegmentChooser` additionally inserts a `"None"` default, so its `getTargets()` returns `Optional.empty()` when unset — hence the `ifPresent(list::addAll)` idiom.

**`AutoDropdownChooser.get()` can return `null`** before the dashboard has published. Every call site null-checks and bails or defaults:

```java
Boolean pathMirrored = autoMirroring.get();
if (pathMirrored == null) {
  return;
}
```

**That bail leaves `cmd` null, and `AdaptableBase.run()` will NPE on it:**

```java
public void run() {
  if (cmd == null) {
    GenerateAuto(true);      // may return without assigning cmd
  }
  cmd.onlyWhile(...).schedule();   // NPE
}
```

So an early return out of `GenerateAuto` is only safe on the background regeneration path (`immediate == false`), where `run()` isn't involved. If you bail, assign a fallback first — `cmd = Commands.none();` — or the robot does nothing at auto start *and* throws. This is a live hazard in `AdaptableBase`, not a hypothetical.

`getSendableChooser()` returns `SendableChooser<String>` (not `SendableChooser<V>`), so `getSelected()` yields the selected option's **key**. That's why `AutoSegmentChooser.getName()` type-checks and why `RobotContainer` and `AdaptableManager` can compare it to `"Adaptable"`. Those comparisons use `==` and work only by string-literal identity — **write `.equals()` in anything new.**

## Adding an option to an existing adaptable auto

Most common change:

1. Add a `public static class MyAttack extends AutoSegment` with an `add(...)` of targets in the constructor.
2. Add `.addOption("MyAttack", new MyAttack())` to **every** chooser that should offer it.

Step 2 is the one people get wrong. The sweep and attack option lists are duplicated by hand across the first- and second-pass choosers, and the labels have already drifted between them (`"Rotate Out"` in the first sweep chooser vs `"No Sweep"` in the second, both `new NoSweep()`). Adding to one chooser silently leaves the other without the option.

Append new options rather than prepending: `GenericAutoDropdownChooser` makes the *first* `addOption` the default, so inserting at the top silently changes which auto runs when nobody touches the dashboard.

```java
public static class NormalAttack extends AutoSegment {
  public NormalAttack() {
    add(
        new BlueRelativeTarget(7.6, 6.64, Rotation2d.fromDegrees(-35)),
        new BlueRelativeTarget(7.6, 4.5, Rotation2d.fromDegrees(-90))
            .withMaxVelocity(PICKUP_VELOCITY));
  }
}
```

Segments are nested `public static class` inside the auto that uses them, grouped under `// Attacks` / `// Sweeps` comments. Sweeps are written to string on after an attack, so their first target must be reachable from wherever the attack ends.

## Writing a new adaptable variation

```java
public class MyAuto extends AdaptableBase {
  public MyAuto() {
    super("MyAuto");            // becomes the NT/Elastic tab name via autoClass
  }

  @Override
  protected void GenerateAuto(boolean immediate) {
    // 1. read choosers, null-check
    // 2. assemble ArrayList<BlueRelativeTarget>
    // 3. build AutoPath(s)
    // 4. assign to `cmd`
    // 5. if (!immediate) AutoVisualizer.VisualizeAuto(start, fullPath);
    // 6. SmartDashboard.putBoolean(autoClass + "/Cached", true);
  }
}
```

Then register it in `AdaptableManager.adaptableVariation`.

The `immediate` flag is false when regenerating in the background while disabled (visualize then) and true when generating on demand at auto start (skip visualization — it costs loop time you don't have).

`GenerateAuto` must assign `cmd`. `AdaptableBase.run()` calls `GenerateAuto(true)` when `cmd == null`, then immediately `InvalidateCache()`s so the next enable regenerates against the current alliance and pose.

## If you forget a step

Almost every mistake here fails silently rather than at compile time. What each omission actually looks like:

| Forgot | Symptom |
|---|---|
| `.onChange(() -> InvalidateCache())` on a chooser | Dashboard change appears to do nothing — the cached command is reused |
| To assign `cmd` in `GenerateAuto` | NPE in `AdaptableBase.run()` at auto start |
| To register a variation in `AdaptableManager.adaptableVariation` | Silently unreachable; no error |
| To register a static auto in `configureCommandChoosers()` | Never appears in the dashboard list |
| `AutoUtils.resetOdometry(start)` | Odometry keeps whatever pose it had; the whole path runs offset. `farFromStart` only warns on the adaptable path. |
| `.withPreciseFinish()` | Path ends on the loose ~25 cm tolerance — fine if the next command re-aims, wrong if the next thing is a shot |
| To add an option to the *second* chooser | Option available in one pass only |
| To append rather than prepend an option | The new option becomes the default |

Regeneration only runs when **all three** hold: robot disabled, DS attached, and the "Adaptable" option selected in the main auto chooser. If a dashboard change seems inert, check those before anything else.

## Where a delay actually takes effect

`Adaptable.GenerateAuto` composes with `Commands.deadline(main, ...)`, where everything finishes when `main` does. A `waitSeconds` placed inside a deadline group whose `main` is a shoot command is simply swallowed — the shot ends the group regardless. Before inserting a wait, find which branch is the deadline's `main` and put the wait outside it, or it will do nothing and look like a tuning problem.

`AutoNetworkNumber` needs no registration. It extends AdvantageKit's `LoggedNetworkNumber`, whose own registry calls `periodic()`. Declare it as a field and read it with `getAsDouble()`.

## Gotchas

- **Alliance flip happens at read time, not build time.** Building the auto while disabled and driving it after an alliance change is fine — `BlueRelativeTarget` re-flips. But anything you cache yourself must not cache a flipped pose.
- **`getName()` string comparison.** `Adaptable` does `secondAttackDepthChooser.getName() == "BLOCKER"` — reference equality. It happens to work because `SendableChooser` hands back the same literal instance that was passed to `addOption` in the same file. `AdaptableManager.periodic()` and `RobotContainer`'s `onChange` do the same thing. Write `.equals()` in anything new.
- **The `farFromStart` alert** compares current pose to the generated start point and warns the driver. Set it in any new variation; it catches "wrong side selected" before the match instead of after.
- **Regeneration only runs while disabled and DS-attached and the "Adaptable" chooser option is selected.** If a dashboard change appears to do nothing, check all three.

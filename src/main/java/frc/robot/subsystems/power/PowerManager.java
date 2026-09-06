// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.power;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShotPlanner;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

/**
 * Hands the battery's supply current to whichever mechanism needs it most for
 * what the robot is doing right now.
 *
 * <p>
 * Applying a Phoenix configuration is a <b>blocking CAN write</b> that can take
 * tens of milliseconds and can fail, so this runs off the main loop. Three
 * things follow from that, and they are the whole reason this class is more than
 * a for-loop:
 *
 * <ul>
 * <li><b>Writes are coalesced.</b> Only the most recently requested state is
 * ever applied. Flipping between states faster than CAN can keep up leaves the
 * robot at the latest state, not at whatever was queued three transitions ago.
 * <li><b>Decreases are applied before increases.</b> A transition is not atomic,
 * so it can be observed half-finished. Lowering first means every intermediate
 * state draws <i>less</i> than either the old or the new budget — a partial
 * application can never exceed what was asked for.
 * <li><b>Unchanged limits are not written.</b> A mechanism whose limit is the
 * same in both states is skipped entirely, which removes most of the CAN traffic
 * and most of the chances to fail.
 * </ul>
 *
 * <p>
 * Failures are not silent: each write goes through {@code PhoenixHelpers.tryConfig},
 * which retries and raises the CAN config error flag, and this class reports the
 * state actually in force alongside the one requested. If they disagree for more
 * than {@link #DIVERGENCE_ALERT_TIME} an alert is raised, because a drivetrain
 * still at its shooting limit after the shot is over is a real performance loss
 * that would otherwise go unnoticed.
 */
public class PowerManager extends VirtualSubsystem {
  /** How long requested and applied may disagree before the driver is told. */
  private static final double DIVERGENCE_ALERT_TIME = 1.0;
  /** How often to retry a budget that did not fully apply. */
  private static final double RETRY_PERIOD = 0.5;
  /**
   * Stick deflection past which the driver is taken to be going somewhere rather
   * than nudging into place, and the drivetrain is given back in full. Set high
   * enough that lining up a shot does not trip it.
   */
  private static final double ESCAPE_DEMAND = 0.7;
  /**
   * How long the shooting budget is held after the robot last fired.
   *
   * <p>
   * The budget is keyed to firing rather than to being ready to fire, so the
   * drivetrain is never weak while merely driving around spun up. That works
   * because the cap is not there to prevent the dip — the dip is the ball taking
   * energy out of the wheel, and nothing electrical stops that. It is there to
   * hold the bus up during the recovery afterwards, which the logs put at 0.06 s
   * for a normal dip and up to 0.6 s for a deep one. A configuration write lands
   * well inside that.
   *
   * <p>
   * The hold also stops the budget thrashing between states during a burst,
   * which would put a pair of blocking CAN writes on the bus for every ball.
   */
  private static final double SHOT_HOLD_TIME = 1.0;

  private final ExecutorService executor = Executors.newSingleThreadExecutor(runnable -> {
    Thread thread = new Thread(runnable, "PowerManager");
    thread.setDaemon(true);
    return thread;
  });

  private final AtomicReference<PowerManagerState> requested = new AtomicReference<>(PowerManagerState.DEFAULT);
  private volatile PowerManagerState applied = null;
  private double lastAttempt = Double.NEGATIVE_INFINITY;

  private final Alert divergenceAlert = new Alert("Power limits did not apply; check CAN", AlertType.kWarning);
  private double divergentSince = Double.NaN;
  private double shotHeldUntil = Double.NEGATIVE_INFINITY;
  private volatile boolean turboOverride = false;

  /**
   * One mechanism's share of a budget. {@code supplyLimit} is only used to order
   * the writes; {@code apply} carries whatever set of limits that mechanism
   * actually needs, which for the flywheel is two numbers rather than one.
   */
  private record Share(
      String name,
      double supplyLimit,
      BooleanSupplier apply
  ) {}

  @Override
  public void periodic() {
    PowerManagerState state = chooseState();
    boolean changed = requested.getAndSet(state) != state;
    // Retry as well as react: a write that failed leaves a mechanism at the
    // previous state's limit, and without this it would stay there silently
    // until the next transition happened to succeed.
    boolean stale = applied != state && Timer.getTimestamp() - lastAttempt > RETRY_PERIOD;
    if (changed || stale) {
      lastAttempt = Timer.getTimestamp();
      executor.execute(this::applyLatest);
    }

    PowerManagerState inForce = applied;
    Logger.recordOutput("Power/Requested State", state.toString());
    Logger.recordOutput("Power/Applied State", inForce == null ? "NONE" : inForce.toString());
    Logger.recordOutput("Power/Draw Ceiling (A)", state.drawCeiling());
    Logger.recordOutput("Power/Driver Override",
        turboOverride || !RobotContainer.state.joysticksFree(ESCAPE_DEMAND));
    Logger.recordOutput("Power/Turbo Override", turboOverride);

    if (inForce == state) {
      divergentSince = Double.NaN;
    } else if (Double.isNaN(divergentSince)) {
      divergentSince = Timer.getTimestamp();
    }
    divergenceAlert.set(
        !Double.isNaN(divergentSince) && Timer.getTimestamp() - divergentSince > DIVERGENCE_ALERT_TIME);
  }

  /**
   * Forces the full drivetrain budget while held, for a driver who needs to move
   * more than they need the next shot.
   *
   * <p>
   * This goes further than the stick threshold, which only returns the drivetrain
   * to its ordinary budget. Turbo also starves everything that is not the
   * drivetrain, which is the part that matters: the drivetrain's own limit is not
   * what holds it back when it is pinned, the bus voltage is.
   *
   * <p>
   * Nothing binds this yet; it wants a spare control, and binding it is one line:
   *
   * <pre>
   * someButton.whileTrue(Commands.startEnd(
   *     () -&gt; RobotContainer.powerManager.setTurboOverride(true),
   *     () -&gt; RobotContainer.powerManager.setTurboOverride(false)));
   * </pre>
   */
  public void setTurboOverride(boolean turbo) {
    turboOverride = turbo;
  }

  /**
   * Picks the budget for what the robot is doing.
   *
   * <p>
   * The trigger is firing, held for {@link #SHOT_HOLD_TIME} afterwards. Two
   * earlier triggers were measured and rejected. A flywheel goal is set a median
   * of 7 s before the shot, and the wheel is nearly at speed a median of 1.1 s
   * before it — either would leave the drivetrain weak while driving around the
   * shooting zone spun up, which is exactly when a defended robot needs it.
   *
   * <p>
   * Firing gives no warning at all: {@code RobotState/Shooting} goes true in the
   * same loop the ball leaves. That is fine, because the cap is not trying to
   * prevent the dip. It is holding the bus up for the recovery afterwards, and
   * the wheel does not start dropping until a median of 0.2 s after the command
   * in any case.
   */
  private PowerManagerState chooseState() {
    // The driver always wins. A robot pinned in its own zone with a spun-up
    // flywheel must not be the robot that cannot drive out, so a hard shove on
    // the sticks hands the drivetrain straight back. A shot taken while moving
    // gently still gets the budget; one taken while fighting to escape does not,
    // which is the right way round.
    if (turboOverride) {
      return PowerManagerState.TURBO;
    }
    if (!RobotContainer.state.joysticksFree(ESCAPE_DEMAND)) {
      return PowerManagerState.DEFAULT;
    }
    if (RobotContainer.state.isShooting()) {
      shotHeldUntil = Timer.getTimestamp() + SHOT_HOLD_TIME;
    }
    if (Timer.getTimestamp() > shotHeldUntil) {
      return PowerManagerState.DEFAULT;
    }
    // Intaking outranks distance: being dragged down in a pile is the one case
    // where the intake genuinely needs its full allowance.
    if (RobotContainer.state.isIntaking()) {
      return PowerManagerState.SHOOTING_AND_INTAKING;
    }
    if (ShotPlanner.distanceToTarget() > PowerManagerState.FAR_SHOT_DISTANCE) {
      return PowerManagerState.SHOOTING_FAR;
    }
    return PowerManagerState.SHOOTING;
  }

  /**
   * Applies whatever state was most recently requested, skipping any that were
   * superseded while this was waiting on CAN. Runs on the executor thread.
   */
  private void applyLatest() {
    try {
      applyBudget();
    } catch (RuntimeException e) {
      // An exception here would kill the executor thread and silently end all
      // future budget changes. Report it and stay alive.
      DriverStation.reportError("PowerManager failed to apply limits: " + e.getMessage(), false);
    }
  }

  private void applyBudget() {
    PowerManagerState target = requested.get();
    PowerManagerState previous = applied;
    if (target == previous) {
      return;
    }

    List<Share> shares = new ArrayList<>(List.of(
        new Share("Drive", target.driveSupplyCurrent,
            () -> RobotContainer.drive.applyCurrentLimits(target.driveSupplyCurrent)),
        new Share("Flywheel", target.flywheelSupplyCurrent,
            () -> RobotContainer.flywheel.applyCurrentLimits(
                target.flywheelStatorCurrent, target.flywheelSupplyCurrent)),
        new Share("Intake", target.intakeSupplyCurrent,
            () -> RobotContainer.intake.applyCurrentLimits(target.intakeSupplyCurrent)),
        new Share("Feeder", target.feederSupplyCurrent,
            () -> RobotContainer.indexer.applyCurrentLimits(target.feederSupplyCurrent)),
        new Share("Spindexer", target.spindexerSupplyCurrent,
            () -> RobotContainer.indexer.applySpindexerCurrentLimits(
                target.spindexerSupplyCurrent))));

    // Write the reductions first so a half-applied transition is under budget
    // rather than over it.
    if (previous != null) {
      shares.removeIf(share -> share.supplyLimit() == limitFor(previous, share.name()));
      shares.sort(Comparator.comparingDouble(
          share -> share.supplyLimit() < limitFor(previous, share.name()) ? 0 : 1));
    }

    boolean allApplied = true;
    for (Share share : shares) {
      allApplied &= share.apply().getAsBoolean();
    }
    // Only claim the state is in force if every write actually landed, so the
    // retry above keeps trying and the alert stays up until it is true.
    if (allApplied) {
      applied = target;
    }
  }

  private static double limitFor(PowerManagerState state, String name) {
    return switch (name) {
      case "Drive" -> state.driveSupplyCurrent;
      case "Flywheel" -> state.flywheelSupplyCurrent;
      case "Intake" -> state.intakeSupplyCurrent;
      case "Feeder" -> state.feederSupplyCurrent;
      case "Spindexer" -> state.spindexerSupplyCurrent;
      default -> throw new IllegalArgumentException("no budget column for " + name);
    };
  }
}

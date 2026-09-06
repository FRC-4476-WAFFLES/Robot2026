// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.power;

/**
 * A complete supply-current budget, in amps per motor, for one thing the robot
 * is doing. Every managed mechanism gets a value in every state, so reading
 * across a row shows the whole trade being made.
 *
 * <p>
 * The numbers come from measured match logs: the drivetrain is about half of a
 * current spike and the flywheel about a sixth, and the flywheel's inability to
 * recover its speed after each ball is what makes shots fall short as the
 * battery sags. While shooting we do not need full drivetrain acceleration, so
 * that capacity goes to the flywheel instead.
 *
 * <p>
 * <b>The intake is never limited below what it uses.</b> Its supply limit was
 * removed on purpose because a 35 A limit made it bog down, and the logs confirm
 * it draws 54 - 64 A precisely when loaded. Its allowance is only tightened in
 * the one state where it is measured drawing 2.2 A - shooting without intaking -
 * so the tightening is a guard against a rogue draw rather than a saving.
 */
public enum PowerManagerState {
  /** Everything at the limits the subsystems configure for themselves. */
  DEFAULT(45, 120, 60, 90, 25, 90),
  /** Flywheel priority: it needs headroom to recover between balls. */
  SHOOTING(20, 160, 140, 20, 25, 90),
  /**
   * A long shot, where accuracy collapses in the logs. The flywheel allowance is
   * the same as {@link #SHOOTING} because it cannot use any more: a long shot
   * needs a higher wheel speed, which means more back-EMF and so less voltage
   * left to drive recovery current. What actually helps is the bus voltage
   * itself, so the drivetrain is cut harder here than anywhere else. Long shots
   * in the logs were taken on a median 7.5 V bus, against 12 V up close.
   *
   * <p>
   * The ball path is capped here too, and only here. A 20 A ceiling sits at what
   * the feeder averages anyway, so it does not slow a normal feed — it clips the
   * peaks, which reach 84 A on the feeder and 64 A on the spindexer, and peaks
   * are what sag the bus. Replaying the logs, that is worth about three more
   * landed shots in 184 on top of the drivetrain cap. Cutting harder than this
   * buys three more and guts the feed for it, which is a bad trade even on a
   * long shot: a slow feed still scores, an inconsistent one does not.
   */
  SHOOTING_FAR(10, 160, 140, 20, 20, 20),
  /**
   * Shooting while intaking. The intake keeps its full allowance because being
   * dragged down in a pile is exactly when it needs torque, so the flywheel's
   * headroom is paid for out of the drivetrain alone.
   */
  SHOOTING_AND_INTAKING(15, 160, 140, 50, 25, 90),
  /**
   * The driver needs to move more than they need the next shot — pinned, being
   * defended, or getting out of somewhere.
   *
   * <p>
   * The drivetrain keeps its ordinary allowance rather than being given a larger
   * one, because its limit is not what stops it: in the logs the drivetrain was
   * pulling 145 - 211 A at the worst point of every brownout, at or past what its
   * limit already permits. What stops it is the bus collapsing underneath it. So
   * everything that is not the drivetrain is starved instead, which frees the
   * battery to deliver what the drivetrain is already allowed to draw.
   */
  TURBO(45, 120, 25, 20, 25, 20);

  /** Distance beyond which accuracy fell off a cliff in the logs, in metres. */
  public static final double FAR_SHOT_DISTANCE = 3.5;

  /** Per drive motor, supply. Four of them. */
  public final double driveSupplyCurrent;
  /**
   * Per flywheel motor, stator. This is the one that governs recovery: measured
   * stator peaks sit at the 120 A configured limit while supply peaks run to
   * 96 A against a 60 A limit, so the stator ceiling is what actually stops the
   * wheel accelerating back to speed.
   */
  public final double flywheelStatorCurrent;
  /** Per flywheel motor, supply. Two of them. */
  public final double flywheelSupplyCurrent;
  /**
   * Per intake motor, supply.
   *
   * <p>
   * {@link #DEFAULT} sits above the measured 84 A peak so it changes nothing: a
   * 35 A limit here used to make the intake bog down and was removed on purpose.
   * That headroom exists for driving hard into a pile, which is an autonomous
   * and teleop-intaking problem, not a shooting one.
   *
   * <p>
   * Shooting while intaking is a slower business — the robot is lining up a shot,
   * not charging a pile — so that state gets a middle number: above the 13.6 A it
   * is measured drawing there, below the allowance reserved for hitting a pile at
   * speed. While shooting without intaking it is measured drawing 2.2 A, so the
   * tightest cap costs nothing and is only a guard.
   */
  public final double intakeSupplyCurrent;

  /**
   * Per feeder motor, supply. Two of them. Held at 25 A everywhere, which is
   * what the subsystem configures for itself.
   *
   * <p>
   * That limit was added deliberately after the 2026 ONWEL event to reduce
   * battery sag, and the drive team reports no loss of shot quality from it. The
   * feeder is in the shot path, so it is never cut <i>below</i> that — a ball
   * entering the shooter at an inconsistent speed makes the shot inconsistent
   * however well the flywheel is holding.
   *
   * <p>
   * <b>Worth re-measuring once a log exists from after that change.</b> Every
   * feeder number here comes from ONWEL logs, which predate the limit: the
   * feeder drew 27 - 35 A at high duty because nothing stopped it. Run
   * {@code logReview --args="motor Indexer/Feeder <log>"} on a recent log — if
   * feeder speed still sits flat at 36 - 37 rps across every duty bucket, the
   * limit costs nothing and should stay. If speed falls off at high duty, it is
   * throttling the feeder under load and this column is where to raise it.
   */
  public final double feederSupplyCurrent;

  /**
   * Per spindexer motor, supply. Two of them.
   *
   * <p>
   * <b>Set above what it uses in every state, so nothing changes.</b> The
   * spindexer governs shot rate, and it has never had a supply limit. Capping it
   * during a long shot would free about 27 A on average, which is not worth
   * risking the rate on. The column exists so the number can be tuned with a
   * measurement behind it; 90 A is above its measured 64 A peak, so today it is
   * a no-op. Its stator limit stays at 150 A either way.
   */
  public final double spindexerSupplyCurrent;

  PowerManagerState(double driveSupplyCurrent, double flywheelStatorCurrent,
      double flywheelSupplyCurrent, double intakeSupplyCurrent, double feederSupplyCurrent,
      double spindexerSupplyCurrent) {
    this.driveSupplyCurrent = driveSupplyCurrent;
    this.flywheelStatorCurrent = flywheelStatorCurrent;
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
    this.feederSupplyCurrent = feederSupplyCurrent;
    this.spindexerSupplyCurrent = spindexerSupplyCurrent;
  }

  /**
   * Supply current this state permits across the mechanisms that actually draw
   * meaningful current while shooting.
   *
   * <p>
   * The intake is excluded on purpose. Its allowance is set above what it uses
   * so that it is never clamped, which means counting it would show a large
   * saving whenever that allowance is lowered even though the intake was drawing
   * 2.2 A either way. Including it hides whether the real draw went up or down.
   *
   * <p>
   * Note this is a ceiling, not a prediction. The drivetrain cap is a guarantee:
   * it can never exceed its number. The flywheel's allowance is an option it
   * only spends while recovering from a ball, which is a fraction of a second at
   * a time. Average draw while shooting falls; the instantaneous peak during a
   * recovery burst is deliberately allowed to rise, because that burst is what
   * puts the shot on target.
   */
  public double drawCeiling() {
    return 4 * driveSupplyCurrent + 2 * flywheelSupplyCurrent + 2 * feederSupplyCurrent
        + 2 * spindexerSupplyCurrent;
  }
}

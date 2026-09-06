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
  DEFAULT(45, 120, 60, 90, 25),
  /** Flywheel priority: it needs headroom to recover between balls. */
  SHOOTING(20, 160, 140, 20, 50),
  /**
   * A long shot, where accuracy collapses in the logs. The flywheel allowance is
   * the same as {@link #SHOOTING} because it cannot use any more: a long shot
   * needs a higher wheel speed, which means more back-EMF and so less voltage
   * left to drive recovery current. What actually helps is the bus voltage
   * itself, so the drivetrain is cut harder here than anywhere else. Long shots
   * in the logs were taken on a median 7.5 V bus, against 12 V up close.
   */
  SHOOTING_FAR(10, 160, 140, 20, 50),
  /**
   * Shooting while intaking. The intake keeps its full allowance because being
   * dragged down in a pile is exactly when it needs torque, so the flywheel's
   * headroom is paid for out of the drivetrain alone.
   */
  SHOOTING_AND_INTAKING(20, 160, 140, 90, 50);

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
   * Per intake motor, supply. The default is set above the measured 84 A peak so
   * it changes nothing: a 35 A limit here used to make the intake bog down and
   * was removed on purpose. It is only tightened while shooting without
   * intaking, where the intake is measured drawing 2.2 A anyway.
   */
  public final double intakeSupplyCurrent;

  /**
   * Per feeder motor, supply. Two of them. The feeder is only ever given more,
   * never less: it holds 36 - 37 rps in the logs regardless of load, and a ball
   * entering the shooter at an inconsistent speed makes the shot inconsistent no
   * matter how well the flywheel is holding. Its configured 25 A limit is below
   * the 27 - 35 A it draws at high duty, so today the limiter is cutting it back
   * under exactly the load where consistency matters most.
   */
  public final double feederSupplyCurrent;

  PowerManagerState(double driveSupplyCurrent, double flywheelStatorCurrent,
      double flywheelSupplyCurrent, double intakeSupplyCurrent, double feederSupplyCurrent) {
    this.driveSupplyCurrent = driveSupplyCurrent;
    this.flywheelStatorCurrent = flywheelStatorCurrent;
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
    this.feederSupplyCurrent = feederSupplyCurrent;
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
    return 4 * driveSupplyCurrent + 2 * flywheelSupplyCurrent + 2 * feederSupplyCurrent;
  }
}

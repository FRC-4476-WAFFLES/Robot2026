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
  DEFAULT(45, 120, 60, 90),
  /** Flywheel priority: it needs headroom to recover between balls. */
  SHOOTING(20, 160, 120, 20),
  /**
   * Shooting while intaking. The intake keeps its full allowance because being
   * dragged down in a pile is exactly when it needs torque, so the flywheel's
   * headroom is paid for out of the drivetrain alone.
   */
  SHOOTING_AND_INTAKING(20, 160, 110, 90);

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

  PowerManagerState(double driveSupplyCurrent, double flywheelStatorCurrent,
      double flywheelSupplyCurrent, double intakeSupplyCurrent) {
    this.driveSupplyCurrent = driveSupplyCurrent;
    this.flywheelStatorCurrent = flywheelStatorCurrent;
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
  }

  /** Total supply current this state permits across every managed motor. */
  public double totalBudget() {
    return 4 * driveSupplyCurrent + 2 * flywheelSupplyCurrent + 2 * intakeSupplyCurrent;
  }
}

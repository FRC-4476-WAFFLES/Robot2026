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
 * <b>The intake is deliberately not managed yet.</b> Its supply limit was
 * removed on purpose because a 35 A limit made it bog down while intaking, and
 * the logs confirm it draws 54 – 64 A precisely when loaded. Adding it back
 * without measuring a safe number would recreate a problem the team already
 * solved. The enum has room for it when that number exists.
 */
public enum PowerManagerState {
  /** Everything at the limits the subsystems configure for themselves. */
  DEFAULT(45, 60),
  /** Flywheel priority: it needs headroom to recover between balls. */
  SHOOTING(20, 90);

  /** Per drive motor. Four of them, so the total is four times this. */
  public final double driveSupplyCurrent;
  /** Per flywheel motor. Two of them. */
  public final double flywheelSupplyCurrent;

  PowerManagerState(double driveSupplyCurrent, double flywheelSupplyCurrent) {
    this.driveSupplyCurrent = driveSupplyCurrent;
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
  }

  /** Total supply current this state permits across every managed motor. */
  public double totalBudget() {
    return 4 * driveSupplyCurrent + 2 * flywheelSupplyCurrent;
  }
}

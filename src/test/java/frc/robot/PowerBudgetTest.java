// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.power.PowerManagerState;

/**
 * Guards the power budgets against being edited into something that draws more
 * than the battery can give.
 *
 * <p>
 * The measured numbers this defends: matches browned out whenever peak draw
 * passed roughly 250 A, and the cleanest match peaked at 250 A with no brownouts
 * at all. These budgets cover the drivetrain and flywheel only, which together
 * are about two thirds of a spike, so their ceiling is set below that.
 */
public class PowerBudgetTest {
  @Test
  void noStatePermitsMoreTotalDrawThanTheRobotAlreadyDid() {
    // DEFAULT reproduces the limits each subsystem sets for itself, so it is the
    // behaviour the match logs were recorded under. The manager is meant to move
    // current between mechanisms, never to hand out more of it.
    double baseline = PowerManagerState.DEFAULT.totalBudget();
    for (PowerManagerState state : PowerManagerState.values()) {
      System.out.printf("%-22s drive %3.0fA x4  flywheel %3.0f stator %3.0f supply x2  "
          + "intake %3.0fA x2  total %4.0fA%n",
          state, state.driveSupplyCurrent, state.flywheelStatorCurrent,
          state.flywheelSupplyCurrent, state.intakeSupplyCurrent, state.totalBudget());
      assertTrue(state.driveSupplyCurrent > 0 && state.flywheelSupplyCurrent > 0
          && state.intakeSupplyCurrent > 0,
          state + " has a non-positive limit, which would stop the mechanism entirely");
      assertTrue(state.totalBudget() <= baseline,
          state + " permits " + state.totalBudget() + "A against a " + baseline + "A baseline");
    }
  }

  @Test
  void theIntakeIsNeverLimitedBelowWhatItDrawsWhileIntaking() {
    // A 35A supply limit made the intake bog down and was removed on purpose.
    // Logs show it draws 54-64A when loaded and peaks at 84A, so any state where
    // intaking happens must leave it clear of that.
    assertTrue(PowerManagerState.DEFAULT.intakeSupplyCurrent > 84,
        "DEFAULT must not clamp the intake below its measured peak");
    assertTrue(PowerManagerState.SHOOTING_AND_INTAKING.intakeSupplyCurrent > 84,
        "SHOOTING_AND_INTAKING must not clamp the intake -- a pile is when it needs torque");
  }

  @Test
  void shootingGivesTheFlywheelMoreThanTheDefaultDoes() {
    // The entire point of the manager: a flywheel that cannot recover between
    // balls is what makes shots fall short as the battery sags.
    assertTrue(PowerManagerState.SHOOTING.flywheelSupplyCurrent > PowerManagerState.DEFAULT.flywheelSupplyCurrent,
        "SHOOTING must give the flywheel more headroom than DEFAULT");
    assertTrue(PowerManagerState.SHOOTING.totalBudget() <= PowerManagerState.DEFAULT.totalBudget(),
        "SHOOTING must pay for the flywheel out of the drivetrain, not out of the battery");
  }
}

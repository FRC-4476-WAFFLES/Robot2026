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
  /** What the managed motors may draw together, in amps. */
  private static final double MANAGED_BUDGET_CEILING = 400;

  @Test
  void everyStateStaysWithinTheBudgetCeiling() {
    for (PowerManagerState state : PowerManagerState.values()) {
      System.out.printf("%-10s drive %.0fA x4, flywheel %.0fA x2, total %.0fA%n",
          state, state.driveSupplyCurrent, state.flywheelSupplyCurrent, state.totalBudget());
      assertTrue(state.driveSupplyCurrent > 0 && state.flywheelSupplyCurrent > 0,
          state + " has a non-positive limit, which would stop the mechanism entirely");
      assertTrue(state.totalBudget() <= MANAGED_BUDGET_CEILING,
          state + " permits " + state.totalBudget() + "A, above the " + MANAGED_BUDGET_CEILING
              + "A ceiling that keeps peak draw near where matches stopped browning out");
    }
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

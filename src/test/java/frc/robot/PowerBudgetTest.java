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
  /**
   * A loose absolute ceiling. Matches in the logs browned out once peak draw
   * passed roughly 250 A and the cleanest match peaked at 250 A with none, so
   * this is not a target — it is a guard against someone typing an extra digit.
   */
  private static final double ABSOLUTE_CEILING = 700;

  @Test
  void everyStateStaysUnderTheAbsoluteCeiling() {
    for (PowerManagerState state : PowerManagerState.values()) {
      System.out.printf("%-22s drive %3.0fA x4  flywheel %3.0f stator %3.0f supply x2  "
          + "intake %3.0f  feeder %3.0f  spindexer %3.0f  ceiling %4.0fA%n",
          state, state.driveSupplyCurrent, state.flywheelStatorCurrent,
          state.flywheelSupplyCurrent, state.intakeSupplyCurrent, state.feederSupplyCurrent,
          state.spindexerSupplyCurrent, state.drawCeiling());
      assertTrue(state.driveSupplyCurrent > 0 && state.flywheelSupplyCurrent > 0
          && state.intakeSupplyCurrent > 0,
          state + " has a non-positive limit, which would stop the mechanism entirely");
      assertTrue(state.drawCeiling() <= ABSOLUTE_CEILING,
          state + " permits " + state.drawCeiling() + "A, above the " + ABSOLUTE_CEILING
              + "A guard");
    }
  }

  @Test
  void shootingPaysForTheFlywheelOutOfTheDrivetrain() {
    // This is the trade the manager exists to make, and it is asymmetric on
    // purpose. The drivetrain cap is a guarantee it can never exceed. The
    // flywheel's allowance is an option it spends only while recovering from a
    // ball. Requiring the ceilings to net out below the default would force the
    // flywheel back down and defeat the point.
    for (PowerManagerState state : new PowerManagerState[] { PowerManagerState.SHOOTING,
        PowerManagerState.SHOOTING_FAR, PowerManagerState.SHOOTING_AND_INTAKING }) {
      assertTrue(state.driveSupplyCurrent < PowerManagerState.DEFAULT.driveSupplyCurrent,
          state + " must cap the drivetrain, which is where the current comes from");
      assertTrue(state.flywheelSupplyCurrent > PowerManagerState.DEFAULT.flywheelSupplyCurrent
          && state.flywheelStatorCurrent > PowerManagerState.DEFAULT.flywheelStatorCurrent,
          state + " must give the flywheel more of both limits, or it cannot recover faster");
    }
  }

  @Test
  void aLongShotCutsTheDrivetrainHardestOfAll() {
    // A long shot needs a higher wheel speed, so there is less voltage left to
    // drive recovery current. More flywheel allowance cannot be used; a higher
    // bus voltage can, and that comes off the drivetrain.
    assertTrue(PowerManagerState.SHOOTING_FAR.driveSupplyCurrent < PowerManagerState.SHOOTING.driveSupplyCurrent,
        "SHOOTING_FAR must cut the drivetrain harder than SHOOTING, which is where its benefit comes from");
  }

  @Test
  void theFeederIsNeverCutWhileShooting() {
    // The feeder holds 36-37 rps in the logs no matter the load, and a ball
    // entering at an inconsistent speed makes the shot inconsistent however well
    // the flywheel is holding. Slowing the feed would buy the flywheel recovery
    // time at the cost of the thing it is recovering for.
    for (PowerManagerState state : new PowerManagerState[] { PowerManagerState.SHOOTING,
        PowerManagerState.SHOOTING_FAR, PowerManagerState.SHOOTING_AND_INTAKING }) {
      assertTrue(state.feederSupplyCurrent >= PowerManagerState.DEFAULT.feederSupplyCurrent,
          state + " must not cut the feeder below its default allowance");
      assertTrue(state.feederSupplyCurrent > 35,
          state + " must leave the feeder clear of the 27-35A it draws at high duty");
    }
  }

  @Test
  void theSpindexerIsNeverCapped() {
    // The spindexer governs shot rate and has never had a supply limit. Capping
    // it during a long shot would free about 27A on average, which is not worth
    // risking the rate on. Every state must stay above its measured 64A peak.
    for (PowerManagerState state : PowerManagerState.values()) {
      assertTrue(state.spindexerSupplyCurrent > 64,
          state + " caps the spindexer below its measured peak, which would cost shot rate");
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

}

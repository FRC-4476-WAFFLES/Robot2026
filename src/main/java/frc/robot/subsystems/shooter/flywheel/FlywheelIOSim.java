// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private SecondOrderSim simState;
  private double setpointVel = 0;

  public FlywheelIOSim() {
    simState = new SecondOrderSim(2.5, 1, 0, 0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    var talonFXSim = flywheel0.getSimState();

    var simResult = simState.Evaluate(setpointVel, CodeConstants.PERIODIC_LOOP_TIME);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    // talonFXSim.setRawRotorPosition(
    //     (flywheel0.getSignalData().position() + (simResult.get(0) * CodeConstants.PERIODIC_LOOP_TIME))
    //         * PhysicalConstants.HOOD_REDUCTION);
    talonFXSim.setRotorVelocity(simResult.get(0));

    super.updateInputs(inputs);
  }

  @Override
  public void runFlywheelVelocity(double velocity) {
    setpointVel = velocity;
    super.runFlywheelVelocity(velocity);
  }
}
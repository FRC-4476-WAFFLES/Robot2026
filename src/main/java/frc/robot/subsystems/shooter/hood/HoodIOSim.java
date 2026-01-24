// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class HoodIOSim extends HoodIOTalonFX {
  private SecondOrderSim simState;
  private double setpointPos;

  public HoodIOSim() {
    simState = new SecondOrderSim(2.5, 1, 0, 0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var talonFXSim = hood.getSimState();

    var simResult = simState.Evaluate(setpointPos, CodeConstants.PERIODIC_LOOP_TIME);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(simResult.get(0) * PhysicalConstants.HOOD_REDUCTION);
    talonFXSim.setRotorVelocity(simResult.get(1) * PhysicalConstants.HOOD_REDUCTION);

    super.updateInputs(inputs);
  }

  @Override
  public void runHoodPosition(double position) {
    setpointPos = position;
    super.runHoodPosition(position);
  }
}
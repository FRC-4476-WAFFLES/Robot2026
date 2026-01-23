// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class ClimberIOSim extends ClimberIOTalonFX {
  private SecondOrderSim climberSim;
  private double setpointPos;

  public ClimberIOSim() {
    climberSim = new SecondOrderSim(2.5, 1, 0, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var talonFXSim = climber.getSimState();

    var simResult = climberSim.Evaluate(setpointPos, CodeConstants.PERIODIC_LOOP_TIME);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(simResult.get(0) * PhysicalConstants.CLIMBER_REDUCTION);
    talonFXSim.setRotorVelocity(simResult.get(1) * PhysicalConstants.CLIMBER_REDUCTION);

    super.updateInputs(inputs);
  }

  @Override
  public void runClimberPosition(double position) {
    setpointPos = position;
    super.runClimberPosition(position);
  }
}
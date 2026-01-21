// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class IntakeIOSim extends IntakeIOTalonFX {
  private SecondOrderSim expanderSim;
  private double setpointPos;

  public IntakeIOSim() {
    expanderSim = new SecondOrderSim(2.5, 1, 0, 0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var talonFXSim = expander.getSimState();

    var simResult = expanderSim.Evaluate(setpointPos, CodeConstants.PERIODIC_LOOP_TIME);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(simResult.get(0) * PhysicalConstants.EXTENDER_REDUCTION);
    talonFXSim.setRotorVelocity(simResult.get(1) * PhysicalConstants.EXTENDER_REDUCTION);

    super.updateInputs(inputs);
  }

  @Override
  public void runExpanderSetpoint(double position) {
    setpointPos = position;
    super.runExpanderSetpoint(position);
  }
}
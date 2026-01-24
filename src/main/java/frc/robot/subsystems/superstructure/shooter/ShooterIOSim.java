// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class ShooterIOSim extends ShooterIOTalonFX {
  private SecondOrderSim shooterSim;
  private double setpointPos;
  public ShooterIOSim() {
    shooterSim= new SecondOrderSim(2.5, 1, 0, 0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var talonFXSim = shooter.getSimState();
    
    var simResult = shooterSim.Evaluate(setpointPos, CodeConstants.PERIODIC_LOOP_TIME);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    
    //Do we need position for an intake?
    //talonFXSim.setRawRotorPosition(simResult.get(0) * PhysicalConstants.EXTENDER_REDUCTION);
    talonFXSim.setRotorVelocity(simResult.get(1) * PhysicalConstants.SHOOTER_REDUCTION);

    super.updateInputs(inputs);
  }
}
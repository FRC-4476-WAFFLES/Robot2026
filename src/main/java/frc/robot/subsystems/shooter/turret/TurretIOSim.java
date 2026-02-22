// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class TurretIOSim extends TurretIOTalonFX {
  private SecondOrderSim pivotSim;
  private double setpointPos, setpointVel;

  public TurretIOSim() {
    pivotSim = new SecondOrderSim(15, 1, 0, 0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var talonFXSim = turret.getSimState();

    var simResult = pivotSim.Evaluate(setpointPos, setpointVel, CodeConstants.PERIODIC_LOOP_TIME);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(simResult.get(0) * PhysicalConstants.TURRET_REDUCTION);
    talonFXSim.setRotorVelocity(simResult.get(1) * PhysicalConstants.TURRET_REDUCTION);

    double actualX = simResult.get(0);
    double y = (actualX * 400 / 36) - Math.floor(actualX * 400 / 36);
    double z = (actualX * 400 / 35) - Math.floor(actualX * 400 / 35);
    cancoder0.getSimState().setRawPosition(z);
    cancoder1.getSimState().setRawPosition(y);

    super.updateInputs(inputs);
  }

  @Override
  public void runSetpoint(double position, double velocity) {
    setpointPos = position;
    setpointVel = velocity;
    super.runSetpoint(position, velocity);
  }
}
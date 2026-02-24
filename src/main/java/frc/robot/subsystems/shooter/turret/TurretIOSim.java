// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;

public class TurretIOSim extends TurretIOTalonFX {
  private double currentOutput = 0;

  private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 50.0), gearbox);

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var appliedVoltage = gearbox.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());

    // Logger.recordOutput("RobotState/Applied Voltage", appliedVoltage);

    sim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
    sim.update(CodeConstants.PERIODIC_LOOP_TIME);

    var talonFXSim = turret.getSimState();

    talonFXSim.setRawRotorPosition(sim.getAngularPosition().in(Rotations) *
        PhysicalConstants.TURRET_REDUCTION);
    talonFXSim.setRotorVelocity(0);

    double actualX = sim.getAngularPositionRotations();
    double y = (actualX * 400 / 36) - Math.floor(actualX * 400 / 36);
    double z = (actualX * 400 / 35) - Math.floor(actualX * 400 / 35);
    cancoder0.getSimState().setRawPosition(z);
    cancoder1.getSimState().setRawPosition(y);

    super.updateInputs(inputs);
  }

  @Override
  public void runSetpoint(double position, double velocity) {
    currentOutput = (position - sim.getAngularPositionRotations()) * 40
        + (velocity) * 1.8;

    super.runSetpoint(position, velocity);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/Shooter Goal Velocity")
  private double shooterGoalVelocity = 0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/Shooter", inputs);

    if (!DriverStation.isEnabled()) {
      io.runShooterVelocity(0);
      return;
    }
    io.runShooterVelocity(shooterGoalVelocity);
  }

  public void setShooterSetpoint(double velocity) {
    shooterGoalVelocity = velocity;
  }
}

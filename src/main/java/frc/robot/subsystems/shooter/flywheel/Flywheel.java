// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @AutoLogOutput(key = "Flywheel/Flywheel Goal Velocity")
  private double flywheelGoalVelocity = 0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/Flywheel", inputs);

    if (!DriverStation.isEnabled()) {
      io.runFlywheelVelocity(0);
      return;
    }
    io.runFlywheelVelocity(flywheelGoalVelocity);
  }

  public void setFlywheelSetpoint(double velocity) {
    flywheelGoalVelocity = velocity;
  }
}

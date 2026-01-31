// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/Climber", inputs);
  }

  public void setSetpoint(double setpoint) {
    Logger.recordOutput("Climber/OutputPosition", setpoint);
    io.runClimberPosition(setpoint);
  }

  public double getPosition() {
    return inputs.climberMotor.position();
  }

  public Command moveElevator(double setpoint) {
    return Commands.runOnce(() -> setSetpoint(setpoint));
  }
}

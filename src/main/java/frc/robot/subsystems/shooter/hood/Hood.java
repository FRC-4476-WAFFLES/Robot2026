// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/Hood", inputs);
  }

  public void runSetpoint(double setpoint) {
    Logger.recordOutput("Hood/OutputPosition", setpoint);
    io.runHoodPosition(setpoint);
  }

  public double getPosition() {
    return inputs.hoodMotor.position();
  }

  public Command runSetpointCommand(DoubleSupplier angle) {
    return Commands.run(() -> {
      runSetpoint(angle.getAsDouble());
    });
  }
}

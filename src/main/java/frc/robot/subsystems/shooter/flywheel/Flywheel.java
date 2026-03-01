// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.utils.lib.EpochTimer;

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
    EpochTimer.BeginEpoch("Flywheel");
    {
      io.updateInputs(inputs);
      Logger.processInputs("Inputs/Flywheel", inputs);

      if (!RobotContainer.state.robotEnabled()) {
        io.runFlywheelVelocity(0);
        return;
      }
      io.runFlywheelVelocity(flywheelGoalVelocity);
    }
    EpochTimer.EndEpoch("Flywheel");
  }

  public void runSetpoint(double velocity) {
    flywheelGoalVelocity = velocity;
  }

  @AutoLogOutput(key = "Flywheel/At Setpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.flywheelMotorData0.velocity() - flywheelGoalVelocity) < (FlywheelConstants.RPM_RANGE / 60.0);
  }

  public Command runSetpointCommand(DoubleSupplier velocity) {
    return Commands.run(() -> {
      runSetpoint(velocity.getAsDouble());
    });
  }
}

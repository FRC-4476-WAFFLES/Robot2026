// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants.HoodConstants;
import frc.robot.utils.lib.EpochTimer;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double setpoint = 0;

  @AutoLogOutput(key = "Hood/Zeroed")
  private boolean hoodZeroed = false;

  private Trigger hoodZeroingTrigger = new Trigger(() -> inputs.hoodMotor.torqueCurrent() < -35)
      .debounce(0.1);

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    EpochTimer.BeginEpoch("Hood");
    {
      io.updateInputs(inputs);
      Logger.processInputs("Inputs/Hood", inputs);
      if (!hoodZeroed) {
        io.runHoodDutyCycle(-0.1);
        if (hoodZeroingTrigger.getAsBoolean()) {
          io.setHoodPosition(0);
          hoodZeroed = true;
        }
        return;
      }

      io.runHoodPosition(setpoint / 360);
    }
    EpochTimer.EndEpoch("Hood");
  }

  public void runSetpoint(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("Hood/OutputPosition", setpoint);

  }

  @AutoLogOutput(key = "Hood/Position")
  public double getPosition() {
    return inputs.hoodMotor.position() * 360;
  }

  @AutoLogOutput(key = "Hood/At Setpoint")
  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < HoodConstants.ANGLE_RANGE;
  }

  public void zero() {
    hoodZeroed = false;
  }

  public Command runSetpointCommand(DoubleSupplier angle) {
    return Commands.run(() -> {
      runSetpoint(angle.getAsDouble());
    });
  }
}

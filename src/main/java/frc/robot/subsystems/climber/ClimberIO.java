// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public TalonFXIOData climberMotor = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setClimberPosition(double position) {}

  default void runClimberPosition(double position) {}

  default void runClimberDutyCycle(double speed) {}
}

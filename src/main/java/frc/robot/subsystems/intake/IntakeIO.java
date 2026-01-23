// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public TalonFXIOData intakeMotor = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData expanderMotor = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setExpanderPosition(double position) {}

  default void runExpanderDutyCycle(double speed) {}

  default void runExpanderPosition(double position) {}

  default void runIntakeVelocity(double velocity) {}
}

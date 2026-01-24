// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public TalonFXIOData hoodMotor = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setHoodPosition(double position) {}

  default void runHoodPosition(double position) {}

  default void runHoodDutyCycle(double speed) {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public TalonFXIOData motorData = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public boolean zeroingSensor = false;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void setPosition(double position) {}

  default void runDutyCycle(double speed) {}

  default void runSetpoint(double position, double velocity) {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public TalonFXIOData turretMotor = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public Rotation2d absolutePosition = Rotation2d.kZero;
    public double relativePosition = 0;
    public double velocity = 0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void setPosition(double position) {}

  default void runDutyCycle(double speed) {}

  default void runSetpoint(double position, double velocity) {}
}

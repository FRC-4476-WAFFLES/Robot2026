// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.data.Constants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class FlywheelIOTalonFX implements FlywheelIO {
  // Hardware Components
  protected final TalonFXIO flywheel;

  // Control Objects
  private final MotionMagicVelocityVoltage flywheelVelocityRequest = new MotionMagicVelocityVoltage(0);

  public FlywheelIOTalonFX() {
    flywheel = new TalonFXIO(Constants.CANIds.flywheelMotor);
    // Configure hardware
    configureFlywheelMotor();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelMotorData = flywheel.getSignalData();
  }

  @Override
  public void runDutyCycle(double speed) {
    flywheel.set(speed);
  }

  @Override
  public void runFlywheelVelocity(double velocity) {
    flywheel.setControl(flywheelVelocityRequest.withVelocity(velocity));
  }

  /**
   * Configures the flywheel motor with current limits
   */
  private void configureFlywheelMotor() {
    TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs flywheelCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    flywheelConfigs.CurrentLimits = flywheelCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kV = 1.5;
    slot0Configs.kG = 0.0;
    flywheelConfigs.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 0;
    flywheelConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> flywheel.getConfigurator().apply(flywheelConfigs));
  }
}

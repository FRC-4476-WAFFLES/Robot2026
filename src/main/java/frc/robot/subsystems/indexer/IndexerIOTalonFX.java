// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.data.Constants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class IndexerIOTalonFX implements IndexerIO {
  // Hardware Components
  protected final TalonFXIO indexer;

  // Control Objects
  private final MotionMagicVelocityVoltage indexerVelocityRequest = new MotionMagicVelocityVoltage(0);

  public IndexerIOTalonFX() {
    indexer = new TalonFXIO(Constants.CANIds.indexerMotor);
    // Configure hardware
    configureIndexerMotor();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerMotorData = indexer.getSignalData();
  }

  @Override
  public void runDutyCycle(double speed) {
    indexer.set(speed);
  }

  @Override
  public void runIndexerVelocity(double velocity) {
    indexer.setControl(indexerVelocityRequest.withVelocity(velocity));
  }

  /**
   * Configures the indexer motor with current limits
   */
  private void configureIndexerMotor() {
    TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs indexerCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    indexerConfigs.CurrentLimits = indexerCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kV = 1.5;
    slot0Configs.kG = 0.0;
    indexerConfigs.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 0;
    indexerConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> indexer.getConfigurator().apply(indexerConfigs));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.data.Constants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class IndexerIOTalonFX implements IndexerIO {
  // Hardware Components
  protected final TalonFXIO indexer1;
  protected final TalonFXIO indexer2;

  // protected final TalonFXIO spindexerTwo;
  protected final TalonFXIO feeder;
  // Control Objects
  private final VelocityVoltage indexerVelocityRequest1 = new VelocityVoltage(0);
  private final Follower indexerFollowerRequest2;
  private final VelocityVoltage feederVelocityRequest = new VelocityVoltage(0);

  public IndexerIOTalonFX() {
    indexer1 = new TalonFXIO(Constants.CANIds.indexerMotor1);
    indexer2 = new TalonFXIO(Constants.CANIds.indexerMotor2);

    indexerFollowerRequest2 = new Follower(indexer1.getDeviceID(), MotorAlignmentValue.Aligned);

    feeder = new TalonFXIO(Constants.CANIds.feederMotor);

    // Configure hardware
    configureFeederMotor();
    configureIndexerMotors();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerMotorData1 = indexer1.getSignalData();
    inputs.indexerMotorData2 = indexer2.getSignalData();

    inputs.feederMotorData = feeder.getSignalData();
  }

  @Override
  public void runDutyCycle(double spindexerSpeed) {
    // indexer1.set(spindexerSpeed);
  }

  @Override
  public void runIndexerVelocity(double spindexerVelocity, double feederVelocity) {
    indexer1.setControl(indexerVelocityRequest1.withVelocity(spindexerVelocity));
    indexer2.setControl(indexerFollowerRequest2);

    feeder.setControl(feederVelocityRequest.withVelocity(feederVelocity));
  }

  private void configureIndexerMotors() {
    TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs indexerCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    indexerConfigs.CurrentLimits = indexerCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kV = 0.156;
    slot0Configs.kG = 0.0;
    indexerConfigs.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 0;
    indexerConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> indexer1.getConfigurator().apply(indexerConfigs));
    PhoenixHelpers.tryConfig(() -> indexer2.getConfigurator().apply(indexerConfigs));
  }

  private void configureFeederMotor() {
    TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs feederCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    feederConfigs.CurrentLimits = feederCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kV = 1.5;
    slot0Configs.kG = 0.0;
    feederConfigs.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 0;
    feederConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> feeder.getConfigurator().apply(feederConfigs));
  }
}

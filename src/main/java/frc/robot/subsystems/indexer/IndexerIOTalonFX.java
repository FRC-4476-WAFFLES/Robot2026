// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.data.Constants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class IndexerIOTalonFX implements IndexerIO {
  // Hardware Components
  protected final TalonFXIO indexer0;
  protected final TalonFXIO indexer1;

  protected final TalonFXIO feeder0;
  protected final TalonFXIO feeder1;

  // Control Objects
  private final VelocityTorqueCurrentFOC indexerVelocityRequest1 = new VelocityTorqueCurrentFOC(0);
  private final Follower indexerFollowerRequest;
  private final VelocityTorqueCurrentFOC feederVelocityRequest = new VelocityTorqueCurrentFOC(0);
  private final Follower feederFollowerRequest;

  public IndexerIOTalonFX() {
    indexer0 = new TalonFXIO(Constants.CANIds.indexerMotor1);
    indexer1 = new TalonFXIO(Constants.CANIds.indexerMotor2);

    indexerFollowerRequest = new Follower(indexer0.getDeviceID(), MotorAlignmentValue.Aligned);

    feeder0 = new TalonFXIO(Constants.CANIds.feederMotor0);
    feeder1 = new TalonFXIO(Constants.CANIds.feederMotor1);

    feederFollowerRequest = new Follower(feeder0.getDeviceID(), MotorAlignmentValue.Aligned);

    // Configure hardware
    configureFeederMotors();
    configureIndexerMotors();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerMotorData0 = indexer0.getSignalData();
    inputs.indexerMotorData1 = indexer1.getSignalData();

    inputs.feederMotorData0 = feeder0.getSignalData();
    inputs.feederMotorData1 = feeder1.getSignalData();
  }

  @Override
  public void runDutyCycle(double spindexerSpeed) {
    // indexer1.set(spindexerSpeed);
  }

  @Override
  public void runIndexerVelocity(double spindexerVelocity, double feederVelocity) {
    indexer0.setControl(indexerVelocityRequest1.withVelocity(spindexerVelocity));
    indexer1.setControl(indexerFollowerRequest);

    feeder0.setControl(feederVelocityRequest.withVelocity(feederVelocity));
    feeder1.setControl(feederFollowerRequest);
  }

  private void configureIndexerMotors() {
    TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs indexerCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(150)
        .withStatorCurrentLimitEnable(true);

    indexerConfigs.CurrentLimits = indexerCurrentLimit;

    // var slot0Configs = new Slot0Configs();
    // slot0Configs.kP = 0.5;
    // slot0Configs.kI = 0;
    // slot0Configs.kD = 0;
    // slot0Configs.kV = 0.6;
    // slot0Configs.kS = 0.28;
    // slot0Configs.kG = 0.0;
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 30;
    slot0Configs.kS = 3;
    slot0Configs.kD = 0;
    slot0Configs.kV = 0.08;
    slot0Configs.kG = 0.0;
    indexerConfigs.Slot0 = slot0Configs;

    indexerConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.INDEXER_REDUCTION;

    indexerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Motion Magic
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 0;
    indexerConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> indexer0.getConfigurator().apply(indexerConfigs));

    indexerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // in case disabled
    PhoenixHelpers.tryConfig(() -> indexer1.getConfigurator().apply(indexerConfigs));
  }

  private void configureFeederMotors() {
    TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs feederCurrentLimit = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(25)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(120)
        .withStatorCurrentLimitEnable(true);

    feederConfigs.CurrentLimits = feederCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 3.5;
    slot0Configs.kS = 8.2001953125;
    slot0Configs.kD = 0;
    slot0Configs.kV = 0.2199999988079071;
    slot0Configs.kG = 0.0;

    feederConfigs.Slot0 = slot0Configs;

    feederConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.FEEDER_REDUCTION;

    feederConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixHelpers.tryConfig(() -> feeder0.getConfigurator().apply(feederConfigs));
    PhoenixHelpers.tryConfig(() -> feeder1.getConfigurator().apply(feederConfigs));
  }
}

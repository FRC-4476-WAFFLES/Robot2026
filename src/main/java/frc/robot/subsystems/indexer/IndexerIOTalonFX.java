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
  protected final TalonFXIO spindexer;
  protected final TalonFXIO spindexerTwo;
  protected final TalonFXIO feeder;
  // Control Objects
  private final MotionMagicVelocityVoltage spindexerVelocityRequest = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityVoltage spindexerTwoVelocityRequest = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityVoltage feederVelocityRequest = new MotionMagicVelocityVoltage(0);

  public IndexerIOTalonFX() {
    spindexer = new TalonFXIO(Constants.CANIds.spindexerMotor);
    spindexerTwo = new TalonFXIO(Constants.CANIds.spindexerMotorTwo);
    feeder = new TalonFXIO(Constants.CANIds.feederMotor);
    // Configure hardware
    configureSpindexerMotor();
    configureFeederMotor();
    configureSecondSpindexerMotor();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerMotorData = spindexer.getSignalData();
    inputs.feederMotorData = feeder.getSignalData();
  }

  @Override
  public void runDutyCycle(double spindexerSpeed, double feederSpeed) {
    spindexer.set(spindexerSpeed);
    spindexerTwo.set(spindexerSpeed);
    feeder.set(feederSpeed);
  }

  @Override
  public void runIndexerVelocity(double spindexerVelocity, double feederVelocity) {
    spindexer.setControl(spindexerVelocityRequest.withVelocity(spindexerVelocity));
    spindexerTwo.setControl(spindexerTwoVelocityRequest.withVelocity(spindexerVelocity));
    feeder.setControl(feederVelocityRequest.withVelocity(feederVelocity));
  }

  /**
   * Configures the spindexer motor with current limits
   */
  private void configureSpindexerMotor() {
    TalonFXConfiguration spindexerConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs spindexerCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    spindexerConfigs.CurrentLimits = spindexerCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kV = 1.5;
    slot0Configs.kG = 0.0;
    spindexerConfigs.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 0;
    spindexerConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> spindexer.getConfigurator().apply(spindexerConfigs));
  }
/**
   * Configures the spindexer motor with current limits
   */
  private void configureSecondSpindexerMotor() {
    TalonFXConfiguration spindexerConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs spindexerCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    spindexerConfigs.CurrentLimits = spindexerCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kV = 1.5;
    slot0Configs.kG = 0.0;
    spindexerConfigs.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 0;
    spindexerConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> spindexerTwo.getConfigurator().apply(spindexerConfigs));
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

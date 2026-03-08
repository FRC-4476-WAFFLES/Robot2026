// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.data.Constants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class FlywheelIOTalonFX implements FlywheelIO {
  // Hardware Components
  protected final TalonFXIO flywheel0;
  protected final TalonFXIO flywheel1;

  // Control Objects
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0);
  private final Follower followerRequest;

  public FlywheelIOTalonFX() {
    flywheel0 = new TalonFXIO(Constants.CANIds.flywheelMotor0);
    flywheel1 = new TalonFXIO(Constants.CANIds.flywheelMotor1);

    followerRequest = new Follower(flywheel0.getDeviceID(), MotorAlignmentValue.Opposed);

    // Configure hardware
    configureFlywheelMotor();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelMotorData0 = flywheel0.getSignalData();
    inputs.flywheelMotorData1 = flywheel1.getSignalData();
  }

  @Override
  public void runDutyCycle(double speed) {
    flywheel0.set(speed);
    flywheel1.set(speed);
  }

  @Override
  public void runFlywheelVelocity(double velocity) {
    runFlywheelVelocity(velocity, 0);
  }

  @Override
  public void runFlywheelVelocity(double velocity, double feedForward) {
    flywheel0.setControl(flywheelVelocityRequest.withVelocity(velocity).withFeedForward(feedForward));
    flywheel1.setControl(followerRequest);
  }

  /**
   * Configures the flywheel motor with current limits
   */
  private void configureFlywheelMotor() {
    TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs flywheelCurrentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(120)
        .withStatorCurrentLimitEnable(true);

    flywheelConfigs.CurrentLimits = flywheelCurrentLimit;

    flywheelConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.FLYWHEEL_REDUCTION;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 5;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kS = 10;
    slot0Configs.kV = 0.094;
    slot0Configs.kG = 0.0;
    flywheelConfigs.Slot0 = slot0Configs;

    // var slot1Configs = new Slot1Configs();
    // slot1Configs.kP = 30;
    // slot1Configs.kI = 0;
    // slot1Configs.kD = 0;
    // slot1Configs.kS = 10;
    // slot1Configs.kV = 0.094;
    // flywheelConfigs.Slot1 = slot1Configs;

    // Motion Magic
    // MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    // motionMagic.MotionMagicAcceleration = 200;
    // motionMagic.MotionMagicJerk = 0;
    // flywheelConfigs.MotionMagic = motionMagic;

    PhoenixHelpers.tryConfig(() -> flywheel0.getConfigurator().apply(flywheelConfigs));
    // flywheelConfigs.MotorOutput.Inverted = InvertedValue.;
    PhoenixHelpers.tryConfig(() -> flywheel1.getConfigurator().apply(flywheelConfigs));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  private final LoggedNetworkNumber tuningKP = new LoggedNetworkNumber("/Tuning/Flywheel kP", 5);
  private final LoggedNetworkNumber tuningKI = new LoggedNetworkNumber("/Tuning/Flywheel kI", 0);
  private final LoggedNetworkNumber tuningKD = new LoggedNetworkNumber("/Tuning/Flywheel kD", 0);
  private final LoggedNetworkNumber tuningKS = new LoggedNetworkNumber("/Tuning/Flywheel kS", 10);
  private final LoggedNetworkNumber tuningKV = new LoggedNetworkNumber("/Tuning/Flywheel kV", 0.094);
  private final LoggedNetworkNumber tuningTarget = new LoggedNetworkNumber("/Tuning/Flywheel Target", 0);
  private final LoggedNetworkNumber tuningRecoveryKP = new LoggedNetworkNumber("/Tuning/Flywheel Recovery kP", 30);

  private double lastKP = 5, lastKI = 0, lastKD = 0, lastKS = 10, lastKV = 0.094;
  private double lastRecoveryKP = 30;

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
    updateTuningGains();
    double target = tuningTarget.get();
    if (target != 0) {
      velocity = target;
    }
    flywheel0.setControl(flywheelVelocityRequest.withVelocity(velocity).withFeedForward(feedForward));
    flywheel1.setControl(followerRequest);
  }

  private void updateTuningGains() {
    double kP = tuningKP.get();
    double kI = tuningKI.get();
    double kD = tuningKD.get();
    double kS = tuningKS.get();
    double kV = tuningKV.get();
    if (kP != lastKP || kI != lastKI || kD != lastKD || kS != lastKS || kV != lastKV) {
      var slot0 = new Slot0Configs();
      slot0.kP = kP;
      slot0.kI = kI;
      slot0.kD = kD;
      slot0.kS = kS;
      slot0.kV = kV;
      flywheel0.getConfigurator().apply(slot0);
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
      lastKS = kS;
      lastKV = kV;
    }

    double recoveryKP = tuningRecoveryKP.get();
    if (recoveryKP != lastRecoveryKP) {
      var slot1 = new Slot1Configs();
      slot1.kP = recoveryKP;
      slot1.kS = kS;
      slot1.kV = kV;
      flywheel0.getConfigurator().apply(slot1);
      lastRecoveryKP = recoveryKP;
    }
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

    var slot1Configs = new Slot1Configs();
    slot1Configs.kP = 30;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;
    slot1Configs.kS = 10;
    slot1Configs.kV = 0.094;
    flywheelConfigs.Slot1 = slot1Configs;

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

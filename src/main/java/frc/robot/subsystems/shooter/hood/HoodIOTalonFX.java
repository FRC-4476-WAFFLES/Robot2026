// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.data.Ports;
import frc.robot.data.Constants.HoodConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFXIO hood;

  private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0);

  public HoodIOTalonFX() {
    hood = new TalonFXIO(Ports.HOOD_MOTOR);

    ConfigureHood();
  }

  private void ConfigureHood() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Current limits
    CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(HoodConstants.MOTOR_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    configs.CurrentLimits = currentLimit;

    // Feedback
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = HoodConstants.MOTOR_kP;
    slot0Configs.kD = HoodConstants.MOTOR_kD;
    slot0Configs.kI = 0;
    slot0Configs.kS = HoodConstants.MOTOR_kS;
    slot0Configs.kV = HoodConstants.MOTOR_kV;
    slot0Configs.kA = HoodConstants.MOTOR_kA;
    configs.Slot0 = slot0Configs;

    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = HoodConstants.MAX_ACCELERATION;
    motionMagic.MotionMagicCruiseVelocity = HoodConstants.MAX_VELOCITY;
    configs.MotionMagic = motionMagic;

    configs.Feedback.SensorToMechanismRatio = PhysicalConstants.HOOD_REDUCTION;
    // configs.Feedback.RotorToSensorRatio = 1;

    configs.MotorOutput.DutyCycleNeutralDeadband = HoodConstants.MOTOR_DEADBAND;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configs.Voltage.PeakForwardVoltage = HoodConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    configs.Voltage.PeakReverseVoltage = -HoodConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    configs.Voltage.SupplyVoltageTimeConstant = 0.1;

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MAX_POSITION_ROTATIONS;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // HoodConstants.MIN_POSITION_ROTATIONS;

    PhoenixHelpers.tryConfig(() -> hood.getConfigurator().apply(configs));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodMotor = hood.getSignalData();
  }

  @Override
  public void runHoodDutyCycle(double speed) {
    hood.set(speed);
  }

  @Override
  public void runHoodPosition(double position) {
    double setpointRotations = MathUtil.clamp(
        position, HoodConstants.MIN_POSITION_ROTATIONS,
        HoodConstants.MAX_POSITION_ROTATIONS);

    hood.setControl(setpointRequest.withPosition(setpointRotations));
  }

  @Override
  public void setHoodPosition(double position) {
    hood.setPosition(position);
  }
}

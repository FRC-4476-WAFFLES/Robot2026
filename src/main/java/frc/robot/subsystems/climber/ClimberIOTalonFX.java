// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.data.Ports;
import frc.robot.data.Constants.ClimberConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class ClimberIOTalonFX implements ClimberIO {
  protected final TalonFXIO climber;

  private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0);

  public ClimberIOTalonFX() {
    climber = new TalonFXIO(Ports.CLIMBER_MOTOR);

    ConfigureClimber();
  }

  private void ConfigureClimber() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Current limits
    CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ClimberConstants.MOTOR_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    configs.CurrentLimits = currentLimit;

    // Feedback
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = ClimberConstants.MOTOR_kP;
    slot0Configs.kD = ClimberConstants.MOTOR_kD;
    slot0Configs.kI = 0;
    slot0Configs.kS = ClimberConstants.MOTOR_kS;
    slot0Configs.kV = ClimberConstants.MOTOR_kV;
    slot0Configs.kA = ClimberConstants.MOTOR_kA;
    configs.Slot0 = slot0Configs;

    configs.Feedback.SensorToMechanismRatio = PhysicalConstants.CLIMBER_REDUCTION;
    configs.Feedback.RotorToSensorRatio = 1;

    configs.MotorOutput.DutyCycleNeutralDeadband = ClimberConstants.MOTOR_DEADBAND;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configs.Voltage.PeakForwardVoltage = ClimberConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    configs.Voltage.PeakReverseVoltage = -ClimberConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    configs.Voltage.SupplyVoltageTimeConstant = 0.1;

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.MAX_POSITION_ROTATIONS;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.MIN_POSITION_ROTATIONS;

    PhoenixHelpers.tryConfig(() -> climber.getConfigurator().apply(configs));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberMotor = climber.getSignalData();
  }

  @Override
  public void runClimberDutyCycle(double speed) {
    climber.set(speed);
  }

  @Override
  public void runClimberPosition(double position) {
    double setpointRotations = MathUtil.clamp(
        position, ClimberConstants.MIN_POSITION_ROTATIONS,
        ClimberConstants.MAX_POSITION_ROTATIONS);

    climber.setControl(setpointRequest.withPosition(setpointRotations));
  }

  @Override
  public void setClimberPosition(double position) {
    climber.setPosition(position);
  }
}

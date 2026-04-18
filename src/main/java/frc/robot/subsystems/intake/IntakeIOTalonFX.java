// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.ExpanderConstants;
import frc.robot.data.Constants.IntakeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFXIO expander;
  protected final TalonFXIO intake0;
  protected final TalonFXIO intake1;

  private final MotionMagicVoltage expanderRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage intakeRequest = new VelocityVoltage(0);
  private final Follower followerRequest;

  public IntakeIOTalonFX() {
    expander = new TalonFXIO(CANIds.expanderMotor);
    intake0 = new TalonFXIO(CANIds.intakeMotor0);
    intake1 = new TalonFXIO(CANIds.intakeMotor1);

    followerRequest = new Follower(intake0.getDeviceID(), MotorAlignmentValue.Opposed);

    ConfigureExpander();
    ConfigureIntake();

    setExpanderPosition(0);
  }

  private void ConfigureIntake() {
    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(IntakeConstants.MOTOR_STATOR_CURRENT_LIMIT)
        // .withSupplyCurrentLimit(35)
        // .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimitEnable(true);
    intakeConfigs.CurrentLimits = currentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = IntakeConstants.MOTOR_kP;
    slot0Configs.kD = 0;
    slot0Configs.kS = IntakeConstants.MOTOR_kS;
    slot0Configs.kV = IntakeConstants.MOTOR_kV;
    slot0Configs.kG = 0;
    intakeConfigs.Slot0 = slot0Configs;

    intakeConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.INTAKE_REDUCTION;

    intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = IntakeConstants.MOTOR_DEADBAND;

    PhoenixHelpers.tryConfig(() -> intake0.getConfigurator().apply(intakeConfigs));
    PhoenixHelpers.tryConfig(() -> intake1.getConfigurator().apply(intakeConfigs));
  }

  private void ConfigureExpander() {
    TalonFXConfiguration extensionConfigs = new TalonFXConfiguration();

    // Current limits
    CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ExpanderConstants.MOTOR_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    extensionConfigs.CurrentLimits = currentLimit;

    // Feedback
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = ExpanderConstants.MOTOR_kP;
    slot0Configs.kD = ExpanderConstants.MOTOR_kD;
    slot0Configs.kI = 0;
    slot0Configs.kS = ExpanderConstants.MOTOR_kS;
    slot0Configs.kV = ExpanderConstants.MOTOR_kV;
    slot0Configs.kA = ExpanderConstants.MOTOR_kA;
    extensionConfigs.Slot0 = slot0Configs;

    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = ExpanderConstants.MAX_ACCELERATION;
    motionMagic.MotionMagicCruiseVelocity = ExpanderConstants.MAX_VELOCITY;
    extensionConfigs.MotionMagic = motionMagic;

    extensionConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.EXPANDER_REDUCTION;
    extensionConfigs.Feedback.RotorToSensorRatio = 1;

    extensionConfigs.MotorOutput.DutyCycleNeutralDeadband = ExpanderConstants.MOTOR_DEADBAND;
    extensionConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    extensionConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    extensionConfigs.Voltage.PeakForwardVoltage = ExpanderConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    extensionConfigs.Voltage.PeakReverseVoltage = -ExpanderConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    // extensionConfigs.Voltage.SupplyVoltageTimeConstant = 0.1;

    extensionConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ExpanderConstants.MAX_POSITION_ROTATIONS;
    extensionConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ExpanderConstants.MIN_POSITION_ROTATIONS;

    PhoenixHelpers.tryConfig(() -> expander.getConfigurator().apply(extensionConfigs));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.expanderMotor = expander.getSignalData();
    inputs.intakeMotor0 = intake0.getSignalData();
    inputs.intakeMotor1 = intake1.getSignalData();
  }

  @Override
  public void runExpanderDutyCycle(double speed) {
    expander.set(speed);
  }

  @Override
  public void runExpanderPosition(double position) {
    double setpointRotations = MathUtil.clamp(
        position, Constants.ExpanderConstants.MIN_POSITION_ROTATIONS,
        Constants.ExpanderConstants.MAX_POSITION_ROTATIONS);

    double feedforward = Math.cos(
        Units.rotationsToRadians(expander.getRawSignals().position().getValueAsDouble()) - (Math.PI / 2)
    );
    feedforward = MathUtil.clamp(feedforward, 0, 1);
    double kG = -0.2;

    expander.setControl(expanderRequest.withPosition(setpointRotations).withFeedForward(feedforward
        * kG));
  }

  @Override
  public void runIntakeVelocity(double velocity) {
    intake0.setControl(intakeRequest.withVelocity(velocity));
    intake1.setControl(followerRequest);
  }

  @Override
  public void setExpanderPosition(double position) {
    expander.setPosition(position);
  }

  @Override
  public void runIntakeDutyCycle(double dutyCycle) {
    intake0.set(MathUtil.clamp(dutyCycle, -1, 1));
    intake1.setControl(followerRequest);
  }
}

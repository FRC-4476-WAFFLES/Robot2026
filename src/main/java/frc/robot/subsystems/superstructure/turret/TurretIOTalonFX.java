// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.TurretConstants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFXIO turret;

  private final PositionVoltage setpointRequest = new PositionVoltage(0);

  public TurretIOTalonFX() {
    turret = new TalonFXIO(CANIds.turretMotor, CANIds.CANivoreBus);

    TalonFXConfiguration turretConfigs = new TalonFXConfiguration();

    // Current limits
    CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(TurretConstants.MOTOR_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    turretConfigs.CurrentLimits = currentLimit;

    // Feedback
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = TurretConstants.MOTOR_kP;
    slot0Configs.kD = TurretConstants.MOTOR_kD;
    slot0Configs.kI = 0;
    slot0Configs.kS = TurretConstants.MOTOR_kS;
    slot0Configs.kV = TurretConstants.MOTOR_kV;
    slot0Configs.kA = TurretConstants.MOTOR_kA;
    turretConfigs.Slot0 = slot0Configs;

    turretConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.TURRET_REDUCTION;
    turretConfigs.Feedback.RotorToSensorRatio = 1;

    turretConfigs.MotorOutput.DutyCycleNeutralDeadband = TurretConstants.MOTOR_DEADBAND;
    turretConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turretConfigs.Voltage.PeakForwardVoltage = TurretConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    turretConfigs.Voltage.PeakReverseVoltage = -TurretConstants.MOTOR_PEAK_SUPPLY_VOLTAGE;
    turretConfigs.Voltage.SupplyVoltageTimeConstant = 0.1;

    turretConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_POSITION_ROTATIONS;
    turretConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_POSITION_ROTATIONS;

    PhoenixHelpers.tryConfig(() -> turret.getConfigurator().apply(turretConfigs));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorData = turret.getSignalData();
  }

  @Override
  public void runDutyCycle(double speed) {
    turret.set(speed); // Applies throttle percentage (-1 to 1)
  }

  @Override
  public void runSetpoint(double position, double velocity) {
    double setpointRotations = MathUtil.clamp(
        position, Constants.TurretConstants.MIN_POSITION_ROTATIONS,
        Constants.TurretConstants.MAX_POSITION_ROTATIONS);

    turret.setControl(setpointRequest.withPosition(setpointRotations).withVelocity(velocity));
    // Run control mode (so a control request from phoneix 6)
  }

  @Override
  public void setPosition(double position) {
    turret.setPosition(position); // Tells motor where it is
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.Mode;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.TurretConstants;
import frc.robot.utils.hardware.CANcoderIO;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFXIO turret;
  protected final CANcoderIO cancoder0; // 35
  protected final CANcoderIO cancoder1;

  private final PositionVoltage setpointRequest = new PositionVoltage(0);

  // protected final StatusSignal<Angle> absolutePosition0;
  // protected final StatusSignal<AngularVelocity> velocity0;
  // protected final StatusSignal<Angle> absolutePosition1;

  protected boolean turretZeroed = false;

  public TurretIOTalonFX() {
    turret = new TalonFXIO(CANIds.turretMotor, CANIds.CANivoreBus);
    cancoder0 = new CANcoderIO(CANIds.turretEncoder0, CANIds.CANivoreBus, 250);
    cancoder1 = new CANcoderIO(CANIds.turretEncoder1, CANIds.CANivoreBus, 250);

    var cancoder0Config = new CANcoderConfiguration();
    cancoder0Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoder0Config.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_0_OFFSET;
    cancoder0Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // makes output in range of 0-1
    PhoenixHelpers.tryConfig(() -> cancoder0.getConfigurator().apply(cancoder0Config));

    var cancoder1Config = new CANcoderConfiguration();
    cancoder1Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoder1Config.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_1_OFFSET;
    cancoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    PhoenixHelpers.tryConfig(() -> cancoder1.getConfigurator().apply(cancoder1Config));

    if (Constants.getMode() == Mode.SIM) {
      cancoder0.getSimState().setRawPosition(TurretConstants.CANCODER_0_OFFSET);
      cancoder1.getSimState().setRawPosition(TurretConstants.CANCODER_1_OFFSET);
    }

    // Status signals
    // absolutePosition0 = cancoder0.getAbsolutePosition();
    // velocity0 = cancoder0.getVelocity();
    // absolutePosition1 = cancoder1.getAbsolutePosition();

    configureMotor();
  }

  private void configureMotor() {
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

    if (Constants.getMode() == Mode.SIM) {
      turretConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.TURRET_REDUCTION;

    } else {
      // Fuse to 36t cancoder

      turretConfigs.Feedback.RotorToSensorRatio = PhysicalConstants.TURRET_REDUCTION
          / PhysicalConstants.TURRET_ENCODER_1_REDUCTION;
      turretConfigs.Feedback.FeedbackRemoteSensorID = cancoder1.getDeviceID();
      turretConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      turretConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.TURRET_ENCODER_1_REDUCTION;
    }

    PhoenixHelpers.tryConfig(() -> turret.getConfigurator().apply(turretConfigs));
  }

  // Borderline black magic I do not fully understand
  private double calculateTurretStartupPosition(boolean waitForStartup) {
    if (waitForStartup) {
      BaseStatusSignal.waitForAll(10.0, cancoder0.getRawSignals().absolutePosition(),
          cancoder1.getRawSignals().absolutePosition());
    }

    double z = cancoder0.getRawSignals().absolutePosition().getValueAsDouble(); // 35t
    double y = cancoder1.getRawSignals().absolutePosition().getValueAsDouble(); // 36t

    // Debugging
    if (Constants.getMode() == Mode.SIM) {
      // z = 0;
      // y = 0;
      double xValue = 0.25;
      y = (xValue * 160 / 36) - Math.floor(xValue * 160 / 36);
      z = (xValue * 160 / 35) - Math.floor(xValue * 160 / 35);
    }

    double difference = z - y;
    if (difference < 0)
      difference += 1;

    double fullrange = PhysicalConstants.ENCODER_0_TEETH * PhysicalConstants.ENCODER_1_TEETH
        / PhysicalConstants.TURRET_GEAR_TEETH;

    double simpleVernier = difference * fullrange;
    double n = PhysicalConstants.ENCODER_0_TEETH * difference - y;

    if (simpleVernier > (fullrange / 2)) {
      simpleVernier -= fullrange;
      n -= PhysicalConstants.ENCODER_0_TEETH; // Works
      // n -= 36; // Cursed
    }

    // Removes noise from simpleVernier if the gear is off by less than a tooth or
    // two
    // Actually makes things worse if there's more significant error
    double x1 = PhysicalConstants.ENCODER_1_TEETH * (Math.round(n) + y) / PhysicalConstants.TURRET_GEAR_TEETH;
    // double x1 = PhysicalConstants.ENCODER_0_TEETH * (Math.round(n) + z) /
    // PhysicalConstants.TURRET_GEAR_TEETH; // Cursed
    return x1;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (!turretZeroed) {
      // Get initial turret pos
      turret.setPosition(calculateTurretStartupPosition(true));
      turretZeroed = true;
    }
    // inputs.motorData = turret.getSignalData();
    Logger.recordOutput("Turret/Zeroing Diagnostic", calculateTurretStartupPosition(false));

    // inputs.absolutePosition = Rotation2d
    // .fromRotations(BaseStatusSignal.getLatencyCompensatedValue(cancoder1.getRawSignals().absolutePosition(),
    // cancoder1.getRawSignals().velocity()).in(Rotations));
    inputs.relativePosition = BaseStatusSignal.getLatencyCompensatedValue(turret.getRawSignals().position(),
        turret.getRawSignals().velocity()).in(Rotations);
    inputs.absolutePosition = Rotation2d.fromRotations(inputs.relativePosition).plus(TurretConstants.PHYSICAL_ZERO);
    inputs.velocity = turret.getRawSignals().velocity().getValueAsDouble();
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

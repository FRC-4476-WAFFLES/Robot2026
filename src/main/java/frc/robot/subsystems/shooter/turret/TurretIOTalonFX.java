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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.data.Constants;
import frc.robot.data.Ports;
import frc.robot.data.Constants.Mode;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.TurretConstants;
import frc.robot.utils.hardware.CANcoderIO;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;
import frc.robot.utils.lib.WafflesUtilities;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFXIO turret;
  protected final CANcoderIO cancoder0; // 35
  protected final CANcoderIO cancoder1;

  private final PositionVoltage setpointRequest = new PositionVoltage(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double relativePosition = 0;

  // protected final StatusSignal<Angle> absolutePosition0;
  // protected final StatusSignal<AngularVelocity> velocity0;
  // protected final StatusSignal<Angle> absolutePosition1;

  protected boolean turretZeroed = false;

  public TurretIOTalonFX() {
    turret = new TalonFXIO(Ports.TURRET_MOTOR);
    cancoder0 = new CANcoderIO(Ports.TURRET_ENCODER_0, 250);
    cancoder1 = new CANcoderIO(Ports.TURRET_ENCODER_1, 250);

    var cancoder0Config = new CANcoderConfiguration();
    cancoder0Config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder0Config.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_0_OFFSET;
    // cancoder0Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // makes
    // output in range of 0-1
    PhoenixHelpers.tryConfig(() -> cancoder0.getConfigurator().apply(cancoder0Config));

    var cancoder1Config = new CANcoderConfiguration();
    cancoder1Config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder1Config.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_1_OFFSET;
    // cancoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
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
    slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    turretConfigs.Slot0 = slot0Configs;

    Slot1Configs slot1Configs = new Slot1Configs();
    slot1Configs.kP = 50;
    slot1Configs.kD = 0;
    slot1Configs.kI = 0;
    slot1Configs.kS = 0;
    slot1Configs.kV = 0;
    slot1Configs.kA = 0;
    slot1Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    turretConfigs.Slot1 = slot1Configs;

    turretConfigs.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MAX_VELOCITY;
    turretConfigs.MotionMagic.MotionMagicAcceleration = TurretConstants.MAX_ACCELERATION;

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

      // turretConfigs.Feedback.SensorToMechanismRatio =
      // PhysicalConstants.TURRET_REDUCTION;

      turretConfigs.Feedback.RotorToSensorRatio = PhysicalConstants.TURRET_REDUCTION
          / PhysicalConstants.TURRET_ENCODER_1_REDUCTION;
      turretConfigs.Feedback.FeedbackRemoteSensorID = cancoder1.getDeviceID();
      turretConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      turretConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.TURRET_ENCODER_1_REDUCTION;
    }

    PhoenixHelpers.tryConfig(() -> turret.getConfigurator().apply(turretConfigs));
  }

  // Borderline black magic I do not fully understand
  @SuppressWarnings("unused")
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

    Logger.recordOutput("Turret/y", y);
    Logger.recordOutput("Turret/z", z);
    Logger.recordOutput("Turret/Difference", difference);

    double fullrange = PhysicalConstants.ENCODER_0_TEETH * PhysicalConstants.ENCODER_1_TEETH
        / PhysicalConstants.TURRET_GEAR_TEETH;

    double simpleVernier = difference * fullrange;
    double n = PhysicalConstants.ENCODER_0_TEETH * difference - z;

    if (simpleVernier > (fullrange / 2)) {
      simpleVernier -= fullrange;
      // n -= PhysicalConstants.ENCODER_0_TEETH; // Works

      n -= 36; // Cursed
    } else {

    }

    // Removes noise from simpleVernier if the gear is off by less than a tooth or
    // two
    // Actually makes things worse if there's more significant error
    // double x1 = PhysicalConstants.ENCODER_1_TEETH * (Math.round(n) + y) /
    // PhysicalConstants.TURRET_GEAR_TEETH;
    double x1 = PhysicalConstants.ENCODER_0_TEETH * (Math.round(n) + z) /
        PhysicalConstants.TURRET_GEAR_TEETH; // Cursed
    return x1;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (!turretZeroed) {
      // Get initial turret pos
      // setPosition(calculateTurretStartupPosition(true));
      turretZeroed = true;
    }
    inputs.turretMotor = turret.getSignalData();
    // Logger.recordOutput("Turret/Zeroing Diagnostic",
    // calculateTurretStartupPosition(false));

    // inputs.absolutePosition = Rotation2d
    // .fromRotations(BaseStatusSignal.getLatencyCompensatedValue(cancoder1.getRawSignals().absolutePosition(),
    // cancoder1.getRawSignals().velocity()).in(Rotations));
    inputs.relativePosition = BaseStatusSignal.getLatencyCompensatedValue(turret.getRawSignals().position(),
        turret.getRawSignals().velocity()).in(Rotations);
    inputs.absolutePosition = Rotation2d.fromRotations(inputs.relativePosition).plus(TurretConstants.PHYSICAL_ZERO);
    inputs.velocity = turret.getRawSignals().velocity().getValueAsDouble();

    relativePosition = inputs.relativePosition;
  }

  @Override
  public void runDutyCycle(double speed) {
    turret.set(speed);
  }

  @Override
  public void runSetpoint(double position, double velocity) {
    double setpointRotations = MathUtil.clamp(
        position, Constants.TurretConstants.MIN_POSITION_ROTATIONS,
        Constants.TurretConstants.MAX_POSITION_ROTATIONS);

    double feedforward = 1.0;
    double deadband = 0.09;
    double deadbandOuter = 0.2;
    // double springFF = 0;
    // if (relativePosition > deadband) {
    // feedforward = springFF;
    // } else if (relativePosition < deadband) {
    // feedforward = -springFF;
    // }

    double ff = feedforward *
        MathUtil.clamp(WafflesUtilities.InvLerp(deadband, deadbandOuter, Math.abs(relativePosition)), 0, 1)
        * Math.signum(relativePosition);

    Logger.recordOutput("Turret/Feedforward", ff);
    // turret.setControl(
    // setpointRequest.withPosition(setpointRotations).awithVelocity(velocity).withFeedForward(feedforward));
    if ((Math.abs(relativePosition - position) > Units.degreesToRotations(30))) {
      turret.setControl(motionMagicRequest.withPosition(setpointRotations).withSlot(1));
      Logger.recordOutput("Turret/ControlScheme", "MM");
    } else {
      turret.setControl(
          setpointRequest.withPosition(setpointRotations).withSlot(0).withVelocity(velocity).withFeedForward(ff));
      Logger.recordOutput("Turret/ControlScheme", "PV");
    }
  }

  @Override
  public void setPosition(double position) {
    Logger.recordOutput("Turret/ZeroedPos", position);
    PhoenixHelpers.tryConfig(5, () -> turret.setPosition(position));
  }
}

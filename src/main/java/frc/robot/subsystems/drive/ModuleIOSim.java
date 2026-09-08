// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.sim.SimBattery;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared
  // locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  /** Whatever the power manager last applied. Effectively unlimited until it does. */
  private double driveSupplyLimit = 1000.0;
  /** The stator ceiling the real module configures, from the same constants. */
  private final double slipCurrent;
  private final double driveGearRatio;
  private final double turnGearRatio;
  /** Steer stator limit, matching what TunerConstants configures. */
  private static final double TURN_STATOR_LIMIT = 35.0;
  private final String name;

  private static int created = 0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    name = "Module" + created++;
    slipCurrent = constants.SlipCurrent;
    driveGearRatio = constants.DriveMotorGearRatio;
    turnGearRatio = constants.SteerMotorGearRatio;
    // Create drive and turn sim models
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
        DRIVE_GEARBOX);
    turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
        TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * The voltage that keeps a motor's stator current inside {@code limit}.
   *
   * <p>
   * For a DC motor the current is {@code (applied - backEMF) / R}, so holding
   * the current inside a band is holding the voltage inside a band centred on
   * the back-EMF. Exact for the model being simulated rather than an
   * approximation that has to be tuned.
   */
  private static double clampToStator(double volts, DCMotorSim sim, double gearRatio,
      DCMotor gearbox, double limit) {
    double backEmf = sim.getAngularVelocityRadPerSec() * gearRatio / gearbox.KvRadPerSecPerVolt;
    double headroom = limit * gearbox.rOhms;
    return MathUtil.clamp(volts, backEmf - headroom, backEmf + headroom);
  }

  /**
   * The voltage that keeps a motor's <i>supply</i> current inside {@code limit}.
   *
   * <p>
   * See the call site for the derivation. Signs are resolved along the direction
   * being driven, so braking — where the back-EMF opposes the applied voltage
   * and the current is therefore larger — is bounded correctly too.
   */
  private static double clampToSupply(double volts, DCMotorSim sim, double gearRatio,
      DCMotor gearbox, double limit, double bus) {
    if (volts == 0 || limit <= 0) {
      return volts;
    }
    double sign = Math.signum(volts);
    double magnitude = Math.abs(volts);
    double backEmf = sign * sim.getAngularVelocityRadPerSec() * gearRatio
        / gearbox.KvRadPerSecPerVolt;
    double root = (backEmf
        + Math.sqrt(backEmf * backEmf + 4 * limit * gearbox.rOhms * bus)) / 2;
    return sign * Math.min(magnitude, Math.max(0, root));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Clamp to what the battery can actually deliver rather than a fixed 12 V.
    // A drivetrain that keeps its authority as the pack sags is simulating a
    // robot that does not exist, and is exactly the case the power manager was
    // written for.
    double bus = SimBattery.getVoltage();
    driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -bus, bus);

    // Honour the stator limit the real module configures, which this had been
    // missing entirely: ModuleIOTalonFX sets StatorCurrentLimit to SlipCurrent,
    // and without it each simulated Kraken pulls stall current. Four of them
    // then drew about 500 A and pinned the simulated pack at its floor, which
    // is more than the real robot's worst measured total of 348 A and made
    // every voltage reading in simulation meaningless.
    //
    // Exact for the DC model rather than a fudge: current is (V - backEMF) / R,
    // so bounding the current is just bounding the voltage either side of the
    // back-EMF.
    driveAppliedVolts = clampToStator(driveAppliedVolts, driveSim, driveGearRatio,
        DRIVE_GEARBOX, slipCurrent);
    turnAppliedVolts = clampToStator(turnAppliedVolts, turnSim, turnGearRatio,
        TURN_GEARBOX, TURN_STATOR_LIMIT);

    // Honour the supply current limit the power manager applies.
    //
    // Supply current is stator current times duty cycle. Both depend on the
    // voltage being solved for, so backing off by a single scale factor does not
    // converge — measured, a 45 A per-module limit still let four modules pull
    // 666 A. Solving it properly is one quadratic:
    //
    // stator = (V - backEMF) / R, duty = V / bus
    // (V - backEMF) / R * V / bus <= limit
    // V^2 - backEMF*V - limit*R*bus <= 0
    //
    // so V is bounded by the positive root.
    driveAppliedVolts = clampToSupply(driveAppliedVolts, driveSim, driveGearRatio,
        DRIVE_GEARBOX, driveSupplyLimit, bus);

    driveSim.setInputVoltage(driveAppliedVolts);
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -bus, bus));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Report what this module is taking out of the pack.
    SimBattery.setLoad(name + "/Drive",
        Math.abs(driveSim.getCurrentDrawAmps()) * Math.abs(driveAppliedVolts) / Math.max(1.0, bus));
    SimBattery.setLoad(name + "/Turn",
        Math.abs(turnSim.getCurrentDrawAmps()) * Math.abs(turnAppliedVolts) / Math.max(1.0, bus));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
    inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
    inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public boolean setDriveSupplyCurrentLimit(double supplyCurrentLimit) {
    driveSupplyLimit = supplyCurrentLimit;
    return true;
  }
}

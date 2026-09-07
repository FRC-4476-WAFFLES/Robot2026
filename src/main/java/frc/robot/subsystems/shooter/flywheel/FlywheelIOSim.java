// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.sim.SimBattery;

/**
 * A flywheel that behaves like the real one, identified from match logs rather
 * than guessed.
 *
 * <p>
 * What it replaced was a second-order response driven straight from the setpoint
 * — no torque, no current limit, no load and no battery. The simulated wheel
 * reached any goal at a fixed rate however little current it was allowed, which
 * made three things untestable: whether a current limit helps, whether the
 * readiness gate closes on a wheel that cannot keep up, and whether anything
 * browns out.
 *
 * <p>
 * Every constant below came out of {@code logReview}:
 *
 * <ul>
 * <li>{@code ACCELERATION_PER_AMP} — 2.311 rps/s per amp of stator, fitted from
 * 3918 accelerating samples.
 * <li>{@code CEILING_*} — the fastest the wheel turns at a given bus voltage:
 * 69 rps at 7.75 V rising to a drag-limited 88 rps above about 10 V. These are
 * genuine saturation points, from a passing bug that commanded a speed which
 * does not exist and left the wheel flat out.
 * <li>{@code AMPS_PER_RPS_OF_HEADROOM} — how much current the wheel can pull
 * given the room between its speed and that ceiling. At 25 rps it drew a
 * measured 108 A; this model gives 108.
 * </ul>
 *
 * <p>
 * The consequence worth understanding: near its goal the wheel is close to the
 * ceiling, so there is little headroom and little current available however
 * generous the limit. That is why raising the current limit does nothing at
 * setpoint and everything during a deep dip, and it is the behaviour the whole
 * power investigation turned on.
 */
public class FlywheelIOSim extends FlywheelIOTalonFX {
  private static final double ACCELERATION_PER_AMP = 2.311;
  private static final double CEILING_RPS_PER_VOLT = 13.8;
  private static final double CEILING_VOLTAGE_OFFSET = 2.72;
  private static final double CEILING_MAX_RPS = 88.0;
  private static final double AMPS_PER_RPS_OF_HEADROOM = 1.71;
  /** Windage, so a wheel left alone coasts down rather than spinning forever. */
  private static final double DRAG_RPS_PER_SECOND_AT_FULL_SPEED = 6.0;

  /**
   * The instance the simulation is running, so a test can disturb the wheel
   * without the subsystem having to expose its IO layer. One robot per JVM.
   */
  private static FlywheelIOSim active;

  private double velocity = 0;
  private double goalVelocity = 0;
  private double feedForwardAmps = 0;
  private double statorLimit = FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT;
  private double supplyLimit = FlywheelConstants.MOTOR_SUPPLY_CURRENT_LIMIT;

  public FlywheelIOSim() {
    active = this;
  }

  /** The running simulated flywheel, or null outside simulation. */
  public static FlywheelIOSim getActive() {
    return active;
  }

  /** The fastest the wheel can turn on a given bus, in rps. */
  public static double speedCeiling(double busVoltage) {
    return Math.min(CEILING_MAX_RPS,
        Math.max(0, CEILING_RPS_PER_VOLT * (busVoltage - CEILING_VOLTAGE_OFFSET)));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double dt = CodeConstants.PERIODIC_LOOP_TIME;
    double bus = SimBattery.getVoltage();
    double ceiling = speedCeiling(bus);

    // What the controller is asking for, in amps, mirroring the gains the real
    // Talon is configured with.
    double error = goalVelocity - velocity;
    double demanded = goalVelocity < 1.0 ? 0
        : 11.0 * error + 0.026 * goalVelocity + 5.5 + feedForwardAmps;

    // What the wheel can actually take. Current comes from the room between the
    // present speed and the fastest this bus allows, so a wheel near its goal
    // can pull very little whatever its limit says.
    double available = Math.max(0, ceiling - velocity) * AMPS_PER_RPS_OF_HEADROOM;
    double duty = ceiling <= 0 ? 0 : MathUtil.clamp(velocity / ceiling, 0.05, 1.0);
    double supplyAllowedStator = supplyLimit / duty;

    double stator = MathUtil.clamp(demanded, -statorLimit,
        Math.min(Math.min(statorLimit, available), supplyAllowedStator));

    double drag = DRAG_RPS_PER_SECOND_AT_FULL_SPEED * (velocity / CEILING_MAX_RPS);
    velocity = Math.max(0, velocity + (ACCELERATION_PER_AMP * stator - drag) * dt);

    double supply = Math.abs(stator) * duty;
    SimBattery.setLoad("Flywheel", supply * 2);
    Logger.recordOutput("SimFlywheel/Ceiling", ceiling);
    Logger.recordOutput("SimFlywheel/Stator", stator);

    var talonFXSim = flywheel0.getSimState();
    talonFXSim.setRotorVelocity(velocity * PhysicalConstants.FLYWHEEL_REDUCTION);
    talonFXSim.setSupplyVoltage(bus);

    super.updateInputs(inputs);
  }

  /**
   * Takes the energy a ball costs out of the wheel.
   *
   * <p>
   * Measured across 184 shots: a median drop of 7.8 rps, 12.8 at the 75th
   * percentile and 18 at the 90th. This is the disturbance the readiness gate
   * has to tolerate without shutting, and the one the flywheel has to recover
   * from between shots.
   */
  public void takeShot(double rpsLost) {
    velocity = Math.max(0, velocity - rpsLost);
  }

  /** The simulated wheel speed, in rps. */
  public double getVelocity() {
    return velocity;
  }

  @Override
  public boolean setCurrentLimits(double statorCurrentLimit, double supplyCurrentLimit) {
    statorLimit = statorCurrentLimit;
    supplyLimit = supplyCurrentLimit;
    return true;
  }

  @Override
  public void runFlywheelVelocity(double velocitySetpoint) {
    runFlywheelVelocity(velocitySetpoint, 0);
  }

  @Override
  public void runFlywheelVelocity(double velocitySetpoint, double feedforward) {
    goalVelocity = velocitySetpoint;
    feedForwardAmps = feedforward;
    super.runFlywheelVelocity(velocitySetpoint, feedforward);
  }
}

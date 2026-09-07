// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * A battery for the simulation, so that current limits mean something.
 *
 * <p>
 * Without this every simulated mechanism has an infinite 12 V supply, which
 * makes the entire power manager inert: a state that caps the drivetrain at 10 A
 * behaves exactly like one that caps it at 45 A, and nothing ever browns out.
 * The measurements that motivated all of that work cannot be reproduced.
 *
 * <p>
 * The numbers are fitted from 63,000 samples of real match logs rather than
 * assumed — {@code logReview battery}. Open circuit 11.79 V and 15.5 mOhm of
 * internal resistance, which is what a healthy pack plus its cable, main breaker
 * and SB50 actually measured. Every 100 A of draw costs 1.55 V of bus.
 *
 * <p>
 * Each simulated mechanism reports what it is drawing; the voltage that comes
 * back is what the rest of the robot sees, including through
 * {@code RobotController.getBatteryVoltage}. A mechanism that ignores the
 * returned voltage is simulating a robot that does not exist.
 */
public final class SimBattery {
  /** Fitted from the match logs. A tired pack reads higher; a fresh one lower. */
  private static final double DEFAULT_RESISTANCE = 0.0155;
  private static final double DEFAULT_OPEN_CIRCUIT = 11.79;
  /**
   * Below this the roboRIO has long since given up, and the linear model stops
   * meaning anything. Clamping keeps a runaway load from producing negative
   * volts and NaNs downstream.
   */
  private static final double MINIMUM_VOLTAGE = 4.0;

  private static final Map<String, Double> loads = new ConcurrentHashMap<>();
  private static volatile double resistance = DEFAULT_RESISTANCE;
  private static volatile double openCircuit = DEFAULT_OPEN_CIRCUIT;

  private SimBattery() {}

  /**
   * Reports what one mechanism is drawing from the battery, in supply amps.
   *
   * @param source a stable name; calling again with the same name replaces the
   *     previous value rather than adding to it
   */
  public static void setLoad(String source, double amps) {
    loads.put(source, Math.max(0, amps));
  }

  /** Everything currently being drawn, in amps. */
  public static double getTotalCurrent() {
    double total = 0;
    for (double amps : loads.values()) {
      total += amps;
    }
    return total;
  }

  /** The bus voltage under the present load. */
  public static double getVoltage() {
    return Math.max(MINIMUM_VOLTAGE, openCircuit - getTotalCurrent() * resistance);
  }

  /**
   * Publishes the simulated voltage to the HAL and the log, so the rest of the
   * robot sees it exactly as it would on hardware. Call once per loop, after
   * every mechanism has reported its load.
   */
  public static void publish() {
    double volts = getVoltage();
    RoboRioSim.setVInVoltage(volts);
    Logger.recordOutput("SimBattery/Voltage", volts);
    Logger.recordOutput("SimBattery/Total Current", getTotalCurrent());
    for (var load : loads.entrySet()) {
      Logger.recordOutput("SimBattery/Loads/" + load.getKey(), load.getValue());
    }
  }

  /**
   * Sets how tired the pack is.
   *
   * <p>
   * Worth exercising: real packs measured 11.2 to 15.0 mOhm across one event, and
   * the difference is 1.14 V of sag at 300 A — more than three times what the
   * drivetrain current cap buys. A test that only ever runs on a perfect battery
   * is not testing the case that loses matches.
   */
  public static void setPack(double openCircuitVolts, double internalResistanceOhms) {
    openCircuit = openCircuitVolts;
    resistance = internalResistanceOhms;
  }

  /** Back to the fitted healthy pack, with nothing drawing. */
  public static void reset() {
    loads.clear();
    resistance = DEFAULT_RESISTANCE;
    openCircuit = DEFAULT_OPEN_CIRCUIT;
  }
}

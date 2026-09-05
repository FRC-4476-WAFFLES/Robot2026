// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import java.lang.reflect.Method;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.data.Constants.CodeConstants;

/**
 * Boots the real robot headlessly and steps it a loop at a time, so tests can
 * drive the whole stack without hardware, a GUI, or a driver station.
 *
 * <p>
 * Loops run in real time, one every 20 ms. Do <b>not</b> reach for
 * {@code SimHooks.pauseTiming()} / {@code stepTiming()} to make this faster and
 * deterministic — {@code PhoenixOdometryThread} waits on CAN signals, and with
 * the sim clock paused that wait never completes while it holds
 * {@code Drive.odometryLock}, so {@code Drive.earlyPeriodic()} deadlocks and the
 * test hangs forever. Real-time stepping is the price of the odometry thread.
 *
 * <p>
 * A consequence worth knowing: a test that waits N seconds takes N seconds. Keep
 * waits short, and prefer asserting on a subsystem reaching a state over a fixed
 * settling time.
 *
 * <p>
 * <b>One robot per JVM.</b> {@code RobotContainer}'s subsystems are static and
 * initialize once, so {@link #boot} is idempotent and every test class in a run
 * shares the same robot. Tests must not assume a clean slate — put the robot
 * into the state you need at the start of each test rather than expecting
 * defaults.
 */
public final class SimHarness {
  private static Robot robot;

  private SimHarness() {}

  /** Boots the robot if it is not already up. Safe to call from every test. */
  public static synchronized void boot() {
    if (robot != null) {
      return;
    }

    assertTrue(HAL.initialize(500, 0), "HAL initialization failed");

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    robot = new Robot();
  }

  /** Runs the robot for a number of 20 ms loops, in real time. */
  public static void step(int loops) {
    final long periodNanos = (long) (CodeConstants.PERIODIC_LOOP_TIME * 1e9);

    for (int i = 0; i < loops; i++) {
      long start = System.nanoTime();
      loggerBeforeUser();
      robot.robotPeriodic();
      loggerAfterUser(System.nanoTime() - start);

      // Sleep only the remainder of the period, so a loop costs 20 ms in total
      // like it does on the robot rather than 20 ms plus however long work took.
      long remaining = periodNanos - (System.nanoTime() - start);
      if (remaining > 0) {
        try {
          Thread.sleep(remaining / 1_000_000L, (int) (remaining % 1_000_000L));
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new IllegalStateException("sim stepping interrupted", e);
        }
      }
    }
  }

  /*
   * LoggedRobot drives these around each user loop, and they are what advances
   * Timer.getTimestamp(). Without them the AdvantageKit clock is frozen, so every
   * debouncer, timer and motion profile in robot code stalls forever. They are
   * package-private in AdvantageKit, hence the reflection.
   */
  private static Method periodicBeforeUser;
  private static Method periodicAfterUser;

  private static void loggerBeforeUser() {
    try {
      if (periodicBeforeUser == null) {
        periodicBeforeUser = Logger.class.getDeclaredMethod("periodicBeforeUser");
        periodicBeforeUser.setAccessible(true);
      }
      periodicBeforeUser.invoke(null);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("AdvantageKit's loop hooks moved; SimHarness needs updating", e);
    }
  }

  private static void loggerAfterUser(long userCodeNanos) {
    try {
      if (periodicAfterUser == null) {
        periodicAfterUser = Logger.class.getDeclaredMethod("periodicAfterUser", long.class, long.class);
        periodicAfterUser.setAccessible(true);
      }
      periodicAfterUser.invoke(null, userCodeNanos, 0L);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("AdvantageKit's loop hooks moved; SimHarness needs updating", e);
    }
  }

  /** Runs the robot for a number of seconds. Takes that long in real time. */
  public static void stepSeconds(double seconds) {
    step((int) Math.round(seconds / CodeConstants.PERIODIC_LOOP_TIME));
  }

  /** Enables the robot in teleop. */
  public static void enableTeleop() {
    setDs(true, false);
  }

  /** Enables the robot in autonomous. */
  public static void enableAutonomous() {
    setDs(true, true);
  }

  /** Disables the robot. */
  public static void disable() {
    setDs(false, false);
  }

  private static void setDs(boolean enabled, boolean autonomous) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    // The enabled flag is cached once per loop in RobotState.updateEnabledState
    step(1);
  }
}

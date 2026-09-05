// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import java.lang.reflect.Method;

import org.littletonrobotics.junction.AutoLogOutputManager;
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
 * <b>One robot per JVM.</b> {@code RobotContainer}'s subsystems are static, so a
 * robot is built once and cannot be rebuilt after {@link #shutdown} calls
 * {@code Logger.end()}. {@code build.gradle} sets {@code forkEvery = 1} so every
 * test class gets a fresh JVM and therefore a fresh robot. Within a single class,
 * state still carries between test methods — set up what you need at the start of
 * each test and leave the robot disabled when you finish.
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
    attachControllers();
    DriverStationSim.notifyNewData();

    robot = new Robot();

    // LoggedRobot does this inside loopFunc, which this harness bypasses. Without
    // it no @AutoLogOutput field is ever published.
    AutoLogOutputManager.addObject(robot);

    // Without this the WPILOG file is left unflushed and SimLog reads nothing.
    Runtime.getRuntime().addShutdownHook(new Thread(SimHarness::shutdown));
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

  /**
   * Flushes and closes the log so {@link SimLog} can read it. Registered as a JVM
   * shutdown hook by {@link #boot}, so tests do not normally call this.
   */
  public static synchronized void shutdown() {
    if (robot == null) {
      return;
    }
    Logger.end();
    robot = null;
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

  /* ---------------------------------------------------------------------- */
  /* Controls                                                                */
  /* ---------------------------------------------------------------------- */

  public static final int LEFT_JOYSTICK = Controls.DriverConstants.kLeftJoystickPort;
  public static final int RIGHT_JOYSTICK = Controls.DriverConstants.kRightJoystickPort;
  public static final int OPERATOR = Controls.OperatorConstants.kOperatorControllerPort;

  /**
   * Tells the simulated driver station that controllers are plugged in. Without
   * this every button read is rejected with "Joystick Button N on port M not
   * available" and no trigger binding can ever fire.
   */
  private static void attachControllers() {
    for (int port : new int[] {LEFT_JOYSTICK, RIGHT_JOYSTICK, OPERATOR}) {
      DriverStationSim.setJoystickButtonCount(port, 16);
      DriverStationSim.setJoystickAxisCount(port, 6);
      DriverStationSim.setJoystickPOVCount(port, 1);
      DriverStationSim.setJoystickPOV(port, 0, -1); // -1 means "not pressed"
    }
    DriverStationSim.setJoystickIsXbox(OPERATOR, true);
  }

  /**
   * Sets a button and runs one loop so the bound trigger sees it.
   *
   * @param port one of {@link #LEFT_JOYSTICK}, {@link #RIGHT_JOYSTICK}, {@link #OPERATOR}
   * @param button 1-based, matching {@code CommandJoystick.button(n)} and
   *     {@code XboxController.Button.kA.value}
   */
  public static void setButton(int port, int button, boolean pressed) {
    DriverStationSim.setJoystickButton(port, button, pressed);
    DriverStationSim.notifyNewData();
    step(1);
  }

  /** Presses a button, holds it for a number of loops, then releases it. */
  public static void tapButton(int port, int button, int holdLoops) {
    setButton(port, button, true);
    step(holdLoops);
    setButton(port, button, false);
  }

  /** Presses and releases a button, holding it for one loop. */
  public static void tapButton(int port, int button) {
    tapButton(port, button, 1);
  }

  /**
   * Sets an axis and runs one loop.
   *
   * @param axis 0-based. On the joysticks 0 is X and 1 is Y; on the Xbox
   *     controller 2 is the left trigger and 3 is the right trigger.
   */
  public static void setAxis(int port, int axis, double value) {
    DriverStationSim.setJoystickAxis(port, axis, value);
    DriverStationSim.notifyNewData();
    step(1);
  }

  /** Sets the D-pad angle in degrees, or -1 to release it. */
  public static void setPov(int port, int degrees) {
    DriverStationSim.setJoystickPOV(port, 0, degrees);
    DriverStationSim.notifyNewData();
    step(1);
  }

  /** Releases every button, axis and D-pad on every controller. */
  public static void releaseAllControls() {
    for (int port : new int[] {LEFT_JOYSTICK, RIGHT_JOYSTICK, OPERATOR}) {
      DriverStationSim.setJoystickButtons(port, 0);
      for (int axis = 0; axis < 6; axis++) {
        DriverStationSim.setJoystickAxis(port, axis, 0.0);
      }
      DriverStationSim.setJoystickPOV(port, 0, -1);
    }
    DriverStationSim.notifyNewData();
    step(1);
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

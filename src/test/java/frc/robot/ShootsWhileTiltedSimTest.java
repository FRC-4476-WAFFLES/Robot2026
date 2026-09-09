// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.shooter.ShotPlanner;
import frc.robot.utils.sim.SimShooter;

/**
 * Does the robot, as built, put the ball in the goal while standing on a slope?
 *
 * <p>
 * The earlier check of this ran the geometry on its own. This one boots the
 * whole robot and asks {@code ShotPlanner} for a real answer, so it covers the
 * wiring as well as the maths — the shot map lookup, the tilt solve, the hood
 * clamp and the speed ratio, in the order the robot actually does them.
 *
 * <p>
 * What it does not do is pull the trigger. Firing needs the turret on target and
 * the flywheel up to speed, which takes seconds of real time in this harness and
 * would test the gates rather than the aim. The parameters are the aim, so the
 * ball is flown from those instead.
 */
public class ShootsWhileTiltedSimTest {
  private static final double GRAVITY = 9.81;
  private static final double GOAL_RADIUS = 0.35;

  @BeforeAll
  static void boot() {
    SimHarness.boot();
    SimHarness.setAlliance(Alliance.Blue);
  }

  @AfterAll
  static void putItBack() {
    GyroIOSim.reset();
    SimHarness.disable();
  }

  /** Where a ball with these parameters crosses the height of the goal. */
  private static double rangeOf(ShotPlanner.ShootingParameters p, double launchElevation) {
    double speed = SimShooter.exitSpeedFor(p.flywheelSpeed());
    double th = Units.degreesToRadians(launchElevation);
    double climb = FieldConstants.Hub.topCenterPoint.getZ()
        - PhysicalConstants.ROBOT_TO_TURRET_CENTER.getZ();
    double vz = speed * Math.sin(th);
    double vx = speed * Math.cos(th);
    double disc = vz * vz - 2 * GRAVITY * climb;
    if (disc < 0) {
      return Double.NaN;
    }
    return vx * (vz + Math.sqrt(disc)) / GRAVITY;
  }

  /**
   * Aims from a spot at a tilt, and reports how far the ball lands from where
   * it was aimed.
   */
  private static double missFrom(double x, double y, double headingDegrees,
      double tiltDegrees, double leanTowardRadians) {
    RobotContainer.drive.setPose(
        new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees)));
    GyroIOSim.setTilt(tiltDegrees, leanTowardRadians);
    // Long enough for odometry and the gravity vector to reach RobotState.
    SimHarness.step(6);

    var p = ShotPlanner.aimToHub();
    // What the hood was asked for, put through the tilt the robot is sitting
    // at, is the angle the ball truly leaves along.
    var actual = frc.robot.utils.lib.TiltedShot.forward(
        RobotContainer.state.getGravityVector(),
        p.turretSetpoint().heading().minus(RobotContainer.state.getRotation()).getRadians(),
        SimShooter.launchElevationFor(p.hoodAngle()));
    double landed = rangeOf(p, actual.launchElevationDegrees());
    return Double.isNaN(landed) ? Double.POSITIVE_INFINITY
        : Math.abs(landed - p.distanceToTarget());
  }

  @Test
  void itShootsFromLevelGround() {
    SimHarness.enableTeleop();
    for (double x : new double[] { 9.0, 8.0, 7.0 }) {
      double miss = missFrom(x, 4.02, 0, 0.0, 0.0);
      System.out.printf("level at x=%.1f: misses by %.2f m%n", x, miss);
      assertTrue(miss < GOAL_RADIUS,
          "a level shot from x=" + x + " should go in, missed by " + miss);
    }
    SimHarness.disable();
  }

  @Test
  void itShootsFromTheBump() {
    SimHarness.enableTeleop();
    // Nose-up on the ramp, the case that produced nothing but misses.
    for (double tilt : new double[] { 5.0, 9.2, 14.0 }) {
      double miss = missFrom(9.0, 4.02, 0, tilt, 0.0);
      System.out.printf("tilted %.1f deg: misses by %.2f m%n", tilt, miss);
      assertTrue(miss < GOAL_RADIUS,
          "a shot tilted " + tilt + " degrees should still go in, missed by " + miss);
    }
    SimHarness.disable();
  }

  @Test
  void itShootsLeaningAnyDirection() {
    SimHarness.enableTeleop();
    // Climbing, dropping off, and standing across the ramp all lean differently
    // and a correction that only knows one of them makes the others worse.
    for (double toward : new double[] { 0.0, Math.PI, Math.PI / 2, -Math.PI / 2 }) {
      double miss = missFrom(9.0, 4.02, 0, 9.2, toward);
      System.out.printf("leaning toward %5.2f rad: misses by %.2f m%n", toward, miss);
      assertTrue(miss < GOAL_RADIUS,
          "leaning toward " + toward + " missed by " + miss);
    }
    SimHarness.disable();
  }

  @Test
  void itShootsTiltedFromAnyHeading() {
    SimHarness.enableTeleop();
    // The robot does not have to be square to the ramp. Turned across it, the
    // same lean becomes roll instead of pitch and swings the bearing rather
    // than the elevation.
    for (double heading : new double[] { 0, 45, 90, 135, 180 }) {
      double miss = missFrom(9.0, 4.02, heading, 9.2, 0.0);
      System.out.printf("heading %3.0f deg, tilted 9.2: misses by %.2f m%n", heading, miss);
      assertTrue(miss < GOAL_RADIUS,
          "at heading " + heading + " it missed by " + miss);
    }
    SimHarness.disable();
  }
}

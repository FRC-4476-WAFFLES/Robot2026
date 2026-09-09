// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.data.Constants.HoodConstants;
import frc.robot.utils.lib.SplineMonotone1D;
import frc.robot.utils.lib.TiltedShot;
import frc.robot.utils.sim.SimShooter;

/**
 * Does a shot taken from a tilted robot actually go in?
 *
 * <p>
 * The whole pipeline in one place: take a distance, look the shot up in the
 * real maps, put the robot on a slope, work out where the ball truly leaves,
 * fly it, and see where it crosses the goal. No robot is booted — the point is
 * the geometry and the maps, and a whole-robot test would run in real time and
 * say less.
 *
 * <p>
 * The launch model comes from {@link SimShooter} rather than being copied here.
 * A duplicated one drifts, and then this passes on a shooter that no longer
 * exists.
 *
 * <p>
 * Both cases are measured every time: the shot as the robot fires it today, and
 * the shot with the tilt solved for. The first is expected to miss — that is
 * the bug — and the second to go in. Reporting them together means the test
 * says how much the fix is worth rather than only whether it compiles.
 */
public class TiltedShotsGoInTest {
  private static final SplineMonotone1D FLYWHEEL = new SplineMonotone1D(FlywheelConstants.DistanceMap);
  private static final SplineMonotone1D HOOD = new SplineMonotone1D(HoodConstants.DistanceMap);

  private static final double LAUNCH_HEIGHT = 0.49093120;
  private static final double HUB_HEIGHT = 72 * 0.0254;
  /** Half the goal's width: inside this and the ball is in. */
  private static final double GOAL_RADIUS = 0.35;
  private static final double GRAVITY = 9.81;

  /** Where a ball fired at this speed and elevation crosses the goal height. */
  private static double rangeAtGoalHeight(double speed, double elevationDegrees) {
    double th = Units.degreesToRadians(elevationDegrees);
    double vz = speed * Math.sin(th);
    double vx = speed * Math.cos(th);
    double gain = HUB_HEIGHT - LAUNCH_HEIGHT;
    double disc = vz * vz - 2 * GRAVITY * gain;
    if (disc < 0) {
      return Double.NaN; // never gets that high
    }
    return vx * (vz + Math.sqrt(disc)) / GRAVITY;
  }

  /**
   * Gravity as the gyro reports it: the field's "up" seen from the robot.
   *
   * <p>
   * Note which way this leans. A robot nose-up on a ramp sees "up" tilted
   * toward its own nose, so a shot fired that way leaves steeper — the bump
   * case. That is the opposite of the surface normal, which leans backward, and
   * passing one where the other belongs silently tests the wrong half of the
   * problem. It reads as a working correction either way, because both signs
   * miss, which is what makes the mistake worth naming here.
   */
  private static Translation3d gravity(double tiltDegrees, double towardRadians) {
    double t = Units.degreesToRadians(tiltDegrees);
    return new Translation3d(
        Math.sin(t) * Math.cos(towardRadians),
        Math.sin(t) * Math.sin(towardRadians),
        Math.cos(t));
  }

  /** Miss distance for a shot fired the way the robot fires one today. */
  private static double missUncorrected(double distance, double tilt, double toward) {
    double hoodRot = HOOD.interpolate(distance);
    double rps = FLYWHEEL.interpolate(distance);
    double elevation = SimShooter.launchElevationFor(hoodRot);
    double speed = SimShooter.exitSpeedFor(rps);
    // The tilt rotates the barrel; nothing compensates.
    var actual = TiltedShot.forward(gravity(tilt, toward), 0.0, elevation);
    double landed = rangeAtGoalHeight(speed, actual.launchElevationDegrees());
    return Double.isNaN(landed) ? Double.POSITIVE_INFINITY : Math.abs(landed - distance);
  }

  /** Miss distance with the tilt solved for and the speed adjusted to suit. */
  private static double missCorrected(double distance, double tilt, double toward) {
    double hoodRot = HOOD.interpolate(distance);
    double elevation = SimShooter.launchElevationFor(hoodRot);
    var g = gravity(tilt, toward);
    var solved = TiltedShot.solve(g, 0.0, elevation,
        SimShooter.hoodMinElevation(), SimShooter.hoodMaxElevation());
    double needed = TiltedShot.requiredSpeed(distance, solved.launchElevationDegrees(),
        HUB_HEIGHT - LAUNCH_HEIGHT);
    if (Double.isNaN(needed)) {
      return Double.POSITIVE_INFINITY;
    }
    double landed = rangeAtGoalHeight(needed, solved.launchElevationDegrees());
    return Double.isNaN(landed) ? Double.POSITIVE_INFINITY : Math.abs(landed - distance);
  }

  @Test
  void shotsGoInFromEveryTiltTheBumpProduces() {
    // Distances that are actually shot from, and tilts the bump actually
    // produces: the median on it is 9.2 degrees and the steepest seen is 20.
    double[] distances = { 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 7.0 };
    double[] tilts = { 0, 5, 9.2, 14, 20 };

    System.out.printf("%8s %7s %14s %14s%n", "range", "tilt", "as fired", "tilt solved");
    int firedIn = 0;
    int solvedIn = 0;
    int total = 0;
    double worstSolved = 0;
    for (double d : distances) {
      for (double tilt : tilts) {
        // Nose-up on the ramp, so the shot leaves steeper: the bump case.
        double bad = missUncorrected(d, tilt, 0.0);
        double good = missCorrected(d, tilt, 0.0);
        total++;
        if (bad < GOAL_RADIUS) {
          firedIn++;
        }
        if (good < GOAL_RADIUS) {
          solvedIn++;
          worstSolved = Math.max(worstSolved, good);
        }
        System.out.printf("%7.1fm %6.1f° %12s %14s%n", d, tilt,
            Double.isInfinite(bad) ? "never lands"
                : String.format("%.2f m %s", bad,
                    bad < GOAL_RADIUS ? "IN " : "out"),
            Double.isInfinite(good) ? "impossible"
                : String.format("%.2f m %s", good,
                    good < GOAL_RADIUS ? "IN " : "out"));
      }
    }
    System.out.printf("%n as fired: %d of %d in.   tilt solved: %d of %d in.%n",
        firedIn, total, solvedIn, total);

    assertTrue(solvedIn > firedIn,
        "solving for the tilt has to land more shots than not solving for it: "
            + solvedIn + " against " + firedIn);
    assertTrue(solvedIn >= total - 2,
        "and it should land nearly all of them, got " + solvedIn + " of " + total);
  }

  @Test
  void aLevelRobotIsUnaffected() {
    // The correction must be inert when there is nothing to correct, or it is
    // worse than no correction at all.
    for (double d : new double[] { 2.0, 3.0, 5.0, 7.0 }) {
      double miss = missCorrected(d, 0.0, 0.0);
      assertTrue(miss < 0.05,
          "a level shot at " + d + " m should land where it was aimed, missed by " + miss);
    }
  }

  @Test
  void leaningEitherWayIsHandled() {
    // Climbing the near face and dropping off the far one lean opposite ways,
    // and a correction that only knows about one of them makes the other worse.
    for (double toward : new double[] { 0.0, Math.PI, Math.PI / 2, -Math.PI / 2 }) {
      double miss = missCorrected(3.0, 9.2, toward);
      assertTrue(miss < GOAL_RADIUS,
          "a 3 m shot leaning toward " + toward + " missed by " + miss);
    }
  }
}

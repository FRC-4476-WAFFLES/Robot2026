// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.utils.sim.SimShooter;
import frc.robot.utils.vendor.FuelSim;

/**
 * Do the balls go in the goal?
 *
 * <p>
 * Every other test here computes where a ball would cross the goal's height and
 * compares that to a radius. This one pulls the trigger, lets maplesim fly the
 * balls, and reads the hub's own counter. The hub scores a ball that is within
 * 0.56 m of its centre <i>and</i> descending through 1.83 m, so it judges the
 * lateral error and the entry direction as well as the range — none of which a
 * one-number miss distance can see.
 *
 * <p>
 * It also runs through the real command. That matters more than it sounds:
 * {@code RobotContainer}'s hub-mode binding applied the hand-tuned bump offsets
 * on top of parameters {@code ShotPlanner} had already corrected, so the robot
 * corrected twice on the bump — and every test that called {@code aimToHub()}
 * directly passed straight through it. Driving the trigger rather than the
 * planner is what closes that gap.
 *
 * <p>
 * Slow by construction: {@link SimHarness} steps in real time, so each volley
 * costs its own few seconds. That is the price of firing real balls.
 */
public class BallsGoInTheGoalTest {
  /** How long to hold the trigger. Long enough for the turret to get there. */
  private static final double VOLLEY_SECONDS = 2.0;
  /** Long enough for everything fired to land or score. */
  private static final double FLIGHT_SECONDS = 2.5;

  /** Where the hood actually was at the end of the last volley, and where it was told to be. */
  private static double hoodPosition;
  private static double hoodGoal;

  @BeforeAll
  static void boot() {
    SimHarness.boot();
    SimHarness.setAlliance(Alliance.Blue);
    SimHarness.enableTeleop();
  }

  @AfterAll
  static void putItBack() {
    SimShooter.setEnabled(false);
    GyroIOSim.reset();
    SimHarness.disable();
  }

  /** Both hubs, so the test does not care which one this alliance shoots at. */
  private static int scored() {
    return FuelSim.Hub.BLUE_HUB.getScore() + FuelSim.Hub.RED_HUB.getScore();
  }

  private static void resetScores() {
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
  }

  /**
   * Stands the robot somewhere, tilts it, holds the trigger, and reports how
   * many balls were fired and how many the hub counted.
   *
   * @return {@code {fired, scored}}
   */
  private static int[] volley(double x, double y, double headingDegrees,
      double tiltDegrees, double leanTowardRadians) {
    RobotContainer.drive.setPose(new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees)));
    GyroIOSim.setTilt(tiltDegrees, leanTowardRadians);
    SimShooter.setEnabled(true);
    // Let odometry, the heading and the gravity vector settle, and give the
    // turret a head start on slewing before the trigger is held.
    SimHarness.stepSeconds(0.6);

    int firedBefore = SimShooter.getShotsFired();
    resetScores();

    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kRightTrigger.value, 1.0);
    SimHarness.stepSeconds(VOLLEY_SECONDS);
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kRightTrigger.value, 0.0);

    // Read before the flight wait, so it is the hood as it was while firing
    // rather than wherever it drifted to afterwards.
    hoodPosition = RobotContainer.hood.getPosition();
    hoodGoal = RobotContainer.hood.getGoalPosition();

    SimShooter.setEnabled(false);
    SimHarness.stepSeconds(FLIGHT_SECONDS);

    return new int[] { SimShooter.getShotsFired() - firedBefore, scored() };
  }

  @Test
  void ballsGoInFromEveryAttitudeTheBumpProduces() {
    // Judges the tilt correction, so there is nothing to judge while it is off.
    assumeTrue(CodeConstants.COMPENSATE_FOR_TILT, "tilt compensation is turned off");
    double hubX = FieldConstants.Hub.topCenterPoint.getX();
    double hubY = FieldConstants.Hub.topCenterPoint.getY();
    System.out.printf("aiming at the hub at (%.2f, %.2f)%n", hubX, hubY);

    double[] ranges = { 2.5, 4.0, 5.5 };
    // setTilt takes the direction the *surface normal* leans, and GyroIOSim
    // inverts it to get gravity -- so azimuth 180 is the near face, climbing,
    // nose up, and azimuth 0 is dropping off the far side. Naming these the
    // other way round is the same surface-normal-for-gravity mix-up that once
    // had a whole sweep testing the descending face and calling it the bump.
    double[][] leans = { { 0, 0 }, { 9.2, 180 }, { 9.2, 0 }, { 9.2, 90 } };
    String[] leanNames = { "level", "nose up", "nose down", "roll left" };

    int totalFired = 0;
    int totalScored = 0;
    int levelFired = 0;
    int levelScored = 0;
    int tiltedFired = 0;
    int tiltedScored = 0;

    System.out.printf("%n%7s %-10s %7s %7s %6s%n", "range", "attitude", "fired", "scored", "rate");
    for (double range : ranges) {
      // Straight out from the hub along -X, which is where the bump is and
      // where the robot actually shoots from.
      double x = hubX - range;
      double y = hubY;
      double heading = 0.0; // facing the hub
      for (int i = 0; i < leans.length; i++) {
        int[] r = volley(x, y, heading, leans[i][0], Units.degreesToRadians(leans[i][1]));
        int fired = r[0];
        int in = r[1];
        totalFired += fired;
        totalScored += in;
        if (leans[i][0] == 0) {
          levelFired += fired;
          levelScored += in;
        } else {
          tiltedFired += fired;
          tiltedScored += in;
        }
        System.out.printf("%6.1fm %-10s %7d %7d %5.0f%%%n", range, leanNames[i], fired, in,
            fired == 0 ? 0.0 : 100.0 * in / fired);
      }
    }

    System.out.printf("%n%d of %d balls scored (%.0f%%)%n", totalScored, totalFired,
        totalFired == 0 ? 0.0 : 100.0 * totalScored / totalFired);
    double levelRate = levelFired == 0 ? 0 : (double) levelScored / levelFired;
    double tiltedRate = tiltedFired == 0 ? 0 : (double) tiltedScored / tiltedFired;
    System.out.printf("level %.0f%% (%d/%d), tilted %.0f%% (%d/%d)%n",
        100 * levelRate, levelScored, levelFired,
        100 * tiltedRate, tiltedScored, tiltedFired);

    assertTrue(totalFired > 0, "nothing was fired at all, so this measured nothing");
    // The question is whether tilt costs anything, so tilted is judged against
    // level rather than against perfection. Level is the shooter's own accuracy
    // -- scatter, spin-up, the map -- and this correction does not control it.
    assertTrue(tiltedRate >= levelRate - 0.15,
        "tilt should not cost much once corrected: level " + levelRate
            + ", tilted " + tiltedRate);
  }

  /**
   * Where it stops working.
   *
   * <p>
   * The sweep above finds no failure that tilt is responsible for, which says
   * the correction holds but not where it runs out. This pushes the tilt past
   * anything the bump produces -- its median is 9.2 degrees and the steepest
   * seen all season is 20 -- until the hood has no travel left and the flywheel
   * is carrying the whole error, so the drive team knows which shots to trust.
   */
  @Test
  void andHereIsWhereItStops() {
    double hubX = FieldConstants.Hub.topCenterPoint.getX();
    double hubY = FieldConstants.Hub.topCenterPoint.getY();

    double[] tilts = { 0, 9.2, 14, 20, 26 };
    double[] ranges = { 3.0, 5.0 };

    System.out.printf("%n%7s %7s %7s %7s %6s%n", "range", "tilt", "fired", "scored", "rate");
    for (double range : ranges) {
      for (double tilt : tilts) {
        // Climbing the near face: nose up, the shot leaves steeper, the case
        // the bump actually produces.
        int[] r = volley(hubX - range, hubY, 0.0, tilt, Math.PI);
        System.out.printf("%6.1fm %6.1f° %7d %7d %5.0f%%   hood %5.1f -> %5.1f%n",
            range, tilt, r[0], r[1], r[0] == 0 ? 0.0 : 100.0 * r[1] / r[0],
            hoodPosition, hoodGoal);
      }
    }
  }
}

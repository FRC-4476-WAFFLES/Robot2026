// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.utils.lib.SplineMonotone1D;

/**
 * Checks the flywheel readiness tolerance against what it is meant to mean: the
 * shot still lands inside the goal, with room left for everything else that
 * aims it.
 *
 * <p>
 * Range error is {@code 2 * distance * speedError / speed}, so the tolerance is
 * derived from an acceptable range error rather than fixed. This walks the shot
 * map and confirms the derived number really does hold the range error near
 * {@code ACCEPTABLE_RANGE_ERROR} across the distances shots are actually taken
 * from, which a single fixed tolerance cannot do.
 */
public class ShotGateTest {
  /** Distances covering the range hub shots were taken from in the match logs. */
  private static final double[] SHOT_DISTANCES = { 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0 };

  private static double toleranceAt(double distance, double goal) {
    return MathUtil.clamp(goal * FlywheelConstants.ACCEPTABLE_RANGE_ERROR / (2 * distance),
        FlywheelConstants.MIN_VELOCITY_TOLERANCE, FlywheelConstants.MAX_VELOCITY_TOLERANCE);
  }

  @Test
  void theToleranceKeepsRangeErrorInsideTheGoal() {
    var flywheel = new SplineMonotone1D(FlywheelConstants.DistanceMap);
    System.out.printf("%10s %10s %12s %14s%n", "distance", "goal", "tolerance", "range error");

    for (double distance : SHOT_DISTANCES) {
      double goal = flywheel.interpolate(distance);
      double tolerance = toleranceAt(distance, goal);
      // Invert the relation: what range error does that tolerance permit?
      double rangeError = 2 * distance * tolerance / goal;
      System.out.printf("%9.1fm %9.1f %11.1f rps %11.2f m%n", distance, goal, tolerance, rangeError);

      // The bounds on the derived tolerance let it drift slightly at the far
      // end, where MIN_VELOCITY_TOLERANCE takes over. That is deliberate: a
      // tolerance below 3 rps would be tighter than the wheel can reliably hold.
      assertTrue(rangeError <= FlywheelConstants.ACCEPTABLE_RANGE_ERROR * 1.1,
          String.format("at %.1fm the tolerance permits %.2fm of range error, well above the %.2fm allowed",
              distance, rangeError, FlywheelConstants.ACCEPTABLE_RANGE_ERROR));
    }
  }

  @Test
  void theFlywheelDoesNotClaimTheWholeGoal() {
    // The goal is about a metre across, so 0.5m either side is the entire error
    // budget. Turret aim, pose and hood all spend from it too, so the flywheel
    // taking more than about two thirds of it would leave nothing for them.
    assertTrue(FlywheelConstants.ACCEPTABLE_RANGE_ERROR < 0.5,
        "the flywheel must leave range error budget for aim, pose and hood");
  }

  @Test
  void theToleranceIsLooserUpCloseThanFarOut() {
    // The whole point of deriving it. A fixed tolerance is simultaneously too
    // strict close in, where it rejected a fifth of shots the drive team lands,
    // and too loose far out.
    var flywheel = new SplineMonotone1D(FlywheelConstants.DistanceMap);
    double close = toleranceAt(2.0, flywheel.interpolate(2.0));
    double far = toleranceAt(4.5, flywheel.interpolate(4.5));
    assertTrue(close > far,
        "a close shot must tolerate more speed error than a long one, got "
            + close + " rps close and " + far + " rps far");
  }

  @Test
  void theOldFixedToleranceWouldHaveAllowedAMiss() {
    // Guards the reason this exists. RPM_RANGE is 1200 RPM, and at 3.5m that
    // permits 2.6m of range error against a goal about 1m across -- which is
    // what the logs show, hub shots reported ready while 20 rps slow.
    var flywheel = new SplineMonotone1D(FlywheelConstants.DistanceMap);
    double goal = flywheel.interpolate(3.5);
    double oldRangeError = 2 * 3.5 * (FlywheelConstants.RPM_RANGE / 60.0) / goal;
    System.out.printf("the old %.0f RPM window permitted %.1f m of range error at 3.5m%n",
        FlywheelConstants.RPM_RANGE, oldRangeError);
    assertTrue(oldRangeError > 2.0,
        "expected the old window to permit a large miss, which is why it was replaced");
  }
}

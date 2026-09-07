// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Checks the bearing rate that feeds the turret's velocity feedforward.
 *
 * <p>
 * It replaced a numerical derivative of the aim angle passed through a hundred
 * millisecond moving average, which cost about fifty milliseconds of lag —
 * nearly six degrees of turret error while rotating at two radians a second, and
 * the reason aim suffered most while turning. The closed form has no lag at all,
 * so the only thing worth checking is that it is right.
 *
 * <p>
 * The maths is duplicated here rather than reached for through {@code
 * ShotPlanner}, whose static aiming path needs a whole robot. The point is to
 * pin the relation itself, so a future edit that gets a sign or a divisor wrong
 * fails here rather than on a field.
 */
public class ShotLeadTest {
  /** How fast the bearing to a fixed target turns, in rotations per second. */
  private static double bearingRate(Translation2d lineOfSight, Translation2d velocity) {
    double range = lineOfSight.getNorm();
    return Units.radiansToRotations(
        (velocity.getX() * lineOfSight.getY() - velocity.getY() * lineOfSight.getX())
            / (range * range));
  }

  @Test
  void drivingStraightAtATargetNeedsNoLead() {
    // Closing head on changes the range and not the bearing, so the turret
    // should be asked to hold still.
    var lineOfSight = new Translation2d(4.0, 0);
    var velocity = new Translation2d(3.0, 0);
    assertEquals(0.0, bearingRate(lineOfSight, velocity), 1e-9,
        "closing straight on should need no turret velocity");
  }

  @Test
  void drivingAcrossTheTargetNeedsTheMostLead() {
    // Pure crossing motion: every bit of the velocity turns the bearing.
    var lineOfSight = new Translation2d(4.0, 0);
    var velocity = new Translation2d(0, 3.0);
    // Bearing turns at v / r, and moving to the left of the line of sight turns
    // the bearing clockwise, hence negative.
    assertEquals(Units.radiansToRotations(-3.0 / 4.0), bearingRate(lineOfSight, velocity), 1e-9);
  }

  @Test
  void theSameSpeedNeedsLessLeadFurtherAway() {
    // The reason this divides by range rather than being a fixed offset: a shot
    // from across the field barely needs leading, one from underneath needs a
    // great deal.
    var velocity = new Translation2d(0, 3.0);
    double near = Math.abs(bearingRate(new Translation2d(2.0, 0), velocity));
    double far = Math.abs(bearingRate(new Translation2d(8.0, 0), velocity));
    System.out.printf("bearing rate at 2m: %.3f rot/s, at 8m: %.3f rot/s%n", near, far);
    assertTrue(near > far * 3.5,
        "lead should fall off with range, got " + near + " near and " + far + " far");
  }

  @Test
  void itMatchesADirectlyDifferentiatedAngle() {
    // The closed form has to agree with what the old numerical derivative was
    // trying to measure, or it is simply a different quantity computed faster.
    var lineOfSight = new Translation2d(3.0, 1.5);
    var velocity = new Translation2d(-1.2, 2.4);
    double dt = 1e-4;

    double before = Math.atan2(lineOfSight.getY(), lineOfSight.getX());
    // The turret moves by v * dt, so the line of sight shortens by the same.
    var moved = lineOfSight.minus(velocity.times(dt));
    double after = Math.atan2(moved.getY(), moved.getX());
    double numerical = Units.radiansToRotations((after - before) / dt);

    double closedForm = bearingRate(lineOfSight, velocity);
    System.out.printf("numerical %.6f rot/s, closed form %.6f rot/s%n", numerical, closedForm);
    assertEquals(numerical, closedForm, 1e-4,
        "the closed form must agree with differentiating the angle");
  }
}

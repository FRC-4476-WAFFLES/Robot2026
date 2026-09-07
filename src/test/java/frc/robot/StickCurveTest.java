// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Guards the stick shaping against the two ways it could quietly go wrong: giving
 * up top speed, or reversing somewhere in the middle so the robot speeds up as
 * the driver eases off.
 */
public class StickCurveTest {
  @Test
  void fullDeflectionStillGivesFullOutput() {
    // The reason for blending toward cubic rather than scaling everything down.
    // Softening the middle must not cost top speed.
    for (double curve : new double[] { 0.0, 0.5, 0.7, 1.0 }) {
      assertEquals(1.0, Controls.applyCurve(1.0, curve), 1e-9, "curve " + curve + " at full stick");
      assertEquals(-1.0, Controls.applyCurve(-1.0, curve), 1e-9, "curve " + curve + " at full reverse");
      assertEquals(0.0, Controls.applyCurve(0.0, curve), 1e-9, "curve " + curve + " at centre");
    }
  }

  @Test
  void aStickThatCannotQuiteReachTheEndStillGivesFullSpeed() {
    // A thumbstick often tops out below 1.0, worse on a diagonal and worse as it
    // wears. Without an upper deadband the driver pushes as hard as the stick
    // goes and never gets full speed. Team 581 uses the same 0.95.
    assertEquals(1.0, Controls.applyCurve(Controls.UPPER_DEADBAND, Controls.TRANSLATION_CURVE), 1e-9,
        "a stick at the upper deadband must count as full");
    assertEquals(1.0, Controls.applyCurve(0.97, Controls.ROTATION_CURVE), 1e-9,
        "past the upper deadband must stay at full, not exceed it");

    var velocity = Controls.getLinearVelocityFromJoysticks(0.67, 0.67);
    assertEquals(1.0, velocity.getNorm(), 0.02,
        "a diagonal that cannot quite reach the corner should still give full speed");
  }

  @Test
  void theCurveIsMonotonic() {
    // A curve that dips would mean pushing the stick further sometimes slows the
    // robot down, which is far worse than any amount of twitchiness.
    for (double curve : new double[] { 0.0, 0.5, 0.7, 1.0 }) {
      double previous = -1.1;
      // Stop short of the upper deadband, where the curve deliberately plateaus.
      for (double input = -0.94; input <= 0.94; input += 0.01) {
        double output = Controls.applyCurve(input, curve);
        assertTrue(output > previous,
            String.format("curve %.1f is not monotonic near %.2f", curve, input));
        previous = output;
      }
    }
  }

  @Test
  void aCurvedStickIsGentlerInTheMiddleThanALinearOne() {
    // The whole point. Half stick should ask for less than half speed.
    double half = Controls.applyCurve(0.5, Controls.TRANSLATION_CURVE);
    assertTrue(half < 0.5, "half stick should ask for less than half speed, got " + half);
    assertTrue(half > 0.2, "half stick should not be crippled either, got " + half);

    System.out.printf("%8s %14s %14s%n", "stick", "translation", "rotation");
    for (double input = 0.0; input <= 1.0001; input += 0.125) {
      System.out.printf("%8.3f %14.3f %14.3f%n", input,
          Controls.applyCurve(input, Controls.TRANSLATION_CURVE),
          Controls.applyRotationCurve(input));
    }
  }

  @Test
  void fullStickGivesFullSpeedInEveryDirection() {
    // A square-gated stick reads 1.0 on both axes at full diagonal, so the raw
    // magnitude is 1.414 -- above full scale. It has to come out as full speed
    // in that direction, not as 1.414 times full speed and not as less than
    // full speed either.
    System.out.printf("%18s %10s %10s%n", "stick", "magnitude", "heading");
    double[][] directions = { { 0, 1 }, { 1, 0 }, { 1, 1 }, { -1, 1 }, { 0.707, 0.707 } };
    for (double[] d : directions) {
      var velocity = Controls.getLinearVelocityFromJoysticks(d[0], d[1]);
      double magnitude = velocity.getNorm();
      System.out.printf("  (%5.2f, %5.2f) %13.3f %9.1f deg%n",
          d[0], d[1], magnitude, velocity.getAngle().getDegrees());
      assertEquals(1.0, magnitude, 0.02,
          String.format("full stick at (%.2f, %.2f) should give full speed", d[0], d[1]));
      assertEquals(Math.toDegrees(Math.atan2(d[1], d[0])), velocity.getAngle().getDegrees(), 0.5,
          "the curve must not bend the direction the robot travels");
    }
  }

  @Test
  void halfStickOnTheDiagonalIsStillCurved() {
    // The magnitude is what gets shaped, so a half-deflected diagonal should be
    // softened by the same amount a half-deflected straight push is.
    double straight = Controls.getLinearVelocityFromJoysticks(0.0, 0.5).getNorm();
    double diagonal = Controls.getLinearVelocityFromJoysticks(0.354, 0.354).getNorm();
    assertEquals(straight, diagonal, 0.02,
        "the same stick distance should give the same speed whichever way it is pushed");
  }

  @Test
  void rotationIsTheMoreLinearOfTheTwo() {
    // At the driver's request. Rotation used to be hard-coded as omega squared,
    // which on a gamepad was far too soft at the bottom of the stick -- the
    // travel is short enough that fine control is already there without help.
    double rotation = Controls.applyRotationCurve(0.5);
    double translation = Controls.applyCurve(0.5, Controls.TRANSLATION_CURVE);
    assertTrue(rotation > translation,
        "rotation should be the more linear of the two, got " + rotation + " vs " + translation);

    double squared = 0.25;
    assertTrue(rotation > squared + 0.1,
        "rotation should be well clear of the squaring it replaced, got " + rotation);
  }

  @Test
  void neitherCurveIsSoSoftThatHalfStickCrawls() {
    // The failure the driver reported: half a stick asking for a quarter of the
    // speed feels broken on a gamepad, whatever it did on a flight stick.
    for (double c : new double[] { Controls.TRANSLATION_CURVE, Controls.ROTATION_CURVE }) {
      double half = Controls.applyCurve(0.5, c);
      assertTrue(half > 0.35,
          "half stick should ask for more than a third of full speed, got " + half
              + " at curve " + c);
    }
  }
}

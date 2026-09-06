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
  void theCurveIsMonotonic() {
    // A curve that dips would mean pushing the stick further sometimes slows the
    // robot down, which is far worse than any amount of twitchiness.
    for (double curve : new double[] { 0.0, 0.5, 0.7, 1.0 }) {
      double previous = -1.1;
      for (double input = -1.0; input <= 1.0; input += 0.01) {
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
  void rotationStaysCloseToTheSquaringItReplaced() {
    // Rotation used to be hard-coded as omega squared in both drive commands.
    // Keeping the new curve near that means the driver should not notice a
    // change in the one axis they were already used to.
    for (double input = 0.1; input <= 1.0; input += 0.1) {
      double squared = input * input;
      double curved = Controls.applyRotationCurve(input);
      assertEquals(squared, curved, 0.08,
          String.format("rotation feel changed at %.1f stick: was %.3f, now %.3f",
              input, squared, curved));
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The cases a tilted shot has to get right, in the numbers the bump produces.
 *
 * <p>
 * Round trips are the backbone here: solving for what to command and then asking
 * what that command actually does must land back where it started. That catches
 * a sign or an inverted rotation, which is the way this goes wrong, and it does
 * so without anybody having to work out the right answer by hand.
 */
public class TiltedShotTest {
  private static final double HOOD_MIN = 58.0;
  private static final double HOOD_MAX = 70.0;
  private static final double LAUNCH_H = 0.491;
  private static final double HUB_H = 72 * 0.0254;

  /** Gravity as the gyro reports it: robot-frame "up", tilted by the ramp. */
  private static Translation3d tilted(double degrees, double towardAzimuthRadians) {
    double t = Units.degreesToRadians(degrees);
    return new Translation3d(
        Math.sin(t) * Math.cos(towardAzimuthRadians),
        Math.sin(t) * Math.sin(towardAzimuthRadians),
        Math.cos(t));
  }

  @Test
  void levelChangesNothing() {
    var flat = new Translation3d(0, 0, 1);
    var s = TiltedShot.solve(flat, 0.4, 64.0, HOOD_MIN, HOOD_MAX);
    assertEquals(0.4, s.turretAzimuthRadians(), 1e-9);
    assertEquals(64.0, s.hoodElevationDegrees(), 1e-9);
    assertEquals(64.0, s.launchElevationDegrees(), 1e-9);
    assertFalse(s.hoodWasClamped());
  }

  @Test
  void tiltingBackwardsSteepensTheShot() {
    // The robot's up leans toward where it is shooting, which tips the barrel
    // up. This is the bump case and the whole reason for the class.
    var g = tilted(9.2, 0.0);
    var actual = TiltedShot.forward(g, 0.0, 64.0);
    assertTrue(actual.launchElevationDegrees() > 70.0,
        "leaning back should steepen a 64 degree shot, got "
            + actual.launchElevationDegrees());
    assertEquals(73.2, actual.launchElevationDegrees(), 0.3);
  }

  @Test
  void solvingThenFiringLandsWhereItWasAimed() {
    // The round trip. If the rotation is inverted anywhere, this fails.
    for (double tilt : new double[] { 0, 3, 6, 9.2 }) {
      for (double toward : new double[] { 0, 1.1, 2.6, -2.0 }) {
        var g = tilted(tilt, toward);
        // Modest elevation so the hood has room and nothing is clamped.
        var s = TiltedShot.solve(g, 0.3, 64.0, 40.0, 85.0);
        var got = TiltedShot.forward(g, s.turretAzimuthRadians(), s.hoodElevationDegrees());
        assertEquals(0.3, got.turretAzimuthRadians(), 1e-6,
            "bearing should survive a " + tilt + " degree tilt toward " + toward);
        assertEquals(64.0, got.launchElevationDegrees(), 1e-6,
            "elevation should survive a " + tilt + " degree tilt toward " + toward);
      }
    }
  }

  @Test
  void theHoodRunsOutAndSaysSo() {
    // 9.2 degrees of tilt asks a 64 degree shot for 54.8 from the hood, and the
    // hood stops at 58. It has to report that rather than pretend.
    var g = tilted(9.2, 0.0);
    var s = TiltedShot.solve(g, 0.0, 64.0, HOOD_MIN, HOOD_MAX);
    assertTrue(s.hoodWasClamped(), "the hood cannot reach this and should say so");
    assertEquals(HOOD_MIN, s.hoodElevationDegrees(), 1e-9);
    assertTrue(s.launchElevationDegrees() > 64.0,
        "with the hood at its stop the shot still leaves steep, got "
            + s.launchElevationDegrees());
    System.out.printf("9.2 deg tilt: hood pinned at %.1f, ball leaves at %.1f%n",
        s.hoodElevationDegrees(), s.launchElevationDegrees());
  }

  @Test
  void theFlywheelCoversWhatTheHoodCannot() {
    double gain = HUB_H - LAUNCH_H;
    double level = TiltedShot.requiredSpeed(3.0, 64.0, gain);
    var g = tilted(9.2, 0.0);
    var s = TiltedShot.solve(g, 0.0, 64.0, HOOD_MIN, HOOD_MAX);
    double tiltedSpeed = TiltedShot.requiredSpeed(3.0, s.launchElevationDegrees(), gain);
    System.out.printf("3 m shot: %.2f m/s level, %.2f m/s at %.1f deg (%+.0f%%)%n",
        level, tiltedSpeed, s.launchElevationDegrees(), 100 * (tiltedSpeed / level - 1));
    assertTrue(tiltedSpeed > level, "a steeper shot needs more speed");
    assertTrue(tiltedSpeed / level < 1.35,
        "and not so much more that the wheel cannot deliver it, wanted "
            + (tiltedSpeed / level));
  }

  @Test
  void aShotTooFlatToClearTheHeightReturnsNaN() {
    // Twenty degrees does not get 1.34 m up in three metres at any speed, and
    // that is a real refusal rather than a large number.
    assertTrue(Double.isNaN(TiltedShot.requiredSpeed(3.0, 20.0, 1.34)),
        "too flat to gain the height, must not pretend otherwise");
    assertTrue(Double.isNaN(TiltedShot.requiredSpeed(0.0, 64.0, 1.34)),
        "no distance is not a shot");
  }

  @Test
  void aNearVerticalShotIsPossibleButAbsurd() {
    // Worth pinning, because the first version of this test assumed such a shot
    // was impossible and it is not: lob it hard enough and it comes down three
    // metres away. The formula is right and the answer is simply unusable, so
    // the caller must reject it on the speed rather than expect a refusal.
    double v = TiltedShot.requiredSpeed(3.0, 89.9, 1.34);
    assertFalse(Double.isNaN(v), "geometrically this does reach");
    assertTrue(v > 50, "but only at a ludicrous speed, got " + v);
    System.out.printf("89.9 deg reaches 3 m, at %.0f m/s%n", v);
  }

  @Test
  void theSpeedItAsksForActuallyGetsThere() {
    // Fly the ball with the speed the formula gives and check it arrives.
    double gain = HUB_H - LAUNCH_H;
    for (double d : new double[] { 2.0, 3.0, 5.0, 7.0 }) {
      for (double elev : new double[] { 58.0, 64.0, 70.0, 75.0 }) {
        double v = TiltedShot.requiredSpeed(d, elev, gain);
        if (Double.isNaN(v)) {
          continue;
        }
        double th = Units.degreesToRadians(elev);
        double vz = v * Math.sin(th);
        double vx = v * Math.cos(th);
        // time to fall back through the target height
        double disc = vz * vz + 2 * 9.81 * -gain;
        double t = (vz + Math.sqrt(disc)) / 9.81;
        assertEquals(d, vx * t, 1e-6,
            "a ball fired at " + v + " m/s and " + elev + " deg should reach " + d);
      }
    }
  }

  @Test
  void tiltingSidewaysSwingsTheBearing() {
    // The sideways offset table was standing in for this. Tilted across the
    // shot, the bearing moves as well as the elevation, and one rotation does
    // both.
    var g = tilted(9.2, Math.PI / 2);
    var got = TiltedShot.forward(g, 0.0, 64.0);
    assertTrue(Math.abs(got.turretAzimuthRadians()) > Units.degreesToRadians(2.0),
        "a sideways tilt should move the bearing, moved "
            + Units.radiansToDegrees(got.turretAzimuthRadians()) + " deg");
    System.out.printf("tilted 9.2 deg sideways: bearing moves %.1f deg%n",
        Units.radiansToDegrees(got.turretAzimuthRadians()));
  }
}

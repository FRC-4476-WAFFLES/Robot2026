// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.utils.lib.TiltedShot;

/**
 * The simulated gyro and the tilt solver have to describe the same world.
 *
 * <p>
 * A simulation is only worth what it agrees with, and here it is being asked to
 * judge a correction to the very thing it models. If the gravity vector it
 * reports and the rotation it applies to a shot disagree, then a fix that is
 * right will look wrong, or worse a fix that is wrong will look right — and
 * nothing about the result would mean anything.
 *
 * <p>
 * These check the two against each other rather than against a number written
 * out by hand, because the failure being guarded against is a sign or an
 * inverted rotation, and both sides being wrong the same way is exactly what a
 * hand-picked expected value would miss.
 */
public class SimTiltTest {
  private static final double HOOD_MIN = 58.0;
  private static final double HOOD_MAX = 70.0;

  @AfterEach
  void putItBack() {
    GyroIOSim.reset();
  }

  @Test
  void levelGravityIsStraightUp() {
    GyroIOSim.reset();
    var n = GyroIOSim.getSurfaceNormal();
    assertEquals(0.0, n.getX(), 1e-9);
    assertEquals(0.0, n.getY(), 1e-9);
    assertEquals(1.0, n.getZ(), 1e-9);
    assertTrue(GyroIOSim.getSurfaceLean().getAngle() < 1e-9,
        "a level robot needs no rotation at all");
  }

  @Test
  void theNormalLeansTheWayTheSlopeSaysItDoes() {
    // Climbing the near face the normal leans back toward where the robot came
    // from, and past the crest it leans on ahead. Getting this backwards flips
    // every shot correction on the bump.
    GyroIOSim.setTilt(10.0, Math.PI);
    assertTrue(GyroIOSim.getSurfaceNormal().getX() < 0,
        "on the near face the normal should lean back along -X");
    GyroIOSim.setTilt(10.0, 0.0);
    assertTrue(GyroIOSim.getSurfaceNormal().getX() > 0,
        "past the crest it should lean on along +X");
  }

  @Test
  void tiltMagnitudeSurvivesTheRoundTrip() {
    for (double tilt : new double[] { 0, 4, 9.2, 16 }) {
      for (double azimuth : new double[] { 0, Math.PI, 1.3 }) {
        GyroIOSim.setTilt(tilt, azimuth);
        var n = GyroIOSim.getSurfaceNormal();
        assertEquals(1.0, n.getNorm(), 1e-9, "the normal must stay a unit vector");
        assertEquals(tilt, Units.radiansToDegrees(Math.acos(n.getZ())), 1e-6,
            "the tilt read back out of the normal must be the tilt put in");
      }
    }
  }

  /**
   * The one that matters: what the simulation does to a shot, and what the
   * solver believes it does, have to be the same thing.
   *
   * <p>
   * The simulation builds the launch direction in the robot's frame and rotates
   * it out by the surface lean. The solver takes the gravity vector and rotates
   * the other way. Composed, they must return what they started with.
   */
  @Test
  void theSolverInvertsWhatTheSimulationApplies() {
    for (double tilt : new double[] { 0, 5, 9.2, 16 }) {
      for (double azimuth : new double[] { 0, Math.PI, 2.0 }) {
        GyroIOSim.setTilt(tilt, azimuth);
        // Gravity as the gyro reports it for a robot facing along +X, which is
        // the frame the solver works in.
        Translation3d gravity = new Translation3d(0, 0, 1)
            .rotateBy(GyroIOSim.getSurfaceLean().unaryMinus());

        double wantAzimuth = 0.35;
        double wantElevation = 64.0;
        // Wide hood limits so nothing clamps and the geometry is tested alone.
        var solved = TiltedShot.solve(gravity, wantAzimuth, wantElevation, 20.0, 89.0);
        var got = TiltedShot.forward(gravity, solved.turretAzimuthRadians(),
            solved.hoodElevationDegrees());

        assertEquals(wantAzimuth, got.turretAzimuthRadians(), 1e-6,
            "bearing lost at tilt " + tilt + " toward " + azimuth);
        assertEquals(wantElevation, got.launchElevationDegrees(), 1e-6,
            "elevation lost at tilt " + tilt + " toward " + azimuth);
      }
    }
  }

  @Test
  void theBumpCostsWhatWasMeasuredOffTheRealLogs() {
    // 9.2 degrees is the median tilt while shooting from the bump in the
    // simulated logs, and the shot leaving nine degrees steeper than intended
    // is the whole complaint. Pinned so a change to the tilt model that
    // quietly removes the problem gets noticed.
    GyroIOSim.setTilt(9.2, Math.PI);
    Translation3d gravity = new Translation3d(0, 0, 1)
        .rotateBy(GyroIOSim.getSurfaceLean().unaryMinus());

    var uncorrected = TiltedShot.forward(gravity, 0.0, 64.0);
    System.out.printf("uncorrected on the bump: asked 64.0, leaves at %.1f deg%n",
        uncorrected.launchElevationDegrees());
    assertTrue(Math.abs(uncorrected.launchElevationDegrees() - 64.0) > 5.0,
        "the bump must still cost something, or there is nothing to fix");

    var corrected = TiltedShot.solve(gravity, 0.0, 64.0, HOOD_MIN, HOOD_MAX);
    assertTrue(corrected.hoodWasClamped(),
        "the hood cannot cover nine degrees and should say so");
    double remaining = Math.abs(corrected.launchElevationDegrees() - 64.0);
    System.out.printf("with the hood at its stop: leaves at %.1f deg, %.1f left "
        + "for the flywheel%n", corrected.launchElevationDegrees(), remaining);
    assertTrue(remaining < Math.abs(uncorrected.launchElevationDegrees() - 64.0),
        "the hood should at least reduce it");
  }
}

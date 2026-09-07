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
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.data.FieldConstants;
import frc.robot.utils.sim.SimField;
import frc.robot.utils.sim.SimShooter;

/**
 * Checks the two things added for watching the simulation rather than reading
 * it: that shooting actually puts balls in the air, and that the field has walls
 * and a bump.
 */
public class SimShooterTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
    SimHarness.enableTeleop();
  }

  @AfterAll
  static void putItBack() {
    SimShooter.setEnabled(false);
    SimField.setEnabled(false);
    SimHarness.levelOut();
  }

  @Test
  void shootingPutsBallsInTheAir() {
    SimShooter.setEnabled(true);
    // Somewhere it would actually shoot from.
    RobotContainer.drive.setPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));
    SimHarness.stepSeconds(0.5);

    int before = SimShooter.getShotsFired();
    RobotContainer.state.setShooting(true);
    RobotContainer.flywheel.runSetpoint(50);
    SimHarness.stepSeconds(2.0);
    RobotContainer.state.setShooting(false);
    SimShooter.setEnabled(false);
    SimHarness.stepSeconds(0.3);

    int after = SimShooter.getShotsFired();
    System.out.printf("shots fired: %d -> %d%n", before, after);
    assertTrue(after > before, "shooting for two seconds should have fired at least one ball");
  }

  @Test
  void theFieldHasWalls() {
    SimField.setEnabled(true);
    RobotContainer.simState.resetSlipTracking();
    // Start close to a wall and drive at it, so it is actually reached.
    RobotContainer.drive.setPose(new Pose2d(1.2, 4.0, Rotation2d.kZero));
    SimHarness.step(5);

    // Stick forward is +X, so this drives back towards the near wall.
    driveForward(1.0, 2.0);

    // The truth pose is what the wall holds. Odometry keeps integrating the
    // wheels, which is the point -- a robot pressed into a wall loses its pose.
    Pose2d truth = RobotContainer.simState.getPose();
    double odometryX = RobotContainer.state.getPose().getX();
    System.out.printf("drove into the wall: truth x %.2f, odometry x %.2f%n", truth.getX(), odometryX);
    assertTrue(truth.getX() >= 0.4,
        "the wall should have held the robot on the field, truth pose was " + truth);

    SimField.setEnabled(false);
  }

  @Test
  void theBumpTiltsTheRobot() {
    // The tilt several behaviours key off, which no simulation could produce
    // before: TARGET_TAG while crossing, the turret's tilt offset, and arming
    // the post-crossing vision recovery.
    //
    // Swept rather than aimed at one coordinate. The bump sits at a
    // blue-relative X and the simulator defaults to red, so a fixed coordinate
    // means one place or another depending on how promptly the alliance
    // propagates -- which made an earlier version of this test flaky rather
    // than wrong.
    SimField.setEnabled(true);
    double tiltedAt = Double.NaN;
    for (double x = 1.0; x < FieldConstants.fieldLength - 1.0; x += 0.25) {
      RobotContainer.drive.setPose(new Pose2d(x, 4.0, Rotation2d.kZero));
      SimHarness.step(3);
      if (!RobotContainer.drive.isLevelOnGround()) {
        tiltedAt = x;
        break;
      }
    }
    System.out.printf("robot first read as tilted at x = %.2f m%n", tiltedAt);
    assertTrue(!Double.isNaN(tiltedAt),
        "driving the length of the field should have crossed the bump and tilted the robot");

    // And it must be level again on the far side, or every behaviour gated on
    // being level would stay off for the rest of the match.
    RobotContainer.drive.setPose(new Pose2d(tiltedAt + 3.0, 4.0, Rotation2d.kZero));
    SimHarness.step(3);
    assertTrue(RobotContainer.drive.isLevelOnGround(),
        "the robot should be level again well past the bump");

    SimField.setEnabled(false);
    SimHarness.levelOut();
  }

  @Test
  void crossingTheBumpCostsOdometryAccuracy() {
    // The failure the drive team actually reports, and one no simulation could
    // produce before: both pose estimators were fed identical module positions,
    // so the truth pose and odometry could never disagree.
    //
    // The peak error during the crossing is what matters, not the error after
    // it. Simulated vision sees the truth pose and corrects odometry back
    // towards it, which is exactly what the real robot does -- the drift that
    // hurts is the drift while it is happening, and while a real robot may have
    // no tag in view to fix it.
    SimField.setEnabled(true);
    RobotContainer.simState.resetSlipTracking();
    RobotContainer.drive.setPose(new Pose2d(2.5, 4.0, Rotation2d.kZero));
    SimHarness.step(5);
    double baseline = odometryError();

    double peak = 0;
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, -1.0);
    for (int i = 0; i < 150; i++) {
      SimHarness.step(1);
      peak = Math.max(peak, odometryError());
    }
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.0);
    SimHarness.stepSeconds(0.3);

    System.out.printf("odometry error: %.3f m at rest, %.3f m peak while crossing%n", baseline, peak);
    SimField.setEnabled(false);
    SimHarness.levelOut();

    assertTrue(peak > baseline + 0.05,
        "crossing the bump should cost real odometry accuracy, peak was " + peak
            + " against a baseline of " + baseline);
  }

  /** Holds the drive stick for a while, so the robot moves under its own power. */
  private static void driveForward(double stick, double seconds) {
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, stick);
    SimHarness.stepSeconds(seconds);
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.0);
    SimHarness.stepSeconds(0.4);
  }

  /** How far odometry has drifted from where the robot actually is. */
  private static double odometryError() {
    return RobotContainer.simState.getPose().getTranslation()
        .getDistance(RobotContainer.state.getPose().getTranslation());
  }
}

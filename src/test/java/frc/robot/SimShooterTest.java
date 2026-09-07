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
    // Drop the robot well outside the field and let the walls put it back.
    RobotContainer.drive.setPose(new Pose2d(-5.0, -5.0, Rotation2d.kZero));
    SimHarness.stepSeconds(0.3);

    Pose2d pose = RobotContainer.state.getPose();
    System.out.printf("placed at (-5.0, -5.0), ended at (%.2f, %.2f)%n", pose.getX(), pose.getY());
    assertTrue(pose.getX() > 0 && pose.getY() > 0,
        "the walls should have pushed the robot back onto the field, got " + pose);
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
}

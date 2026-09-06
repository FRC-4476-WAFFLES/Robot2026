// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.SimHarness;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotState.ShooterState;
import frc.robot.data.Constants.VisionConstants;

/**
 * Drives automatic pose recovery on a booted robot.
 *
 * <p>
 * This is the feature most worth testing hard, because it takes the turret away
 * from the driver on its own initiative. The failures that matter are not "it
 * fails to recover" — they are "it fires when it should not" and "it does not
 * let go". Both are covered here.
 *
 * <p>
 * Agreement is driven directly rather than through simulated vision, which
 * produces its own estimates and cannot be made to disagree on demand.
 */
public class PoseRecoverySimTest {
  private static final Pose2d ROBOT_POSE = new Pose2d(3.5, 4.0, Rotation2d.kZero);
  private static final int SHOOT_TRIGGER = XboxController.Axis.kRightTrigger.value;

  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  @BeforeEach
  void startFromAKnownPlace() {
    SimHarness.enableTeleop();
    SimHarness.releaseAllControls();
    RobotContainer.drive.setPose(ROBOT_POSE);
    agree();
    SimHarness.step(2);
  }

  /** An estimate that lands on top of odometry, so the pose is trusted. */
  private static void agree() {
    RobotContainer.vision.updatePoseAgreement(ROBOT_POSE);
  }

  /** An estimate a long way from odometry, held long enough to be believed. */
  private static void loseThePose() throws InterruptedException {
    Pose2d wrong = new Pose2d(ROBOT_POSE.getX() + VisionConstants.POSE_LOST_DISTANCE * 4,
        ROBOT_POSE.getY(), Rotation2d.kZero);
    RobotContainer.vision.updatePoseAgreement(wrong);
    Thread.sleep((long) (VisionConstants.POSE_LOST_TIME * 1000) + 200);
    RobotContainer.vision.updatePoseAgreement(wrong);
    SimHarness.step(3);
  }

  @Test
  void aTrustedPoseNeverTriggersRecovery() {
    // The failure that would ruin a match: the turret wandering off during
    // normal play. Replaying the logs, the shipped isPoseStable would have done
    // exactly this for most of a match.
    for (int i = 0; i < 10; i++) {
      agree();
      SimHarness.step(2);
      assertFalse(RobotContainer.vision.poseLikelyLost(), "pose should be trusted");
      assertNotEquals(ShooterState.RECOVER_POSE, RobotContainer.state.shooterState,
          "recovery must not run while vision and odometry agree");
    }
  }

  @Test
  void aLostPoseTakesTheTurret() throws InterruptedException {
    loseThePose();
    assertTrue(RobotContainer.vision.poseLikelyLost(), "setup: the pose should read as lost");
    assertEquals(ShooterState.RECOVER_POSE, RobotContainer.state.shooterState,
        "a lost pose with no shot requested should hand the turret to recovery");
  }

  @Test
  void askingToShootOutranksRecovery() throws InterruptedException {
    // The driver may be able to see the goal perfectly well. Taking the turret
    // off them mid-cycle is worse than a pose that is a metre out.
    loseThePose();
    assertEquals(ShooterState.RECOVER_POSE, RobotContainer.state.shooterState, "setup");

    SimHarness.setAxis(SimHarness.DRIVER, SHOOT_TRIGGER, 1.0);
    SimHarness.step(3);
    assertNotEquals(ShooterState.RECOVER_POSE, RobotContainer.state.shooterState,
        "holding the shoot trigger must take the turret back");

    SimHarness.setAxis(SimHarness.DRIVER, SHOOT_TRIGGER, 0.0);
    SimHarness.step(3);
  }

  @Test
  void anAgreeingEstimateHandsTheTurretBack() throws InterruptedException {
    loseThePose();
    assertEquals(ShooterState.RECOVER_POSE, RobotContainer.state.shooterState, "setup");

    agree();
    SimHarness.step(3);
    assertFalse(RobotContainer.vision.poseLikelyLost(), "one agreeing estimate should clear it");
    assertNotEquals(ShooterState.RECOVER_POSE, RobotContainer.state.shooterState,
        "recovery must let go the moment the robot knows where it is");
  }

  @Test
  void recoveryDoesNotSpinTheFlywheel() throws InterruptedException {
    // The turret is being used as a camera mount, not a gun. A flywheel spinning
    // up during recovery would be both a surprise and a waste of battery at
    // exactly the wrong moment.
    loseThePose();
    SimHarness.stepSeconds(0.5);
    assertEquals(0.0, RobotContainer.flywheel.getGoalVelocity(), 1e-6,
        "recovery must not command the flywheel");
  }

  @Test
  void recoveryAimsBeforeItSweeps() throws InterruptedException {
    // Aiming at the nearest tag finds it far faster than sweeping, and a lost
    // pose is usually wrong by a metre or two rather than half a field.
    loseThePose();
    SimHarness.stepSeconds(0.4);
    var aimed = RobotContainer.turret.getGoalHeading();

    SimHarness.stepSeconds(0.4);
    var stillAimed = RobotContainer.turret.getGoalHeading();
    assertEquals(aimed.getDegrees(), stillAimed.getDegrees(), 15.0,
        "the turret should hold a steady aim before the sweep starts, not wander");
  }

  @Test
  void theTurretStaysInsideItsTravelWhileSweeping() throws InterruptedException {
    // A sweep that commands past the hardstops would grind the turret against
    // them for as long as the pose stays lost.
    loseThePose();
    for (int i = 0; i < 40; i++) {
      SimHarness.stepSeconds(0.15);
      double degrees = RobotContainer.turret.getGoalHeading().getDegrees();
      assertTrue(Math.abs(degrees) <= 190.0,
          "recovery commanded the turret to " + degrees + " deg, outside its travel");
    }
  }
}

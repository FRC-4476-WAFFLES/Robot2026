// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.SimHarness;
import frc.robot.data.Constants.VisionConstants;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Drives the pose-agreement logic directly. Simulated vision cannot be used for
 * this: its estimates currently land metres away from the sim's own truth pose,
 * so nothing ever agrees and the counter would never leave zero.
 */
public class PoseAgreementTest {
  private static final Pose2d ROBOT_POSE = new Pose2d(3.5, 4.0, Rotation2d.kZero);

  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  /**
   * Every test starts flat and disabled. Without this a test that tilts the
   * robot or enables it leaks into the next one, and the vision loop keeps
   * feeding its own estimates into the very counter under test.
   */
  @BeforeEach
  void flatAndDisabled() {
    SimHarness.disable();
    GyroIOSim.reset();
    SimHarness.step(2);
  }

  /** Puts odometry at a known pose and clears any accumulated agreement. */
  private static Vision atKnownPose() {
    RobotContainer.drive.setPose(ROBOT_POSE);
    SimHarness.step(1);

    Vision vision = RobotContainer.vision;
    // A wildly wrong estimate resets the counter to a known zero.
    vision.updatePoseAgreement(ROBOT_POSE.plus(
        new edu.wpi.first.math.geometry.Transform2d(new Translation2d(50, 50), Rotation2d.kZero)));
    return vision;
  }

  @Test
  void agreeingEstimatesBuildConfidenceUntilTheThresholdIsReached() {
    Vision vision = atKnownPose();
    assertEquals(0, vision.getPoseStableUpdates());
    assertFalse(vision.isPoseStable());

    // Just inside the epsilon, so every one of these counts as agreement.
    Pose2d closeEnough = new Pose2d(
        ROBOT_POSE.getX() + VisionConstants.POSE_AGREEMENT_EPSILON * 0.5,
        ROBOT_POSE.getY(),
        Rotation2d.kZero);

    for (int i = 0; i < VisionConstants.POSE_STABLE_UPDATE_THRESHOLD - 1; i++) {
      vision.updatePoseAgreement(closeEnough);
    }
    assertFalse(vision.isPoseStable(), "one short of the threshold is not stable yet");

    vision.updatePoseAgreement(closeEnough);
    assertTrue(vision.isPoseStable(), "reaching the threshold should mark the pose stable");
    assertEquals(VisionConstants.POSE_STABLE_UPDATE_THRESHOLD, vision.getPoseStableUpdates());
  }

  @Test
  void oneDisagreeingEstimateResetsTheCount() {
    Vision vision = atKnownPose();

    for (int i = 0; i < 50; i++) {
      vision.updatePoseAgreement(ROBOT_POSE);
    }
    assertEquals(50, vision.getPoseStableUpdates());

    // Just outside the epsilon.
    Pose2d tooFar = new Pose2d(
        ROBOT_POSE.getX() + VisionConstants.POSE_AGREEMENT_EPSILON * 1.5,
        ROBOT_POSE.getY(),
        Rotation2d.kZero);
    vision.updatePoseAgreement(tooFar);

    assertEquals(0, vision.getPoseStableUpdates(), "a single disagreement should reset confidence");
    assertFalse(vision.isPoseStable());
  }

  @Test
  void confidenceGoesStaleWhenVisionStopsProducingEstimates() {
    Vision vision = atKnownPose();

    for (int i = 0; i < VisionConstants.POSE_STABLE_UPDATE_THRESHOLD; i++) {
      vision.updatePoseAgreement(ROBOT_POSE);
    }
    assertTrue(vision.isPoseStable());

    // No further estimates: after the stale window it must stop claiming stable
    // rather than latching on the old count.
    SimHarness.stepSeconds(VisionConstants.POSE_AGREEMENT_STALE_TIME + 0.3);
    assertFalse(vision.isPoseStable(), "stale agreement should not still read as stable");
  }

  /**
   * Holds a large disagreement for longer than the lost-pose timeout.
   *
   * <p>
   * Simulated vision derives its estimates from the drive's own pose, so it
   * agrees perfectly on every loop and clears the timer as fast as this could
   * set it. The clock is therefore advanced without running the robot, leaving
   * the test in sole control of what the logic sees.
   */
  private static void holdADisagreement(Vision vision) {
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 3));
    SimHarness.advanceClockOnly(VisionConstants.POSE_LOST_TIME + 0.3);
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 3));
  }

  /** An estimate a given distance from where odometry thinks the robot is. */
  private static Pose2d offsetBy(double metres) {
    return new Pose2d(ROBOT_POSE.getX() + metres, ROBOT_POSE.getY(), Rotation2d.kZero);
  }

  @Test
  void aSingleBadEstimateDoesNotMeanTheRobotIsLost() {
    // About 19 % of accepted estimates miss by more than 20 cm in normal
    // operation, so a detector that trips on one of them fires constantly. This
    // is the mistake that made isPoseStable unusable for anything automatic.
    Vision vision = atKnownPose();
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 3));
    assertFalse(vision.poseLikelyLost(),
        "one estimate a long way out must not be enough to declare the pose lost");
  }

  @Test
  void aDisagreementHeldPastTheTimeoutDoes() {
    Vision vision = atKnownPose();
    holdADisagreement(vision);
    assertTrue(vision.poseLikelyLost(),
        "an error past the threshold, held past the timeout, should declare the pose lost");
  }

  @Test
  void oneAgreeingEstimateClearsItImmediately() {
    // Recovery must not lag. Whatever ends up acting on this has to hand control
    // back the moment the robot knows where it is again.
    Vision vision = atKnownPose();
    holdADisagreement(vision);
    assertTrue(vision.poseLikelyLost(), "setup: should be lost first");

    vision.updatePoseAgreement(offsetBy(0.0));
    assertFalse(vision.poseLikelyLost(), "an agreeing estimate should clear it at once");
  }

  @Test
  void anErrorInsideTheThresholdNeverTripsIt() {
    // The everyday tail. The 90th percentile agreement error in the match logs
    // is 0.33 m, so errors of this size are normal and must never be reported.
    Vision vision = atKnownPose();
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 0.5));
    SimHarness.advanceClockOnly(VisionConstants.POSE_LOST_TIME + 0.3);
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 0.5));
    assertFalse(vision.poseLikelyLost(),
        "an error inside the threshold must never declare the pose lost, however long it lasts");
  }

  @Test
  void leavingTheBumpArmsTheHighTrustEstimates() {
    // Driven through the pose rather than by setting onBump directly, because
    // StateOrchestrator recomputes onBump from the pose every loop and would
    // overwrite anything set by hand.
    // The arming runs from a Trigger, and a scheduled command does nothing while
    // the robot is disabled.
    SimHarness.enableTeleop();
    Vision vision = RobotContainer.vision;

    // Inside the bump band: 4.0 to the near neutral zone line.
    RobotContainer.drive.setPose(new Pose2d(4.5, 4.0, Rotation2d.kZero));
    SimHarness.step(4);
    assertTrue(RobotContainer.state.onBump, "setup: this pose should read as on the bump");
    vision.highTrustEstimatesLeft = 0;

    // Back out of it.
    RobotContainer.drive.setPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
    SimHarness.step(4);
    assertFalse(RobotContainer.state.onBump, "setup: this pose should read as off the bump");
    assertTrue(vision.highTrustEstimatesLeft > 0,
        "leaving the bump should trust the next few vision estimates far more than usual");
    SimHarness.disable();
  }

  @Test
  void aPartialCrossingStillArmsTheRecovery() {
    // The case the second trigger exists for, and the one that used to be
    // untestable. The robot tilts onto the bump and comes back down without the
    // pose ever leaving the bump band, so onBump stays true throughout and the
    // position test never notices the crossing ended. Replaying the match logs,
    // the robot became level 330 times and onBump was true on every one.
    SimHarness.enableTeleop();
    Vision vision = RobotContainer.vision;

    RobotContainer.drive.setPose(new Pose2d(4.5, 4.0, Rotation2d.kZero));
    SimHarness.tiltOntoBump();
    assertFalse(RobotContainer.drive.isLevelOnGround(), "setup: the robot should read as tilted");
    assertTrue(RobotContainer.state.onBump, "setup: and as on the bump");
    vision.highTrustEstimatesLeft = 0;

    // Back down, but still inside the bump band by position.
    SimHarness.levelOut();
    assertTrue(RobotContainer.drive.isLevelOnGround(), "the robot should be level again");
    assertTrue(RobotContainer.state.onBump,
        "the position test should still think we are on the bump, which is the whole point");
    assertTrue(vision.highTrustEstimatesLeft > 0,
        "becoming level should arm the recovery even though the position test has not noticed");

    SimHarness.levelOut();
    SimHarness.disable();
  }

  @Test
  void tiltAloneIsEnoughToCountAsOnTheBump() {
    // determineOnBump falls back to the gyro when the pose says otherwise, and
    // that fallback had never been exercised.
    SimHarness.enableTeleop();
    RobotContainer.drive.setPose(new Pose2d(1.0, 4.0, Rotation2d.kZero));
    SimHarness.step(3);
    assertFalse(RobotContainer.state.onBump, "setup: well clear of the bump band");

    SimHarness.tiltOntoBump();
    SimHarness.step(2);
    assertTrue(RobotContainer.state.onBump,
        "a tilted robot should read as on the bump wherever the pose says it is");

    SimHarness.levelOut();
    SimHarness.step(2);
    assertFalse(RobotContainer.state.onBump, "and level again once it is flat");
    SimHarness.disable();
  }
}

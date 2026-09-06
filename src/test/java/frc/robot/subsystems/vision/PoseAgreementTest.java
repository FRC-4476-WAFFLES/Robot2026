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
import frc.robot.SimHarness;
import frc.robot.data.Constants.VisionConstants;

import org.junit.jupiter.api.BeforeAll;
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
  void aDisagreementHeldPastTheTimeoutDoes() throws InterruptedException {
    Vision vision = atKnownPose();
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 3));
    Thread.sleep((long) (VisionConstants.POSE_LOST_TIME * 1000) + 200);
    SimHarness.step(1);
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 3));
    assertTrue(vision.poseLikelyLost(),
        "an error past the threshold, held past the timeout, should declare the pose lost");
  }

  @Test
  void oneAgreeingEstimateClearsItImmediately() throws InterruptedException {
    // Recovery must not lag. Whatever ends up acting on this has to hand control
    // back the moment the robot knows where it is again.
    Vision vision = atKnownPose();
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 3));
    Thread.sleep((long) (VisionConstants.POSE_LOST_TIME * 1000) + 200);
    SimHarness.step(1);
    vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 3));
    assertTrue(vision.poseLikelyLost(), "setup: should be lost first");

    vision.updatePoseAgreement(offsetBy(0.0));
    assertFalse(vision.poseLikelyLost(), "an agreeing estimate should clear it at once");
  }

  @Test
  void anErrorInsideTheThresholdNeverTripsIt() throws InterruptedException {
    // The everyday tail. The 90th percentile agreement error in the match logs
    // is 0.33 m, so errors of this size are normal and must never be reported.
    Vision vision = atKnownPose();
    for (int i = 0; i < 5; i++) {
      vision.updatePoseAgreement(offsetBy(VisionConstants.POSE_LOST_DISTANCE * 0.5));
      Thread.sleep(100);
    }
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
  void thePartialCrossingCaseIsNotCoveredBySimulation() {
    // Recorded so nobody assumes it is tested. The second arming trigger fires
    // when the Pigeon's gravity vector says the robot has become level again,
    // which in the match logs happened 330 times while the position test still
    // said "on bump" -- the case where a partial crossing would otherwise never
    // arm the recovery at all.
    //
    // GyroIOSim has no tilt: isLevelOnGround is true from boot and never
    // changes, so the edge that trigger watches for cannot be produced here.
    // Verifying it needs a robot, or a gyro sim that can be tilted.
    assertTrue(RobotContainer.drive.isLevelOnGround(),
        "the simulated gyro is always level, which is why this case cannot be tested in sim");
  }
}

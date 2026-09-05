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
}

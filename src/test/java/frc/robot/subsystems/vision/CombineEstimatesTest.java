// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.data.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision.TagPoseEstimate;

import org.junit.jupiter.api.Test;

/**
 * Covers issue #12: when two camera estimates are fused and only one has a
 * trustworthy yaw, the returned heading and its standard deviation must both
 * come from that one. Each case runs with the trusted estimate newer and older,
 * and with the arguments in both orders, since the fusion reorders by time.
 */
public class CombineEstimatesTest {
  private static final double TRUSTED_YAW_STD = 0.1;
  private static final double EPSILON = 1e-9;

  private static final Pose2d ODOMETRY = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45));

  private static TagPoseEstimate estimate(double headingDegrees, double timestamp, boolean trustYaw) {
    Matrix<N3, N1> stdDevs = VecBuilder.fill(0.05, 0.05,
        trustYaw ? TRUSTED_YAW_STD : VisionConstants.LARGE_VARIANCE);
    return new TagPoseEstimate(
        new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(headingDegrees)),
        timestamp, stdDevs, 1, ODOMETRY);
  }

  /** Fuses with the arguments in both orders and checks they agree. */
  private static TagPoseEstimate fuse(TagPoseEstimate x, TagPoseEstimate y) {
    TagPoseEstimate forward = Vision.combineEstimates(x, y).orElseThrow();
    TagPoseEstimate reversed = Vision.combineEstimates(y, x).orElseThrow();
    assertEquals(forward.pose().getRotation().getDegrees(), reversed.pose().getRotation().getDegrees(), EPSILON,
        "argument order should not change the heading");
    assertEquals(forward.standardDeviation().get(2, 0), reversed.standardDeviation().get(2, 0), EPSILON,
        "argument order should not change the yaw standard deviation");
    return forward;
  }

  @Test
  void bothTrustedWeightsTheHeadings() {
    for (boolean firstIsNewer : new boolean[] { true, false }) {
      var fused = fuse(estimate(80, firstIsNewer ? 2.0 : 1.0, true),
          estimate(100, firstIsNewer ? 1.0 : 2.0, true));

      assertEquals(90, fused.pose().getRotation().getDegrees(), 1e-6);
      assertEquals(TRUSTED_YAW_STD / Math.sqrt(2), fused.standardDeviation().get(2, 0), EPSILON);
    }
  }

  @Test
  void onlyTheTrustedHeadingIsReturnedWhenItIsNewer() {
    // The case from the issue: newer trusted, older untrusted used to return the
    // older heading with the newer one's confidence.
    var fused = fuse(estimate(90, 2.0, true), estimate(0, 1.0, false));

    assertEquals(90, fused.pose().getRotation().getDegrees(), EPSILON);
    assertEquals(TRUSTED_YAW_STD, fused.standardDeviation().get(2, 0), EPSILON);
  }

  @Test
  void onlyTheTrustedHeadingIsReturnedWhenItIsOlder() {
    var fused = fuse(estimate(90, 1.0, true), estimate(0, 2.0, false));

    assertEquals(90, fused.pose().getRotation().getDegrees(), EPSILON);
    assertEquals(TRUSTED_YAW_STD, fused.standardDeviation().get(2, 0), EPSILON);
  }

  @Test
  void neitherTrustedKeepsOdometryHeadingAndSaysSo() {
    for (boolean firstIsNewer : new boolean[] { true, false }) {
      var fused = fuse(estimate(10, firstIsNewer ? 2.0 : 1.0, false),
          estimate(20, firstIsNewer ? 1.0 : 2.0, false));

      assertEquals(ODOMETRY.getRotation().getDegrees(), fused.pose().getRotation().getDegrees(), EPSILON);
      assertEquals(VisionConstants.LARGE_VARIANCE, fused.standardDeviation().get(2, 0), EPSILON);
    }
  }

  @Test
  void anOlderTrustedHeadingIsCarriedForwardByOdometry() {
    // Odometry turned 10 degrees between the two frames, so the older trusted
    // heading should come out 10 degrees further round.
    var older = new TagPoseEstimate(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90)), 1.0,
        VecBuilder.fill(0.05, 0.05, TRUSTED_YAW_STD), 1, ODOMETRY);
    var newer = new TagPoseEstimate(new Pose2d(3.0, 4.0, Rotation2d.kZero), 2.0,
        VecBuilder.fill(0.05, 0.05, VisionConstants.LARGE_VARIANCE), 1,
        ODOMETRY.rotateAround(ODOMETRY.getTranslation(), Rotation2d.fromDegrees(10)));

    var fused = fuse(older, newer);

    assertEquals(100, fused.pose().getRotation().getDegrees(), 1e-6);
    assertEquals(TRUSTED_YAW_STD, fused.standardDeviation().get(2, 0), EPSILON);
  }
}

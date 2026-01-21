// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;
import frc.robot.utils.vision.VisionHelpers;

public class Vision extends VirtualSubsystem {
  public record TagPoseEstimate(
      Pose2d pose,
      double timestampSeconds,
      Matrix<N3, N1> standardDeviation,
      int numTags,
      Pose2d odometryAtTimestamp
  ) {}

  /** Networktables */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable softwareTable = inst.getTable("SoftwareInfo");
  private final StructPublisher<Pose2d> validPoseNT = softwareTable
      .getStructTopic("Validated Pose", Pose2d.struct)
      .publish();

  /** Limelight hardware */
  public final TagCamera leftLimelight;
  public final TagCamera rightLimelight;

  public Vision(VisionIO leftIO, VisionIO rightIO) {
    leftLimelight = new TagCamera(leftIO,
        VisionConstants.LIMELIGHT_NAME_L, VisionConstants.LEFT_CAMERA_TRANSFORM);
    rightLimelight = new TagCamera(rightIO,
        VisionConstants.LIMELIGHT_NAME_R, VisionConstants.RIGHT_CAMERA_TRANSFORM);
  }

  @Override
  public void earlyPeriodic() {
    // Updates odometry from vision.
    // Does not flush networktables.
    var leftEstimate = leftLimelight.update();
    var rightEstimate = rightLimelight.update();

    // Leftover from when MegaTag2 was in use
    // {
    //     // Flush networktables explicitly once to avoid network latency
    //     // Do not flush once per limelight, since flushing NT is ratelimited to once every 10ms
    //     // With one or more cameras each flushing periodically, you start seeing loop overruns
    //     NetworkTableInstance.getDefault().flush();
    // }

    Optional<TagPoseEstimate> chosenEstimate = Optional.empty();
    if (leftEstimate.isPresent() != rightEstimate.isPresent()) {
      chosenEstimate = leftEstimate.isPresent() ? leftEstimate : rightEstimate;
    } else if (leftEstimate.isPresent() && rightEstimate.isPresent()) {
      chosenEstimate = combineEstimates(leftEstimate.get(), rightEstimate.get());
    }

    // Only fuse in one estimate to avoid "double tapping" the Kalman filter
    // Prevents excessively weighting vision over odometry
    if (chosenEstimate.isPresent()) {
      var estimate = chosenEstimate.get();

      if (VisionHelpers.isValidPose(estimate.pose)
          && VisionHelpers.isValidStdevs(estimate.standardDeviation)) {
        validPoseNT.set(estimate.pose);
        RobotContainer.drive.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            estimate.standardDeviation
        );
      }
    }
  }

  // Directly taken from FRC 254's 2025 codebase
  private Optional<TagPoseEstimate> combineEstimates(TagPoseEstimate a, TagPoseEstimate b) {
    // Ensure A is the most recent pose
    if (a.timestampSeconds < b.timestampSeconds) {
      var tmp = a;
      a = b;
      b = tmp;
    }

    // Latency compensate the older pose to match the more recent one's timestamp
    Transform2d b_T_a = a.odometryAtTimestamp
        .minus(b.odometryAtTimestamp);

    Pose2d poseA = a.pose;
    Pose2d poseB = b.pose.transformBy(b_T_a);

    // Inverse‑variance weighting
    var varianceA = a.standardDeviation.elementTimes(a.standardDeviation);
    var varianceB = b.standardDeviation.elementTimes(b.standardDeviation);

    Rotation2d fusedHeading = poseB.getRotation();
    if (varianceA.get(2, 0) < VisionConstants.LARGE_VARIANCE
        && varianceB.get(2, 0) < VisionConstants.LARGE_VARIANCE) {
      fusedHeading = new Rotation2d(
          poseA.getRotation().getCos() / varianceA.get(2, 0)
              + poseB.getRotation().getCos() / varianceB.get(2, 0),
          poseA.getRotation().getSin() / varianceA.get(2, 0)
              + poseB.getRotation().getSin() / varianceB.get(2, 0));
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose = new Pose2d(
        new Translation2d(
            (poseA.getTranslation().getX() * weightAx
                + poseB.getTranslation().getX() * weightBx)
                / (weightAx + weightBx),
            (poseA.getTranslation().getY() * weightAy
                + poseB.getTranslation().getY() * weightBy)
                / (weightAy + weightBy)),
        fusedHeading);

    Matrix<N3, N1> fusedStdDev = VecBuilder.fill(
        Math.sqrt(1.0 / (weightAx + weightBx)),
        Math.sqrt(1.0 / (weightAy + weightBy)),
        Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = a.numTags + b.numTags;
    double time = b.timestampSeconds;

    return Optional.of(new TagPoseEstimate(
        fusedPose,
        time,
        fusedStdDev,
        numTags,
        a.odometryAtTimestamp
    ));
  }

  /**
   * Checks if both limelights see a tag, used for pit debugging
   * @return true if both limelights see a tag
   */
  public boolean limelightsSeeTag() {
    return leftLimelight.canSeeTag() && rightLimelight.canSeeTag();
  }
}

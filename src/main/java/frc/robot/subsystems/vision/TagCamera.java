// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision.TagPoseEstimate;
import frc.robot.subsystems.vision.VisionIO.PoseEstimateRecord;
import frc.robot.subsystems.vision.VisionIO.RawFiducialRecord;
import frc.robot.utils.vision.VisionHelpers;

public class TagCamera {
  private VisionIO visionIO;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private static final boolean VISUALIZE_TARGETS = true;

  // Timestamp deduplication
  private double lastResultTimestamp = -1;
  private final String cameraName;
  private final Transform3d cameraOffset;

  public TagCamera(VisionIO io, String name, Transform3d offset) {
    visionIO = io;
    cameraName = name;
    cameraOffset = offset;
  }

  /**
   * Call every periodic loop to fetch vision reported poses. 
   */
  public Optional<TagPoseEstimate> update() {
    visionIO.updateInputs(inputs);
    Logger.processInputs("Vison/" + cameraName, inputs);

    // Throttle performance while disabled to prevent overheating
    if (DriverStation.isEnabled()) {
      visionIO.setEnabled();
    } else {
      visionIO.setDisabled();
    }

    // Skip if disconnected
    if (!inputs.isAlive) {
      cleanDebugLines();
      return Optional.empty();
    }
    // Skip if no tags visible
    if (!inputs.canSeeTag) {
      cleanDebugLines();
      return Optional.empty();
    }

    if (inputs.megatagResult != null && inputs.megatagResult.tagCount() > 0) {
      // Skip duplicates
      if (inputs.megatagResult.timestampSeconds() > lastResultTimestamp) {
        if (VisionHelpers.isValidPose(inputs.megatagResult.pose())) {
          // Validate Z-axis
          if (Math.abs(inputs.rawPose3d.getZ()) <= VisionConstants.MAX_Z_ERROR) {
            var megatagEstimate = filterMegatagEstimate();
            var gyroEstimate = calculateGyroEstimate();

            if (megatagEstimate.isPresent()) {
              Logger.recordOutput("Vision/" + cameraName + "/Megatag Accepted",
                  megatagEstimate.get().pose());
            }
            if (gyroEstimate.isPresent()) {
              Logger.recordOutput("Vision/" + cameraName + "/Gyro Fused Accepted",
                  gyroEstimate.get().pose());
            }

            if (megatagEstimate.isPresent()) {
              Logger.recordOutput("Vision/" + cameraName + "/Estimate Type", "MEGATAG");
              lastResultTimestamp = inputs.megatagResult.timestampSeconds();
              drawDebugLines(inputs.rawFiducials, inputs.fiducialArrayLength);

              return megatagEstimate;
            } else if (gyroEstimate.isPresent()) {
              Logger.recordOutput("Vision/" + cameraName + "/Estimate Type", "GYRO");
              lastResultTimestamp = inputs.megatagResult.timestampSeconds();
              drawDebugLines(inputs.rawFiducials, inputs.fiducialArrayLength);

              return gyroEstimate;
            }
          }
        }
      }
    }
    Logger.recordOutput("Vision/" + cameraName + "/Estimate Type", "NONE");

    return Optional.empty();
  }

  private void cleanDebugLines() {
    if (!VISUALIZE_TARGETS) {
      return;
    }

    Logger.recordOutput("Vision/" + cameraName + "/Target Visualization", new Pose3d[0]);
  }

  private void drawDebugLines(RawFiducialRecord[] rawFiducials, int length) {
    if (!VISUALIZE_TARGETS) {
      return;
    }

    Pose3d robotPose = new Pose3d(RobotContainer.state.getPose());

    // Transform3d camOffset = Transform3d.kZero;

    Pose3d cameraPose = robotPose.transformBy(cameraOffset);

    Pose3d[] output = new Pose3d[length * 2];
    int k = 0;
    // output[0] = cameraPose;
    for (int i = 0; i < length; i++) {
      var tagPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(rawFiducials[i].id());
      if (tagPose.isPresent()) {
        output[k++] = tagPose.get();
      } else {
        output[k++] = cameraPose;
      }
      output[k++] = cameraPose;
    }

    Logger.recordOutput("Vision/" + cameraName + "/Target Visualization", output);
  }

  /**
   * Returns true if the camera can see a tag
   * @return A boolean
   */
  public boolean canSeeTag() {
    return inputs.canSeeTag && inputs.isAlive;
  }

  /**
   * Check for limelight heartbeat
   */
  public boolean isAlive() {
    return inputs.isAlive;
  }

  /**
  * Return the limelight's name
  * @return string
  */
  public String getName() {
    return cameraName;
  }

  private Optional<TagPoseEstimate> filterMegatagEstimate() {
    var megatagResult = inputs.megatagResult;

    // Single-tag validation
    if (megatagResult.tagCount() == 1) {
      if (!isAmbiguityAcceptable(inputs.rawFiducials, inputs.fiducialArrayLength)) {
        return Optional.empty();
      }

      // Don't check min tag area when disabled and seeding 
      // ie. angle does not yet match
      if (DriverStation.isEnabled() || isYawDifferenceAcceptable(megatagResult)) {
        if (megatagResult.avgTagArea() < VisionConstants.MIN_TAG_AREA_SINGLE_TAG) {
          return Optional.empty();
        }

        // For small tags, also check yaw difference
        if (megatagResult.avgTagArea() < VisionConstants.MIN_TAG_AREA_FOR_YAW_CHECK) {
          if (!isYawDifferenceAcceptable(megatagResult)) {
            return Optional.empty();
          }
        }
      }

    }

    if (Math.abs(RobotContainer.state.getYawVelocityAtTimestamp(
        megatagResult.timestampSeconds()
    ).orElse(Double.POSITIVE_INFINITY)) > VisionConstants.MAX_YAW_RATE_RADS) {

      return Optional.empty();
    }

    // If not disabled, ensure tag is within a certain distance
    // if (!DriverStation.isDisabled()) {
    //     if (
    //         megatagResult.pose.minus(driveSubsystem.getPose()).getTranslation().getNorm() 
    //         < VisionConstants.MEGATAG1_MAX_DISTANCE_THRESHOLD
    //     ) {
    //         return Optional.empty();   
    //     }
    // }

    // Calculate standard deviations
    Matrix<N3, N1> estimationStdDevs;
    if (VisionConstants.USE_AUTOMATIC_STANDARD_DEVIATIONS) {
      double quality = VisionHelpers.getMegatagEstimateQuality(inputs.rawFiducials);

      double xStd = inputs.rawStandardDeviationArray[VisionConstants.MEGATAG_1_XStdDevIndex] * quality;
      double yStd = inputs.rawStandardDeviationArray[VisionConstants.MEGATAG_1_YStdDevIndex] * quality;
      double xyStd = Math.max(xStd, yStd);

      double yawStd = inputs.rawStandardDeviationArray[VisionConstants.MEGATAG_1_YawStdDevIndex] * quality;

      estimationStdDevs = VecBuilder.fill(xyStd, xyStd, yawStd);
    } else {
      estimationStdDevs = VisionHelpers.calculateStdDevsMegatag(megatagResult, inputs.rawFiducials);
    }

    // Logger.recordOutput("Vision" + cameraName + "/Megatag STDDevs",
    //         new double[] { estimationStdDevs.get(0, 0), estimationStdDevs.get(1, 0), estimationStdDevs.get(2, 0) }
    // );

    var odometryAtTimestamp = RobotContainer.state.getPoseAtTimestamp(megatagResult.timestampSeconds());
    // Edgecase handling for if pose buffer hasn't been filled yet or the megatagResult is extremely out of date 
    if (odometryAtTimestamp.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(new TagPoseEstimate(
        megatagResult.pose(),
        megatagResult.timestampSeconds(),
        estimationStdDevs,
        megatagResult.tagCount(),
        odometryAtTimestamp.get()
    ));
  }

  private Optional<TagPoseEstimate> calculateGyroEstimate() {
    var megatagResult = inputs.megatagResult;

    // Prefer megatag 1 when more than one tag is visible
    if (megatagResult.tagCount() > 1) {
      return Optional.empty();
    }

    var odometryAtTimestamp = RobotContainer.state.getPoseAtTimestamp(megatagResult.timestampSeconds());
    // Edgecase handling for if pose buffer hasn't been filled yet or the megatagResult is extremely out of date 
    if (odometryAtTimestamp.isEmpty()) {
      return Optional.empty();
    }

    // Filter out estimates taken while spinning too fast (latency compensation has it's limits)
    if (Math.abs(RobotContainer.state.getYawVelocityAtTimestamp(
        megatagResult.timestampSeconds()
    ).orElse(Double.POSITIVE_INFINITY)) > VisionConstants.MAX_YAW_RATE_RADS) {

      return Optional.empty();
    }

    var tagPose3d = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(inputs.rawFiducials[0].id());
    if (tagPose3d.isEmpty()) {
      return Optional.empty();
    }

    Pose2d tagPose2d = new Pose2d(tagPose3d.get().toPose2d().getTranslation(), Rotation2d.kZero);
    Pose2d robotToTag = tagPose2d.relativeTo(megatagResult.pose());

    Pose2d calculatedPose = new Pose2d(
        tagPose2d
            .getTranslation()
            .minus(
                robotToTag
                    .getTranslation()
                    .rotateBy(odometryAtTimestamp.get().getRotation())),
        odometryAtTimestamp.get().getRotation());

    // Calculate standard deviations
    Matrix<N3, N1> estimationStdDevs;
    if (VisionConstants.USE_AUTOMATIC_STANDARD_DEVIATIONS) {
      double xStd = inputs.rawStandardDeviationArray[VisionConstants.MEGATAG_1_XStdDevIndex];
      double yStd = inputs.rawStandardDeviationArray[VisionConstants.MEGATAG_1_YStdDevIndex];
      double xyStd = Math.max(xStd, yStd);
      estimationStdDevs = VecBuilder.fill(xyStd, xyStd, VisionConstants.LARGE_VARIANCE);
    } else {
      estimationStdDevs = VisionHelpers.calculateStdDevsGyroFusion(megatagResult);
    }

    // Logger.recordOutput("Vision" + cameraName + "/Fused STDDevs",
    //         new double[] { estimationStdDevs.get(0, 0), estimationStdDevs.get(1, 0), estimationStdDevs.get(2, 0) }
    // );

    return Optional.of(new TagPoseEstimate(
        calculatedPose,
        megatagResult.timestampSeconds(),
        estimationStdDevs,
        1,
        odometryAtTimestamp.get()
    ));
  }

  /**
   * Checks if the ambiguity of detected tags is acceptable
   * @param tags Array of raw fiducial detections
   * @return true if ambiguity is below threshold, false otherwise
   */
  private boolean isAmbiguityAcceptable(RawFiducialRecord[] tags, int tagCount) {
    if (tags == null || tagCount == 0) {
      return false;
    }

    for (int i = 0; i < tagCount; i++) {
      var tag = tags[i];
      if (tag != null) {
        if (tag.ambiguity() > VisionConstants.AMBIGUITY_THRESHOLD) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Checks if the yaw difference between vision and odometry is acceptable for small tags
   * @param visionPose The pose estimate from vision
   * @param timestamp the timestamp of the vision estimate for latency compensation
   * @return true if yaw difference is below threshold, false otherwise
   */
  private boolean isYawDifferenceAcceptable(PoseEstimateRecord visionPose) {
    var odometryPose = RobotContainer.state.getPoseAtTimestamp(visionPose.timestampSeconds());
    if (odometryPose.isEmpty()) {
      return false;
    }

    double yawDifference = Math.abs(MathUtil.angleModulus(
        odometryPose.get().getRotation().getRadians() - visionPose.pose().getRotation().getRadians()
    ));

    return Math.toDegrees(yawDifference) <= VisionConstants.MAX_YAW_DIFFERENCE_DEG;
  }
}

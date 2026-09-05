// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.lib.EpochTimer;
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

  /** hardware */
  public final TagCamera frameCamera;
  public final TagCamera turretCamera;

  public int highTrustEstimatesLeft = 0;

  /*
   * Pose agreement. Counts consecutive accepted estimates that land close to
   * where
   * odometry already thinks we are. A high count means vision and odometry have
   * converged, so the pose can be trusted for something precise. A single good
   * estimate does not mean that; a sustained run of them does.
   */
  @AutoLogOutput(key = "Vision/Pose Stable Updates")
  private int poseStableUpdates = 0;

  private double lastAgreementTimestamp = -1;
  public final Trigger onGround = new Trigger(() -> {
    return !RobotContainer.state.onBump;
  }).onTrue(Commands.runOnce(() -> {
    highTrustEstimatesLeft = 5;
  }));
  // public final Trigger returnToNormal = new Trigger(() ->
  // shouldReturnToNormal).debounce(0.1)
  // .onTrue(Commands.runOnce(() -> justCrossedBump = false));

  public Vision(VisionIO frameIO, VisionIO turretIO) {
    frameCamera = new TagCamera(frameIO,
        VisionConstants.LIMELIGHT_NAME_FRAME, (timestamp) -> PhysicalConstants.ROBOT_TO_FRAME_CAMERA_PARTIAL);
    turretCamera = new TagCamera(turretIO,
        VisionConstants.LIMELIGHT_NAME_TURRET, this::calculateTurretCamPose);
  }

  @Override
  public void earlyPeriodic() {
    EpochTimer.BeginEpoch("Vision");
    {
      // Updates odometry from vision.
      // Does not flush networktables.
      var frameEstimate = frameCamera.update(false);
      var turretEstimate = turretCamera.update(true);

      if ((RobotContainer.state.isManualMode()) && RobotContainer.state.robotEnabled()) {
        // Ignore vision when on bump or manual mode
        frameEstimate = Optional.empty();
        turretEstimate = Optional.empty();
        Logger.recordOutput("Vision/Vision Enabled", false);
      } else {
        Logger.recordOutput("Vision/Vision Enabled", true);
      }
      // Leftover from when MegaTag2 was in use
      // {
      // // Flush networktables explicitly once to avoid network latency
      // // Do not flush once per limelight, since flushing NT is ratelimited to once
      // every 10ms
      // // With one or more cameras each flushing periodically, you start seeing loop
      // overruns
      // NetworkTableInstance.getDefault().flush();
      // }

      Optional<TagPoseEstimate> chosenEstimate = Optional.empty();
      if (frameEstimate.isPresent() != turretEstimate.isPresent()) {
        chosenEstimate = frameEstimate.isPresent() ? frameEstimate : turretEstimate;
      } else if (frameEstimate.isPresent() && turretEstimate.isPresent()) {
        chosenEstimate = combineEstimates(frameEstimate.get(), turretEstimate.get());
      }

      Logger.recordOutput("Vision/Bump High Trust Estimates", highTrustEstimatesLeft);
      // Logger.recordOutput("Vision/Bump Vision Reset Finishing",
      // shouldReturnToNormal);

      // Only fuse in one estimate to avoid "double tapping" the Kalman filter
      // Prevents excessively weighting vision over odometry
      if (chosenEstimate.isPresent()) {

        var estimate = chosenEstimate.get();

        if (VisionHelpers.isValidPose(estimate.pose)
            && VisionHelpers.isValidStdevs(estimate.standardDeviation)) {
          Logger.recordOutput("Vision/Validated Pose", estimate.pose);

          // Compare before applying the measurement, otherwise the correction
          // pulls odometry towards the estimate and the check answers itself.
          updatePoseAgreement(estimate.pose);

          Matrix<N3, N1> chosenDeviations = estimate.standardDeviation;
          if (highTrustEstimatesLeft > 0) {
            chosenDeviations = chosenDeviations.times(0.1); // Essentially teleport to the first few things we see
            highTrustEstimatesLeft--;
          }
          RobotContainer.drive.addVisionMeasurement(
              estimate.pose,
              estimate.timestampSeconds,
              chosenDeviations
          );
        }
      }
    }
    EpochTimer.EndEpoch("Vision");
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
    double time = a.timestampSeconds;

    return Optional.of(new TagPoseEstimate(
        fusedPose,
        time,
        fusedStdDev,
        numTags,
        a.odometryAtTimestamp
    ));
  }

  /**
   * Records whether an accepted estimate agrees with the current odometry pose.
   * Consecutive agreements build confidence; one disagreement resets it.
   */
  /* Package-private so the agreement logic can be driven directly from a test. */
  void updatePoseAgreement(Pose2d estimatePose) {
    double error = estimatePose.getTranslation()
        .getDistance(RobotContainer.state.getPose().getTranslation());

    if (error < VisionConstants.POSE_AGREEMENT_EPSILON) {
      poseStableUpdates++;
    } else {
      poseStableUpdates = 0;
    }

    lastAgreementTimestamp = Timer.getTimestamp();
    Logger.recordOutput("Vision/Pose Agreement Error", error);
  }

  /** How many consecutive accepted estimates have agreed with odometry. */
  public int getPoseStableUpdates() {
    return poseStableUpdates;
  }

  /**
   * Whether vision and odometry have agreed for long enough that the pose can be
   * trusted for something precise. False if no estimate has been accepted
   * recently, so this goes false when vision drops out rather than latching true.
   */
  @AutoLogOutput(key = "Vision/Pose Stable")
  public boolean isPoseStable() {
    if (lastAgreementTimestamp < 0
        || Timer.getTimestamp() - lastAgreementTimestamp > VisionConstants.POSE_AGREEMENT_STALE_TIME) {
      return false;
    }
    return poseStableUpdates >= VisionConstants.POSE_STABLE_UPDATE_THRESHOLD;
  }

  /**
   * Checks if either of the limelights see a tag, used for pit debugging
   * @return true if either of the limelights see a tag
   */
  public boolean limelightsSeeTag() {
    return frameCamera.canSeeTag() || turretCamera.canSeeTag();
  }

  // Calculates turret pose in robot space
  private Transform3d calculateTurretCamPose(double timestamp) {
    var turretAngle = RobotContainer.state.getTurretAngleTimestamp(timestamp);
    double turretAngleRadians = 0;
    if (turretAngle.isPresent()) {
      turretAngleRadians = turretAngle.get().getRadians();
    }

    var robotToTurret = new Transform3d(PhysicalConstants.ROBOT_TO_TURRET_CENTER.getTranslation(),
        new Rotation3d(0, 0, turretAngleRadians));
    return robotToTurret.plus(PhysicalConstants.TURRET_CAMERA_OFFSET_FROM_CENTER_PARTIAL);
  }
}

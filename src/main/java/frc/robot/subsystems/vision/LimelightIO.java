// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Objects;
import java.util.function.DoubleFunction;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionHelpers;

/** Encapsulates the logic for megatag based localization with a limelight */
public class LimelightIO implements VisionIO {
  private final String limelightName;

  // Connection monitoring
  private double lastHeartbeatValue = -1;
  private double lastHeartbeatTime = -1;

  public LimelightIO(String name) {
    this.limelightName = Objects.requireNonNull(name, "Limelight name cannot be null");
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, DoubleFunction<Transform3d> cameraOffset) {
    double heartBeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
    if (lastHeartbeatValue != heartBeat) {
      lastHeartbeatValue = heartBeat;
      lastHeartbeatTime = Timer.getFPGATimestamp();
    }
    inputs.isAlive = (Timer.getFPGATimestamp() - lastHeartbeatTime) < VisionConstants.LL_HEARTBEAT_MIN_FREQ;

    // Exit early if disconnected
    if (!inputs.isAlive) {
      return;
    }

    // Update valid tag IDs
    // LimelightHelpers.SetFiducialIDFiltersOverride(limelightName,
    // VisionHelpers.getValidTagIDs());

    inputs.canSeeTag = LimelightHelpers.getTV(limelightName);
    if (!inputs.canSeeTag) {
      return;
    }

    // Process MegaTag1
    var result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    inputs.megatagResult = new PoseEstimateRecord(
        result.pose, result.timestampSeconds,
        result.latency,
        result.tagCount, result.tagSpan, result.avgTagDist,
        result.avgTagArea, result.isMegaTag2
    );

    // Place raw fiducials into array. Do not reallocate array periodically to save
    // on performance.
    // Reallocating records is fine since records are heavily optimized by the JVM
    inputs.fiducialArrayLength = Math.min(result.rawFiducials.length, inputs.rawFiducials.length);
    for (int i = 0; i < inputs.fiducialArrayLength; i++) {
      var f = result.rawFiducials[i];
      inputs.rawFiducials[i] = new RawFiducialRecord(
          f.id,
          f.txnc,
          f.tync,
          f.ta,
          f.distToCamera,
          f.distToRobot,
          f.ambiguity
      );
    }
    // Clear out old tags
    for (int i = inputs.fiducialArrayLength; i < inputs.rawFiducials.length; i++) {
      inputs.rawFiducials[i] = null;
    }

    inputs.rawPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
    // Get latest standard deviations from cameras
    inputs.rawStandardDeviationArray = VisionHelpers.getAutomaticStandardDeviations(limelightName);
  }

  /** 
   * Runs cameras unthrottled while enabled
   */
  @Override
  public void setEnabled() {
    LimelightHelpers.SetThrottle(limelightName, 0);
  }

  /**
   * Throttles cameras to manage temperature while robot is disabled
   */
  @Override
  public void setDisabled() {
    LimelightHelpers.SetThrottle(limelightName, VisionConstants.LIMELIGHT_DISABLED_THROTTLE);
  }
}

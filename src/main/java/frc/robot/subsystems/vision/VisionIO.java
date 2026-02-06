// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.DoubleFunction;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
  /** Data coming from a camera */
  @AutoLog
  public static class VisionIOInputs {
    public boolean canSeeTag;
    public boolean isAlive;
    // public LimelightHelpers.PoseEstimate megatagResult;
    public PoseEstimateRecord megatagResult = new PoseEstimateRecord(Pose2d.kZero, 0, 0, 0, 0, 0, 0, false);
    public RawFiducialRecord[] rawFiducials = new RawFiducialRecord[32]; // Will contian null entries. Will fail I
                                                                         // guess if it sees more than 32 tags at
                                                                         // once.
    public int fiducialArrayLength;
    public Pose3d rawPose3d;
    public double[] rawStandardDeviationArray = new double[12];

    public VisionIOInputs() {
      for (int i = 0; i < rawFiducials.length; i++) {
        rawFiducials[i] = new RawFiducialRecord(0, 0, 0, 0, 0, 0, 1);
      }
    }
  }

  // Mirror of LimelightHelpers.RawFiducial
  public record RawFiducialRecord(
      int id,
      double txnc,
      double tync,
      double ta,
      double distToCamera,
      double distToRobot,
      double ambiguity
  ) {}

  // Mirror of LimelightHelpers.PoseEstimate (excluding the fiducial array)
  public record PoseEstimateRecord(
      Pose2d pose,
      double timestampSeconds,
      double latency,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      boolean isMegaTag2
  ) {}

  public default void updateInputs(VisionIOInputs inputs, DoubleFunction<Transform3d> cameraTransform) {}

  public default void setEnabled() {};

  public default void setDisabled() {};
}

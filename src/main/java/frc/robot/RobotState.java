// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.data.Constants.CodeConstants;

public class RobotState {
  public static enum SuperstructureState {
    TARGET_PASS,
    TARGET_HUB,
    DISABLED
  }

  @AutoLogOutput(key = "RobotState/Superstructure State")
  public static SuperstructureState superstructureState = SuperstructureState.TARGET_HUB;

  /*                       */
  /* Latency Compensation */
  /*                       */

  // Timestamps are in the timebase of Timer.getFPGATimestamp()
  private TimeInterpolatableBuffer<Pose2d> poseHistoryBuffer = TimeInterpolatableBuffer
      .createBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);
  private TimeInterpolatableBuffer<Double> yawVelocityHistoryBuffer = TimeInterpolatableBuffer
      .createDoubleBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);

  private ChassisSpeeds latestChassisSpeeds = new ChassisSpeeds();
  private Pose2d latestPose = new Pose2d();

  /**
   * Gets the robot pose at the given timestamp (FPGA timebase)
   */
  public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
    return poseHistoryBuffer.getSample(timestamp);
  }

  /**
   * Gets the robot yaw velocity at the given timestamp (FPGA timebase)
   */
  public Optional<Double> getYawVelocityAtTimestamp(double timestamp) {
    return yawVelocityHistoryBuffer.getSample(timestamp);
  }

  /**
   * Gets the robot velocity in robot space
   */
  public ChassisSpeeds getRobotVelocity() {
    return latestChassisSpeeds;
  }

  /**
   * Gets the robot velocity in field space
   */
  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(latestChassisSpeeds, getRotation());
  }

  /**
   * Returns the current odometry pose. Private to standardize all access through
   * RobotState
   */
  public Pose2d getPose() {
    return latestPose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  // Called once in earlyPeriodic to update odometry state
  public void updateOdometryState(double timestamp, Pose2d pose, ChassisSpeeds chassisSpeeds) {
    // Less accurate than high hz odometry thread but probably good enough?
    poseHistoryBuffer.addSample(timestamp, pose);
    yawVelocityHistoryBuffer.addSample(timestamp, chassisSpeeds.omegaRadiansPerSecond);
    latestChassisSpeeds = chassisSpeeds;
    latestPose = pose;
  }

  public SuperstructureState getSuperstructureState() {
    return superstructureState;
  }

  public void setSuperstructureState(SuperstructureState state) {
    superstructureState = state;
  }
}

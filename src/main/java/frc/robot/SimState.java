// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SimState {
  private SwerveDrivePoseEstimator underlyingPoseEstimator = null;
  @AutoLogOutput(key = "RobotState/Fuel Loaded")
  private int fuelCount = 0;

  public void initializeUnderlyingPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d rawGyroRotation,
      SwerveModulePosition[] lastModulePositions) {
    underlyingPoseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
        lastModulePositions, Pose2d.kZero);
  }

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
    return underlyingPoseEstimator;
  }

  /*
   * How much of the wheels' motion the robot does not actually make. Zero is
   * perfect grip; one is wheels spinning while the robot stands still.
   *
   * This is what makes the simulated robot able to lose its odometry the way the
   * real one does. Both pose estimators used to be fed identical module
   * positions, so the truth pose and odometry tracked each other exactly and
   * could never disagree -- which meant nothing that reads pose agreement could
   * be exercised in simulation, and the drift measured across real bump
   * crossings (three centimetres for a clean one, over a metre for a slow one)
   * had no counterpart here.
   */
  private double slip = 0;
  private SwerveModulePosition[] truePositions = null;
  private double[] lastSeenDistances = null;

  /** Sets how much grip the robot is losing, 0 for none and 1 for all of it. */
  public void setSlip(double value) {
    slip = MathUtil.clamp(value, 0, 1);
  }

  @AutoLogOutput(key = "SimField/Slip")
  public double getSlip() {
    return slip;
  }

  /**
   * Advances the truth pose by what the robot actually moved, which is the
   * wheels' motion less whatever they slipped away.
   *
   * <p>
   * Odometry still integrates the full wheel motion, because that is all the
   * real robot can see. The gap between the two is the error vision then has to
   * correct, and it accumulates and persists exactly as it does on the field.
   */
  public void updateUnderlying(double timestamp, Rotation2d gyroRotation,
      SwerveModulePosition[] positions) {
    if (underlyingPoseEstimator == null) {
      return;
    }
    if (truePositions == null) {
      truePositions = new SwerveModulePosition[positions.length];
      lastSeenDistances = new double[positions.length];
      for (int i = 0; i < positions.length; i++) {
        truePositions[i] = new SwerveModulePosition(positions[i].distanceMeters, positions[i].angle);
        lastSeenDistances[i] = positions[i].distanceMeters;
      }
    }

    for (int i = 0; i < positions.length; i++) {
      double delta = positions[i].distanceMeters - lastSeenDistances[i];
      lastSeenDistances[i] = positions[i].distanceMeters;
      truePositions[i] = new SwerveModulePosition(
          truePositions[i].distanceMeters + delta * (1 - slip), positions[i].angle);
    }
    underlyingPoseEstimator.updateWithTime(timestamp, gyroRotation, truePositions);
  }

  /** Moves the truth pose, for a wall the robot cannot drive through. */
  public void setTruePose(Pose2d pose, Rotation2d gyroRotation) {
    if (underlyingPoseEstimator == null || truePositions == null) {
      return;
    }
    underlyingPoseEstimator.resetPosition(gyroRotation, truePositions, pose);
  }

  /** Forgets the accumulated slip, so the truth pose is wherever odometry says. */
  public void resetSlipTracking() {
    truePositions = null;
    lastSeenDistances = null;
  }

  @AutoLogOutput(key = "Vision/UnderlyingFieldPose")
  public Pose2d getPose() {
    if (underlyingPoseEstimator != null) {
      return underlyingPoseEstimator.getEstimatedPosition();
    }
    return Pose2d.kZero;
  }

  public void simIntake() {
    fuelCount++;
  }
}

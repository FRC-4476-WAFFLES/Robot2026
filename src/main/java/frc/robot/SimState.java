// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

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

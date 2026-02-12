// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.shooter.turret.Turret.TurretSetpoint;
import frc.robot.utils.lib.Spline1D;
import frc.robot.utils.lib.WafflesUtilities;

public class ShotPlanner {
  public static record ShootingParameters(
      TurretSetpoint turretSetpoint,
      double hoodAngle,
      double flywheelSpeed
  ) {}

  private static ShootingParameters parameters = null;
  private static final Spline1D flywheelSpeeds = new Spline1D(Constants.FlywheelConstants.DistanceMap);
  private static final Spline1D hoodAngle = new Spline1D(Constants.HoodConstants.DistanceMap);

  public static final Translation2d passingTargetLeft = new Translation2d(1.5, 1);
  public static final Translation2d passingTargetRight = new Translation2d(passingTargetLeft.getX(),
      WafflesUtilities.FlipYIfRedAlliance(passingTargetLeft.getY()));
  public static final double latencyCompensationStep = CodeConstants.PERIODIC_LOOP_TIME;

  public static ShootingParameters aimAtField(Translation2d fieldPose) { // Pose is flipped before this function
    Pose2d robotPose = RobotContainer.state.getPose();
    ChassisSpeeds robotChassisSpeeds = RobotContainer.state.getRobotVelocity();
    robotPose = robotPose.exp(
        new Twist2d(
            robotChassisSpeeds.vxMetersPerSecond * latencyCompensationStep,
            robotChassisSpeeds.vyMetersPerSecond * latencyCompensationStep,
            robotChassisSpeeds.omegaRadiansPerSecond * latencyCompensationStep));

    Pose2d turretPose = robotPose.transformBy(
        new Transform2d(PhysicalConstants.ROBOT_TO_TURRET_CENTER.getTranslation().toTranslation2d(), Rotation2d.kZero)
    );

    double distanceToTarget = turretPose.getTranslation().getDistance(fieldPose);

    Logger.recordOutput("RobotState/Shooter Target", new Pose2d(fieldPose, Rotation2d.kZero));
    Logger.recordOutput("RobotState/Turret Position", turretPose);

    Rotation2d turretAngle = fieldPose.minus(turretPose.getTranslation()).getAngle();

    parameters = new ShootingParameters(
        new TurretSetpoint(turretAngle, 0),
        hoodAngle.interpolate(distanceToTarget),
        flywheelSpeeds.interpolate(distanceToTarget)
    );

    return parameters;
  }

  public static ShootingParameters aimToPass() {
    var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.state.getPose());
    Translation2d targetPoint = passingTargetLeft;

    if (pose.getY() > FieldConstants.fieldWidth / 2) {
      targetPoint = passingTargetRight;
    }
    return aimAtField(WafflesUtilities.FlipIfRedAlliance(targetPoint));
  }

  public static ShootingParameters aimToHub() {
    return aimAtField(WafflesUtilities.FlipIfRedAlliance(FieldConstants.Hub.topCenterPoint.toTranslation2d()));
  }

  public static ShootingParameters aimToTag() {
    return aimAtField(
        WafflesUtilities
            .FlipIfRedAlliance(
                VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(19).orElse(Pose3d.kZero).toPose2d().getTranslation())
    );
  }

  public static ShootingParameters getLatestParameters() {
    return parameters;
  }

  public static Supplier<TurretSetpoint> turretSetpoint() {
    return () -> parameters.turretSetpoint;
  }

  public static DoubleSupplier flywheelSpeed() {
    return () -> parameters.hoodAngle;
  }

  public static DoubleSupplier hoodAngle() {
    return () -> parameters.hoodAngle;
  }
}

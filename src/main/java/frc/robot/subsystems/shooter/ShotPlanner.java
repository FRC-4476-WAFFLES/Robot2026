// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.data.Constants.HoodConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.shooter.turret.Turret.TurretSetpoint;
import frc.robot.utils.lib.EpochTimer;
import frc.robot.utils.lib.SplineMonotone1D;
import frc.robot.utils.lib.WafflesUtilities;

public class ShotPlanner {
  public static record ShootingParameters(
      TurretSetpoint turretSetpoint,
      double hoodAngle,
      double flywheelSpeed
  ) {}

  private static ShootingParameters parameters = null;
  private static final SplineMonotone1D flywheelSpeeds = new SplineMonotone1D(FlywheelConstants.DistanceMap);
  private static final SplineMonotone1D hoodAngle = new SplineMonotone1D(HoodConstants.DistanceMap);
  private static final SplineMonotone1D timeOfFlightMap = new SplineMonotone1D(CodeConstants.TimeofFlightMap);

  public static final Translation2d passingTargetLeft = new Translation2d(1.5, 1);
  public static final Translation2d passingTargetRight = new Translation2d(passingTargetLeft.getX(),
      WafflesUtilities.FlipYIfRedAlliance(passingTargetLeft.getY()));
  public static final double latencyCompensationStep = CodeConstants.PERIODIC_LOOP_TIME;

  private static Rotation2d lastTurretAngle;

  // private static final LoggedNetworkNumber hoodAngleTuner = new
  // LoggedNetworkNumber("/Tuning/");
  // private static final LoggedNetworkNumber shooterSpeedTuner = new
  // LoggedNetworkNumber("/Tuning/Shooter Speed");

  private static final LinearFilter turretAngleFilter = LinearFilter
      .movingAverage((int) (0.1 / CodeConstants.PERIODIC_LOOP_TIME));

  public static ShootingParameters aimAtField(Translation2d fieldTarget) { // Pose is flipped before this function
    EpochTimer.BeginEpoch("Aiming");
    {
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

      double distanceToTarget = turretPose.getTranslation().getDistance(fieldTarget);

      Logger.recordOutput("RobotState/Shooter Target", new Pose2d(fieldTarget, Rotation2d.kZero));
      Logger.recordOutput("RobotState/Turret Position", turretPose);
      Logger.recordOutput("RobotState/Distance To Target", distanceToTarget);

      Translation2d currentTarget = fieldTarget;
      // Hastily taken from 6328. Everybody say thank you 6328.
      if (CodeConstants.SHOOT_ON_MOVE) {
        ChassisSpeeds robotVelocity = RobotContainer.state.getFieldVelocity();
        double robotAngle = robotPose.getRotation().getRadians();
        double turretOffsetX = PhysicalConstants.ROBOT_TO_TURRET_CENTER.getX();
        double turretOffsetY = PhysicalConstants.ROBOT_TO_TURRET_CENTER.getY();
        double turretVelocityX = robotVelocity.vxMetersPerSecond
            - robotVelocity.omegaRadiansPerSecond
                * (turretOffsetX * Math.sin(robotAngle) + turretOffsetY * Math.cos(robotAngle));
        double turretVelocityY = robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turretOffsetX * Math.cos(robotAngle) - turretOffsetY * Math.sin(robotAngle));

        Translation2d turretVel = new Translation2d(turretVelocityX, turretVelocityY);

        double previousTimeOfFlight = Double.NaN;
        double currentDistance = distanceToTarget;
        for (int i = 0; i < 5; i++) {
          double timeOfFlight = timeOfFlightMap.interpolate(currentDistance);

          currentTarget = fieldTarget.minus(turretVel.times(timeOfFlight * 0.1));

          currentDistance = currentTarget.getDistance(turretPose.getTranslation());

          if (previousTimeOfFlight != Double.NaN && Math.abs(timeOfFlight - previousTimeOfFlight) < 0.1) {
            break;
          }
          previousTimeOfFlight = timeOfFlight;
        }

        distanceToTarget = currentDistance;

        Logger.recordOutput("RobotState/Adjusted Target Position", new Pose2d(currentTarget, Rotation2d.kZero));
        Logger.recordOutput("RobotState/Adjusted Distance To Target", currentDistance);
      }

      Rotation2d turretAngle = currentTarget.minus(turretPose.getTranslation()).getAngle();

      double turretVelocity = 0;
      if (CodeConstants.SHOOT_ON_MOVE) {
        // Numerically differentiate the desired turret angle and low pass filter it
        if (lastTurretAngle == null)
          lastTurretAngle = turretAngle;
        // Convert to rotations/sec to match turret profile units
        turretVelocity = turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRotations() / CodeConstants.PERIODIC_LOOP_TIME);
        lastTurretAngle = turretAngle;
      }

      parameters = new ShootingParameters(
          new TurretSetpoint(turretAngle, turretVelocity),
          hoodAngle.interpolate(distanceToTarget),
          flywheelSpeeds.interpolate(distanceToTarget)
      );
    }
    EpochTimer.EndEpoch("Aiming");

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
                VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(20).orElse(Pose3d.kZero).toPose2d().getTranslation())
    );
  }

  public static ShootingParameters aimManual() {
    if (CodeConstants.MANUAL_SHOOTER_TUNING) {
      parameters = new ShootingParameters(
          new TurretSetpoint(Rotation2d.kZero, 0),
          SmartDashboard.getNumber("Hood Angle", 0),
          SmartDashboard.getNumber("Shooter Speed", 0)
      );

    } else {
      var target = RobotContainer.state.getManualOverrideTarget();

      parameters = new ShootingParameters(
          new TurretSetpoint(target.getTurretSetpoint(), 0),
          hoodAngle.interpolate(target.getDistance()),
          flywheelSpeeds.interpolate(target.getDistance())
      );
    }

    return parameters;
  }

  public static ShootingParameters getLatestParameters() {
    return parameters;
  }

  public static Supplier<TurretSetpoint> turretSetpoint() {
    return () -> parameters.turretSetpoint;
  }

  public static DoubleSupplier flywheelSpeed() {
    return () -> parameters.flywheelSpeed;
  }

  public static DoubleSupplier hoodAngle() {
    return () -> parameters.hoodAngle;
  }
}

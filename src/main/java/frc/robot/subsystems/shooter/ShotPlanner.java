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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.data.Constants.HoodConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.shooter.turret.Turret.TurretSetpoint;
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
  // TODO: Setup sotm. Mostly ready to drop in.
  private static final SplineMonotone1D timeOfFlight = new SplineMonotone1D(CodeConstants.TimeofFlightMap);

  public static final Translation2d passingTargetLeft = new Translation2d(1.5, 1);
  public static final Translation2d passingTargetRight = new Translation2d(passingTargetLeft.getX(),
      WafflesUtilities.FlipYIfRedAlliance(passingTargetLeft.getY()));
  public static final double latencyCompensationStep = CodeConstants.PERIODIC_LOOP_TIME;

  // private static final LoggedNetworkNumber hoodAngleTuner = new
  // LoggedNetworkNumber("/Tuning/");
  // private static final LoggedNetworkNumber shooterSpeedTuner = new
  // LoggedNetworkNumber("/Tuning/Shooter Speed");

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
    Logger.recordOutput("RobotState/Distance To Target", distanceToTarget);

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
                VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(20).orElse(Pose3d.kZero).toPose2d().getTranslation())
    );
  }

  public static ShootingParameters aimManual() {
    // var target = RobotContainer.state.getManualOverrideTarget();

    // parameters = new ShootingParameters(
    // new TurretSetpoint(target.getTurretSetpoint(), 0),
    // hoodAngle.interpolate(target.getDistance()),
    // flywheelSpeeds.interpolate(target.getDistance())
    // );

    parameters = new ShootingParameters(
        new TurretSetpoint(Rotation2d.kZero, 0),
        SmartDashboard.getNumber("Hood Angle", 0),
        SmartDashboard.getNumber("Shooter Speed", 0)
    );
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

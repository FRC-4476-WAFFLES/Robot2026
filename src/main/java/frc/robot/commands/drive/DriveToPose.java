// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class DriveToPose {
  protected Command cmd;
  private double prevTime = 0;
  private double lastMaxAngularVelocityConstraint = 0;

  public DriveToPose(Supplier<BlueRelativeTarget> target, BooleanSupplier purePursuit,
      boolean selfEnd) {
    cmd = generate(target, purePursuit, selfEnd);
  }

  public Command follow() {
    return cmd;
  }

  private Command generate(Supplier<BlueRelativeTarget> target, BooleanSupplier purePursuit,
      boolean selfEnd) {
    ProfiledPIDController angleController = new ProfiledPIDController(
        DriveCommands.ANGLE_KP,
        0.0,
        DriveCommands.ANGLE_KD,
        new TrapezoidProfile.Constraints(DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    var state = RobotContainer.state;
    var drive = RobotContainer.drive;

    ChassisSpeeds lastOutput = new ChassisSpeeds();
    ChassisSpeeds lastFieldSpeeds = new ChassisSpeeds();

    return Commands.run(() -> {
      Pose2d pose = state.getPose();
      ChassisSpeeds actualSpeeds = state.getRobotVelocity();

      BlueRelativeTarget blueTarget = target.get();
      APTarget currentTarget = blueTarget.getFieldRelative();

      // Mutate constraints
      RobotState.setAutopilotMaxVelocity(blueTarget.getMaxVelocity());

      if (lastMaxAngularVelocityConstraint != blueTarget.getMaxRotationRate()) {
        angleController.setConstraints(
            new TrapezoidProfile.Constraints(DriveCommands.ANGLE_MAX_VELOCITY, blueTarget.getMaxRotationRate()));
        lastMaxAngularVelocityConstraint = blueTarget.getMaxRotationRate();
      }

      // Technically ignores the rotation radus field of an APTarget since we just
      // always go to setpoint. Would be easy to fix but unnecessary.
      ChassisSpeeds fieldRelativeGoalSpeed;
      double omega = angleController.calculate(
          RobotContainer.state.getRotation().getRadians(), currentTarget.getReference().getRotation().getRadians());

      // Use autopilot for precise positions or entry angle requirements
      if (!purePursuit.getAsBoolean() || blueTarget.hasEntryAngle() || CodeConstants.DISABLE_PURE_PURSUIT) {
        // Don't feed in actual robot velocity to avoid posibility of feedback loops
        // Actually consistent with how motion profiles (effectively what autopilot
        // generates) should be created. The docs do not do this.
        APResult output = state.autopilot().calculate(pose, lastOutput, currentTarget);

        fieldRelativeGoalSpeed = new ChassisSpeeds(
            output.vx().in(MetersPerSecond),
            output.vy().in(MetersPerSecond),
            omega);
        Logger.recordOutput("RobotState/Autopilot/Pure Pursuit", false);

      } else {
        // Otherwise just go flat out for continuous driving
        Translation2d direction = currentTarget.getReference().getTranslation().minus(pose.getTranslation());
        Double directionMagnitude = direction.getNorm();

        fieldRelativeGoalSpeed = new ChassisSpeeds(
            (direction.getX() / directionMagnitude) * blueTarget.getMaxVelocity(),
            (direction.getY() / directionMagnitude) * blueTarget.getMaxVelocity(),
            omega
        );

        // Don't slew limit outside pursuit to avoid issues with autopilot
        Translation2d deltaV = new Translation2d(
            fieldRelativeGoalSpeed.vxMetersPerSecond - lastFieldSpeeds.vxMetersPerSecond,
            fieldRelativeGoalSpeed.vyMetersPerSecond - lastFieldSpeeds.vyMetersPerSecond);
        double deltaMagnitude = deltaV.getNorm();

        double currentTime = Timer.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        prevTime = currentTime;

        double constrainedMagnitude = Math.min(deltaMagnitude, CodeConstants.AUTO_SLEW_LIMIT * elapsedTime);

        fieldRelativeGoalSpeed.vxMetersPerSecond = lastFieldSpeeds.vxMetersPerSecond
            + ((deltaV.getX() / deltaMagnitude) * constrainedMagnitude);
        fieldRelativeGoalSpeed.vyMetersPerSecond = lastFieldSpeeds.vyMetersPerSecond
            + (deltaV.getY() / deltaMagnitude) * constrainedMagnitude;
        copyChassisSpeeds(lastFieldSpeeds, fieldRelativeGoalSpeed);

        Logger.recordOutput("RobotState/Autopilot/Pure Pursuit", true);
      }

      var robotRelativeGoalSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeGoalSpeed, pose.getRotation());

      Logger.recordOutput("RobotState/Autopilot/Target", currentTarget.getReference());
      Logger.recordOutput("RobotState/Autopilot/Field Relative Goal Speeds", fieldRelativeGoalSpeed);
      Logger.recordOutput("RobotState/Autopilot/Velocity Limit", RobotState.getAutopilotVelocityConstraint());

      var trackingError = actualSpeeds.minus(robotRelativeGoalSpeed);
      if (Math.hypot(trackingError.vxMetersPerSecond,
          trackingError.vyMetersPerSecond) > CodeConstants.AUTO_MAX_TRACKING_ERROR.in(Meters)) {
        copyChassisSpeeds(lastOutput, actualSpeeds);
      } else {
        copyChassisSpeeds(lastOutput, robotRelativeGoalSpeed);
      }

      Logger.recordOutput("RobotState/Autopilot/Tracking Error", trackingError);

      drive.runVelocity(robotRelativeGoalSpeed);
    }, RobotContainer.drive)
        .until(() -> selfEnd && state.autopilot().atTarget(state.getPose(), target.get().getFieldRelative()))
        .beforeStarting(() -> {
          angleController.reset(RobotContainer.state.getRotation().getRadians());

          var fieldRelativeSpeeds = state.getFieldVelocity();
          copyChassisSpeeds(lastFieldSpeeds, fieldRelativeSpeeds);

          var robotRelativeSpeeds = state.getRobotVelocity();
          copyChassisSpeeds(lastOutput, robotRelativeSpeeds);

          prevTime = Timer.getTimestamp();

          Logger.recordOutput("RobotState/Autopilot/Active", true);
        })
        .finallyDo(() -> {
          RobotState.resetAutopilotConstraints();

          Logger.recordOutput("RobotState/Autopilot/Active", false);

          if (target.get().getFieldRelative().getVelocity() > 0) {
            return;
          }
          RobotContainer.drive.stop();
        });
  }

  // Helper to mutate chassisSpeeds objects (mutated because local variables
  // lambda enclosing scopes must be effectively final)
  private static void copyChassisSpeeds(ChassisSpeeds target, ChassisSpeeds toCopy) {
    target.vxMetersPerSecond = toCopy.vxMetersPerSecond;
    target.vyMetersPerSecond = toCopy.vyMetersPerSecond;
    target.omegaRadiansPerSecond = toCopy.omegaRadiansPerSecond;
  }
}

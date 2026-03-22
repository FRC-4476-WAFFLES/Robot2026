// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class PassThroughTarget {
  private Pose2d startPose = Pose2d.kZero;
  private Pose2d targetPose = Pose2d.kZero;
  private final Command cmd;

  public PassThroughTarget(BlueRelativeTarget target) {
    cmd = passThroughTarget(target);
  }

  private Command passThroughTarget(BlueRelativeTarget target) {
    var state = RobotContainer.state;

    return DriveCommands.autoToFieldPose(() -> target, () -> true, false)
        .onlyWhile(() -> !AutoPath.ShouldAdvanceToNextTarget(state.getPose(), targetPose, startPose))
        .beforeStarting(() -> {
          startPose = state.getPose();
          targetPose = target.getFieldRelativePose();
          RobotState.setAutopilotMaxVelocity(target.getMaxVelocity());
        });
  }

  public Command follow() {
    return cmd;
  }
}

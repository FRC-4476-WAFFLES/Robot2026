// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoPath {
  private int targetIndex = 0;
  private boolean onFinalTarget = false;
  private boolean preciseFinish = false;
  private Pose2d startingPose = Pose2d.kZero;
  private final BlueRelativeTarget[] targets;
  private final Command cmd;

  public AutoPath(BlueRelativeTarget... targets) {
    this.targets = targets;
    cmd = followPath();
  }

  public AutoPath withPreciseFinish() {
    preciseFinish = true;
    return this;
  }

  public void mirror() {
    for (BlueRelativeTarget blueRelativeTarget : targets) {
      blueRelativeTarget.mirror();
    }
  }

  public Command follow() {
    return cmd;
  }

  private Command followPath() {
    var state = RobotContainer.state;
    return Commands.sequence(
        DriveCommands.autoToFieldPose(() -> {
          var current = RobotContainer.state.getPose();
          var target = targets[targetIndex].getFieldRelativePose();
          var lastTarget = targetIndex == 0 ? startingPose : targets[targetIndex - 1].getFieldRelativePose();

          if (ShouldAdvanceToNextTarget(current, target, lastTarget)) {
            if (targetIndex >= targets.length - 1) {
              onFinalTarget = true;
            } else {
              targetIndex++;
            }
          }

          // Mutate constraints
          RobotState.setAutopilotMaxVelocity(targets[targetIndex].getMaxVelocity());

          return targets[targetIndex].getFieldRelative();
        }, () -> !onFinalTarget, false).until(this::shouldFinish)
    ).beforeStarting(() -> {
      targetIndex = 0;
      onFinalTarget = false;
      startingPose = state.getPose();
    });
  }

  private boolean shouldFinish() {
    if (onFinalTarget) {
      if (preciseFinish) {
        return RobotContainer.state.autopilot().atTarget(RobotContainer.state.getPose(),
            targets[targets.length - 1].getFieldRelative());
      } else {
        var current = RobotContainer.state.getPose();
        var target = targets[targets.length - 1].getFieldRelativePose();
        var lastTarget = targets[targets.length - 2].getFieldRelativePose();

        return ShouldAdvanceToNextTarget(current, target, lastTarget);
      }
    }
    return false;
  }

  public static boolean ShouldAdvanceToNextTarget(Pose2d robot, Pose2d target, Pose2d lastTarget) {
    double distanceToTarget = robot.getTranslation().getDistance(target.getTranslation());
    if (distanceToTarget < CodeConstants.AUTO_POSITION_TOLERANCE_VAGUE.in(Meters)) {
      return true;
    }

    // Check if overshot target
    Translation2d currentTargetToRobot = robot.getTranslation().minus(target.getTranslation());
    Translation2d lastToCurrentTarget = target.getTranslation().minus(lastTarget.getTranslation());

    if (currentTargetToRobot.dot(lastToCurrentTarget) > 0) {
      return true;
    }
    return false;
  }
}

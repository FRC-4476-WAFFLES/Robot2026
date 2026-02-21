// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoPath {
  private int targetIndex = 0;
  private boolean onFinalTarget = false;
  private boolean preciseFinish = false;
  private Pose2d startingPose = Pose2d.kZero;
  private final BlueRelativeTarget[] targets;
  private final Command cmd;

  public AutoPath(boolean flipped, boolean preciseFinish, BlueRelativeTarget... targets) {
    this.targets = targets;
    this.preciseFinish = preciseFinish;
    cmd = followPath(flipped);
  }

  public Command follow() {
    return cmd;
  }

  private Command followPath(boolean flipped) {
    var state = RobotContainer.state;
    return Commands.sequence(
        DriveCommands.autoToFieldPose(() -> {
          var target = targets[targetIndex].getFieldRelativePose();
          var lastTarget = targetIndex == 0 ? startingPose : targets[targetIndex - 1].getFieldRelativePose();
          var current = RobotContainer.state.getPose();

          if (ShouldAdvanceToNextTarget(current, target, lastTarget)) {
            if (targetIndex >= targets.length - 1) {
              onFinalTarget = true;
            } else {
              targetIndex++;
            }
          }
          return targets[targetIndex].getFieldRelative();
        }, true, () -> preciseFinish && onFinalTarget).onlyWhile(() -> !onFinalTarget || preciseFinish)
    ).beforeStarting(() -> {
      targetIndex = 0;
      onFinalTarget = false;
      startingPose = state.getPose();
    });
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

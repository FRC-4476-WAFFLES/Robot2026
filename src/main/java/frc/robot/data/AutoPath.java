// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoPath {
  private int targetIndex = 0;
  private boolean canFinish = false;
  private final Command cmd;

  public AutoPath(boolean flipped, double velocity, BlueRelativeTarget... targets) {
    cmd = followPath(flipped, velocity, targets);
  }

  public Command follow() {
    return cmd;
  }

  private Command followPath(boolean flipped, double velocity, BlueRelativeTarget... targets) {
    // Command[] commands = new Command[targets.length + 1];
    // Commands.runOnce(() -> {
    for (BlueRelativeTarget target : targets) {
      target.withVelocity(velocity);
    }
    // });

    // for (int i = 1; i < targets.length + 1; i++) {
    // commands[i] = DriveCommands.autoToPose(targets[i - 1],
    // RobotContainer.state.pathAutopilot());
    // }

    var state = RobotContainer.state;
    return Commands.sequence(
        DriveCommands.autoToFieldPose(() -> {
          if (state.pathAutopilot().atTarget(RobotContainer.state.getPose(), targets[targetIndex].getFieldRelative())) {
            if (targetIndex >= targets.length - 1) {
              canFinish = true;
            } else {
              targetIndex++;
            }
          }
          return targets[targetIndex].getFieldRelative();
        }, true, () -> canFinish)
    ).beforeStarting(() -> {
      targetIndex = 0;
      canFinish = false;
    });
  }
}

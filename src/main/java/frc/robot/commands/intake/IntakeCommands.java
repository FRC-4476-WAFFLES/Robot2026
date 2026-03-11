// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class IntakeCommands {
  public static Command intakeCommand() {
    return Commands.runEnd(
        () -> RobotContainer.state.setIntaking(true),
        () -> RobotContainer.state.setIntaking(false)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake.ExpanderState;

public class IntakeCommands {
  public static Command intakeCommand() {
    return Commands.startEnd(
        () -> {
          RobotContainer.intake.setIntakeSetpoint(IntakeConstants.INTAKE_SPEED);
          RobotContainer.state.setExpanderState(ExpanderState.INTAKING);
        },
        () -> {
          RobotContainer.intake.setIntakeSetpoint(0);
          RobotContainer.state.setExpanderState(ExpanderState.EXTENDED);
        }, RobotContainer.intake);
  }
}

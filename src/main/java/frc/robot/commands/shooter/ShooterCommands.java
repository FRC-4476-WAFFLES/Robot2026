// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.data.Constants.SpindexerConstants.IndexerState;

public class ShooterCommands {
  public static Command shootCommand() {
    return Commands.parallel(
        Commands.sequence(
            Commands.waitUntil(() -> RobotContainer.state.joysticksFree() && !RobotContainer.state.autonomousEnabled()),
            DriveCommands.stopWithX(RobotContainer.drive).until(() -> !RobotContainer.state.joysticksFree())
                .withName("Lock Wheels").asProxy()
        ).repeatedly(),
        RobotContainer.indexer.runIndexerCommand(IndexerState.RUN)
    )
        .beforeStarting(() -> RobotContainer.state.setShooting(true))
        .finallyDo(() -> RobotContainer.state.setShooting(false))
        .withName("Fire shot");
  }

  public static Command shootAutoCommand(double delay) {
    Timer timer = new Timer();
    return Commands.parallel(
        shootCommand(),
        Commands.sequence(
            Commands.waitUntil(() -> timer.get() > delay),
            Commands.runOnce(() -> RobotContainer.state.setForceIntakeIn(true))
        )
    )
        .beforeStarting(() -> timer.restart())
        .finallyDo(() -> RobotContainer.state.setForceIntakeIn(false));
  }

  public static Command backoffIndexer() {
    return Commands.startEnd(
        () -> RobotContainer.indexer.runIndexer(IndexerState.REVERSE),
        () -> RobotContainer.indexer.stopIndexer(),
        RobotContainer.indexer)
        .withTimeout(0.25)
        .withName("Backoff Indexer");
  }
}

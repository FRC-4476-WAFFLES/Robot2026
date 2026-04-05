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
  private static final double AGITATION_CYCLE_TIME = 2;

  public static Command shootCommand() {
    Timer agitationTimer = new Timer();

    return Commands.parallel(
        Commands.sequence(
            Commands.waitUntil(() -> RobotContainer.state.joysticksFree() && !RobotContainer.state.autonomousEnabled()),
            DriveCommands.stopWithX(RobotContainer.drive).until(() -> !RobotContainer.state.joysticksFree())
                .withName("Lock Wheels").asProxy()
        ).repeatedly(),
        Commands.run(() -> {
          if (agitationTimer.get() % AGITATION_CYCLE_TIME < 1.2) {
            RobotContainer.indexer.runIndexer(IndexerState.RUN);
          } else {
            RobotContainer.indexer.runIndexer(IndexerState.RUNSLOW);
          }

        })
    )
        .beforeStarting(() -> RobotContainer.state.setShooting(true))
        .beforeStarting(() -> agitationTimer.restart())
        .finallyDo(() -> RobotContainer.state.setShooting(false))
        .withName("Fire shot");
  }

  public static Command shootAutoCommand(double delay) {
    Timer timer = new Timer();
    return Commands.parallel(
        Commands.repeatingSequence( // Handle turret wrap event
            shootCommand().until(() -> !RobotContainer.turret.atGoal()),
            backoffIndexer().until(() -> RobotContainer.turret.atGoal())
        ),
        Commands.sequence(
            Commands.waitUntil(() -> timer.get() > delay),
            Commands.runOnce(() -> RobotContainer.state.setForceIntakeIn(true))
        )
    )
        .beforeStarting(() -> timer.restart())
        .finallyDo(() -> {
          RobotContainer.state.setForceIntakeIn(false);
          RobotContainer.indexer.stopIndexer();
        });
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

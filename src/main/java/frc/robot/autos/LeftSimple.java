// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState.ShooterState;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class LeftSimple extends SequentialCommandGroup {
  private final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));
  private final BlueRelativeTarget point1 = new BlueRelativeTarget(6.1, 5.8, Rotation2d.fromDegrees(-10))
      .withExitVelocity(2);
  private final BlueRelativeTarget point2 = new BlueRelativeTarget(7.4, 6.9, Rotation2d.fromDegrees(-45));
  private final BlueRelativeTarget point3 = new BlueRelativeTarget(7.55, 4.5, Rotation2d.fromDegrees(-90))
      .withMaxVelocity(1.5);
  private final BlueRelativeTarget point4 = new BlueRelativeTarget(5.0, 5.4, Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(0.7);
  private final BlueRelativeTarget end = new BlueRelativeTarget(0.65, 5.95, Rotation2d.fromDegrees(180));

  public LeftSimple() {
    AutoPath collectBalls = new AutoPath(point2, point3, point4, end)
        .withPreciseFinish();

    addCommands(
        AutoUtils.resetOdometry(start),

        DriveCommands.passThroughTarget(point1),

        Commands.parallel(
            Commands.sequence(
                IntakeCommands.intakeCommand()
                    .until(() -> RobotContainer.state.getShooterState() == ShooterState.TARGET_HUB &&
                        RobotContainer.state.notMoving()),
                ShooterCommands.shootAutoCommand(8)
            ),
            collectBalls.follow()
        )
    );
  }
}

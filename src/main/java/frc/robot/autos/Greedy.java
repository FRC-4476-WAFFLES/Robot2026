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

public class Greedy extends SequentialCommandGroup {
  public final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));
  public final BlueRelativeTarget point1 = new BlueRelativeTarget(5.9, 5.8, Rotation2d.fromDegrees(-10))
      .withExitVelocity(2);
  public final BlueRelativeTarget point2 = new BlueRelativeTarget(7.65, 7, Rotation2d.fromDegrees(-80));
  public final BlueRelativeTarget point3 = new BlueRelativeTarget(7.74, 3.4, Rotation2d.fromDegrees(-90))
      .withMaxVelocity(1.5);
  public final BlueRelativeTarget point4 = new BlueRelativeTarget(5.0, 5.4, Rotation2d.fromDegrees(0))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(1.5);
  public final BlueRelativeTarget end = new BlueRelativeTarget(3.2, 5.4, Rotation2d.fromDegrees(0));

  public Greedy() {
    // AutoPath pathTest = new AutoPath(point1, point2, point3, point4, point5);
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
                ShooterCommands.shootCommand()
            ),
            collectBalls.follow()
        )
    );
  }
}

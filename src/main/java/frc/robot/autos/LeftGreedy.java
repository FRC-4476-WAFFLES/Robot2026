// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class LeftGreedy extends SequentialCommandGroup {
  private final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));
  private final BlueRelativeTarget point1 = new BlueRelativeTarget(5.9, 5.8, Rotation2d.fromDegrees(-10))
      .withExitVelocity(3.5);
  private final BlueRelativeTarget point2 = new BlueRelativeTarget(7.9, 6.2, Rotation2d.fromDegrees(-90));
  private final BlueRelativeTarget point3 = new BlueRelativeTarget(7.6, 4.5, Rotation2d.fromDegrees(-90))
      .withMaxVelocity(1.7);
  private final BlueRelativeTarget point4 = new BlueRelativeTarget(6.8, 4.0, Rotation2d.fromDegrees(-160)) // Mid swing
                                                                                                           // point
      .withEntryAngle(Rotation2d.fromDegrees(-160))
      .withMaxVelocity(1.5);
  private final BlueRelativeTarget point5 = new BlueRelativeTarget(6.0, 3.7, Rotation2d.fromDegrees(90))
      .withEntryAngle(Rotation2d.fromDegrees(90))
      .withMaxVelocity(1.5);
  private final BlueRelativeTarget point6 = new BlueRelativeTarget(5.0, 5.4,
      Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(0.7);
  private final BlueRelativeTarget end = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(180));

  private final BlueRelativeTarget readyForNextPass = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(0))
      .withMaxRotationRate(2);

  public LeftGreedy() {
    AutoPath collectBalls = new AutoPath(point1, point2, point3, point4, point5, point6, end)
        .withPreciseFinish();
    AutoPath secondPass = new AutoPath(point1, point2, point3, point4, point5, point6, end)
        .withPreciseFinish();

    addCommands(
        AutoUtils.resetOdometry(start),

        Commands.deadline(
            collectBalls.follow(),
            IntakeCommands.intakeCommand()
        ),

        Commands.parallel(
            ShooterCommands.shootAutoCommand(4).withTimeout(9),
            DriveCommands.autoToTarget(readyForNextPass)
        ),

        Commands.deadline(
            secondPass.follow(),
            IntakeCommands.intakeCommand()
        )
    );
  }
}

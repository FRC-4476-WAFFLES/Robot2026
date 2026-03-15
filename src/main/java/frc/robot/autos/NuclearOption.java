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

public class NuclearOption extends SequentialCommandGroup {
  private final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));
  private final BlueRelativeTarget middle = new BlueRelativeTarget(8.2, 5.6, Rotation2d.fromDegrees(-2.5))
      .withExitVelocity(3);
  private final BlueRelativeTarget sweep = new BlueRelativeTarget(8.2, 4.3, Rotation2d.fromDegrees(-90));
  private final BlueRelativeTarget point3 = new BlueRelativeTarget(6.691, 4.3, Rotation2d.fromDegrees(180));
  private final BlueRelativeTarget point4 = new BlueRelativeTarget(6.213, 5.516, Rotation2d.fromDegrees(112));
  private final BlueRelativeTarget end = new BlueRelativeTarget(0.60, 5.95, Rotation2d.fromDegrees(180));

  public NuclearOption() {
    AutoPath collectPath = new AutoPath(sweep, point3, point4, end)
        .withPreciseFinish();

    addCommands(
        AutoUtils.resetOdometry(start),

        DriveCommands.passThroughTarget(middle),

        Commands.parallel(
            Commands.sequence(
                IntakeCommands.intakeCommand()
                    .until(() -> RobotContainer.state.getShooterState() == ShooterState.TARGET_HUB),
                Commands.waitSeconds(1),
                ShooterCommands.shootAutoCommand(8)
            ),
            collectPath.follow()
        )
    );
  }
}

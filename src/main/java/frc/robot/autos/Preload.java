// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class Preload extends SequentialCommandGroup {
  BlueRelativeTarget start = new BlueRelativeTarget(3.570, 4, Rotation2d.fromDegrees(0));
  BlueRelativeTarget end = new BlueRelativeTarget(2.5, 4, Rotation2d.fromDegrees(0));

  public Preload() {
    addCommands(
        AutoUtils.resetOdometry(start),
        DriveCommands.autoToTarget(end),
        ShooterCommands.shootCommand()
    );
  }
}

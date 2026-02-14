// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.data.AutoCoordinates;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class Olympic extends SequentialCommandGroup {
  public Olympic(boolean left) {
    addCommands(
        AutoUtils.resetOdometry(AutoCoordinates.Example),
        DriveCommands.autoToPose(AutoCoordinates.Test),
        DriveCommands
            .autoToPose(new BlueRelativeTarget(AutoCoordinates.Example).withEntryAngle(Rotation2d.kCW_90deg))
    );
  }
}

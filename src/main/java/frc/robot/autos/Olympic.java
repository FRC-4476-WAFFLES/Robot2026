// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.lib.AutoPath;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class Olympic extends SequentialCommandGroup {
  // Targets
  public static final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.882, Rotation2d.kZero);
  public static final BlueRelativeTarget point1 = new BlueRelativeTarget(6.1, 6.067, Rotation2d.fromDegrees(80))
      .withVelocity(1);
  public static final BlueRelativeTarget point2 = new BlueRelativeTarget(7.65, 6.606, Rotation2d.fromDegrees(10));
  public static final BlueRelativeTarget point3 = new BlueRelativeTarget(7.74, 3.4, Rotation2d.fromDegrees(0));
  public static final BlueRelativeTarget point4 = new BlueRelativeTarget(5.0, 5.125, Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withVelocity(1.5);
  public static final BlueRelativeTarget point5 = new BlueRelativeTarget(2.950, 5.1, Rotation2d.fromDegrees(180))
      .withVelocity(2);
  public static final BlueRelativeTarget end = new BlueRelativeTarget(1.035, 4.747, Rotation2d.fromDegrees(180));

  // Paths
  public static final AutoPath pathTest = new AutoPath(false, false, point1, point2, point3, point4, point5);
  public static final AutoPath collectBalls = new AutoPath(false, true, point2, point3, point4, point5, end);

  public Olympic(boolean left) {
    addCommands(
        AutoUtils.resetOdometry(start),
        Commands.waitSeconds(1),

        DriveCommands.autoToPose(point1, CodeConstants.AUTO_POSITION_TOLERANCE_VAGUE),

        RobotContainer.intake.extend(),

        collectBalls.follow()
    );
  }
}

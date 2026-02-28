// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class NotOlympic extends SequentialCommandGroup {
  // Targets (headings adjusted -90 for front-of-robot orientation fix)
  public final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.882, Rotation2d.fromDegrees(-90));
  public final BlueRelativeTarget point1 = new BlueRelativeTarget(6.1, 6.067, Rotation2d.fromDegrees(-10))
      .withVelocity(2);
  public final BlueRelativeTarget point2 = new BlueRelativeTarget(7.65, 6.606, Rotation2d.fromDegrees(-80));
  public final BlueRelativeTarget point3 = new BlueRelativeTarget(7.74, 3.4, Rotation2d.fromDegrees(-90));
  public final BlueRelativeTarget point4 = new BlueRelativeTarget(5.0, 5.125, Rotation2d.fromDegrees(90))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withVelocity(1.5);
  public final BlueRelativeTarget point5 = new BlueRelativeTarget(2.950, 5.1, Rotation2d.fromDegrees(90))
      .withVelocity(2);
  public final BlueRelativeTarget end = new BlueRelativeTarget(1.035, 4.747, Rotation2d.fromDegrees(90));

  public NotOlympic() {
    // AutoPath pathTest = new AutoPath(point1, point2, point3, point4, point5);
    AutoPath collectBalls = new AutoPath(point2, point3, point4, point5, end)
        .withPreciseFinish();

    addCommands(
        AutoUtils.resetOdometry(start),

        DriveCommands.passThroughTarget(point1),

        RobotContainer.intake.extend(),

        collectBalls.follow()
    );
  }
}

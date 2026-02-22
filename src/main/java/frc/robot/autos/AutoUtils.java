// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.lib.WafflesUtilities;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoUtils {
  public static Command resetOdometry(Pose2d pose) {
    return resetOdometry(() -> pose, false);
  }

  public static Command resetOdometry(BlueRelativeTarget target) {
    return resetOdometry(() -> target.getFieldRelativePose(), false);
  }

  public static Command resetOdometry(Supplier<Pose2d> pose, boolean flipPose) {
    if (!CodeConstants.RESET_ODOMETRY_AUTO_START) {
      return new InstantCommand();
    }
    return Commands.runOnce(() -> {
      RobotContainer.drive.setPose(
          flipPose ? WafflesUtilities.FlipIfRedAlliance(pose.get()) : pose.get()
      );
    });
  }

  public static void mirrorTargets(BlueRelativeTarget... targets) {
    for (BlueRelativeTarget target : targets) {
      target.mirror();
    }
  }
}

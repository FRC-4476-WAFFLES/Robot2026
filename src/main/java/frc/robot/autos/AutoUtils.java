// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.lib.WafflesUtilities;

public class AutoUtils {
        public static Command resetOdometry(Pose2d instantPose) {
                if (!CodeConstants.RESET_ODOMETRY_AUTO_START) {
                        return new InstantCommand();
                }
                return new InstantCommand(() -> {
                        RobotContainer.driveSubsystem.setPose(
                                        WafflesUtilities.FlipIfRedAlliance(instantPose)
                        );
                });
        }
}

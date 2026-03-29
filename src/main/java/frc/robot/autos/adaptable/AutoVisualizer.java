// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoPath;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoVisualizer {

  public static void VisualizeAuto(BlueRelativeTarget start, List<BlueRelativeTarget> targets) {
    var poseList = new ArrayList<Pose2d>();
    if (CodeConstants.COMPLEX_AUTO_PREVIEW) {
      poseList = simulatePath(start, targets);
    } else {
      poseList = targets.stream().map(target -> target.getFieldRelativePose())
          .collect(Collectors.toCollection(ArrayList::new));
      poseList.add(0, start.getFieldRelativePose());

      while (poseList.size() < 9) {
        // need to extent to min 8 elements for silly elastic reasons
        poseList.add(poseList.get(poseList.size() - 1));
      }
    }

    RobotContainer.telemetry.dashboardField.getObject("trajectory_autoPath")
        .setPoses(poseList);
  }

  private static ArrayList<Pose2d> simulatePath(BlueRelativeTarget start, List<BlueRelativeTarget> targets) {
    ChassisSpeeds lastOutput = new ChassisSpeeds();
    Pose2d simulatedRobotPose = start.getFieldRelativePose();
    simulatedRobotPose = new Pose2d(simulatedRobotPose.getTranslation(), Rotation2d.kZero);

    var finalTarget = targets.get(targets.size() - 1).getFieldRelative();
    int iter = 0;

    var poseList = new ArrayList<Pose2d>(500);

    int targetIndex = 0;
    int poseListThrottle = 0;

    while (!(simulatedRobotPose.getTranslation().getDistance(finalTarget.getReference().getTranslation()) < 0.1)) {
      var target = targets.get(targetIndex).getFieldRelativePose();
      var lastTarget = targetIndex == 0 ? start.getFieldRelativePose()
          : targets.get(targetIndex - 1).getFieldRelativePose();

      if (AutoPath.ShouldAdvanceToNextTarget(simulatedRobotPose, target, lastTarget)) {
        if (!(targetIndex >= targets.size() - 1)) {
          targetIndex++;
        }
      }

      APResult output = RobotContainer.state.autopilot().calculate(simulatedRobotPose, lastOutput,
          targets.get(targetIndex).getFieldRelative());

      lastOutput.vxMetersPerSecond = output.vx().in(MetersPerSecond);
      lastOutput.vyMetersPerSecond = output.vy().in(MetersPerSecond);

      // Add one in 3 simulated poses
      if (poseListThrottle == 2) {
        poseList.add(simulatedRobotPose);
        poseListThrottle = 0;
      } else {
        poseListThrottle++;
      }

      simulatedRobotPose = simulatedRobotPose.plus(
          new Transform2d(lastOutput.vxMetersPerSecond * 0.02, lastOutput.vyMetersPerSecond * 0.02, Rotation2d.kZero));

      iter++;
      if (iter > 1000) {
        break;
      }
    }

    return poseList;
  }

  public static void ClearVisualizer() {
    RobotContainer.telemetry.dashboardField.getObject("trajectory_autoPath")
        .setPoses(new Pose2d[0]);
  }
}

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class MechanismPoses extends VirtualSubsystem {
  private Pose3d[] poseArray = new Pose3d[2];

  private Transform3d turretRestPose = new Transform3d(-0.09842500, 0.25082500, 0.5509895, Rotation3d.kZero);

  private Transform3d intakeRestPose = new Transform3d(0, -0.2, 0.2032, Rotation3d.kZero);

  public MechanismPoses() {
    Logger.recordOutput("Telemetry/Zero", Pose3d.kZero);
  }

  @Override
  public void latePeriodic() {
    turretRestPose = new Transform3d(0.25082500, 0.09842500, 0.5509895, Rotation3d.kZero);
    Pose3d turretPose = new Pose3d(
        turretRestPose.getTranslation(),
        new Rotation3d(0, 0, Units.degreesToRadians(45))
    );
    poseArray[0] = turretPose;

    // Create ground intake pose - rotates around Y axis
    Pose3d groundIntakePose = new Pose3d(
        intakeRestPose.getTranslation(),
        new Rotation3d(Units.degreesToRadians(125), 0, 0)
    );
    poseArray[1] = groundIntakePose;

    Logger.recordOutput("Telemetry/MechanismPoses", poseArray);
  }
}
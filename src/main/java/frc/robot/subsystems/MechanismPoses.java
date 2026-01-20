package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class MechanismPoses extends VirtualSubsystem {
  private Pose3d[] poseArray = new Pose3d[2];

  public MechanismPoses() {
    Logger.recordOutput("Telemetry/Zero", Pose3d.kZero);
  }

  @Override
  public void latePeriodic() {
    Pose3d turretPose = new Pose3d(
        Translation3d.kZero,
        new Rotation3d(0, 0, Units.degreesToRadians(45))
    );
    poseArray[0] = turretPose;

    // Create ground intake pose - rotates around Y axis
    Pose3d groundIntakePose = new Pose3d(
        Translation3d.kZero,
        new Rotation3d(0, Units.degreesToRadians(45), 0)
    );
    poseArray[1] = groundIntakePose;

    Logger.recordOutput("Telemetry/MechanismPoses", poseArray);
  }
}
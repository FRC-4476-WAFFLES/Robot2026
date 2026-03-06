package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.TurretConstants;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class MechanismPoses extends VirtualSubsystem {
  private Pose3d[] poseArray = new Pose3d[4];

  private Transform3d turretRestPose = PhysicalConstants.ROBOT_TO_TURRET_CENTER;

  private Transform3d intakeRestPose = new Transform3d(0.2, 0, 0.2032, Rotation3d.kZero);

  private Transform3d hoodRestPoseTurretSpace = new Transform3d(0.10001250, 0, 0.09048750,
      new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(21.74), 0));

  private Transform3d climbRestPose = new Transform3d(0.2, 0, 0.2032, Rotation3d.kZero);

  public MechanismPoses() {
    Logger.recordOutput("RobotState/Zero", Pose3d.kZero);
  }

  @Override
  public void latePeriodic() {
    Pose3d turretPose = new Pose3d(
        turretRestPose.getTranslation(),
        new Rotation3d(0, 0,
            Rotation2d.fromRotations(RobotContainer.turret.getMechanismRelativePosition())
                .plus(TurretConstants.PHYSICAL_ZERO).getRadianvtrsys()
        )
    );
    poseArray[0] = turretPose;

    // Create ground intake pose (intake on +X side, swings around Y axis)
    Pose3d groundIntakePose = new Pose3d(
        intakeRestPose.getTranslation(),
        new Rotation3d(0, Units.rotationsToRadians(RobotContainer.intake.getExpanderPosition()), 0)
    );
    poseArray[1] = groundIntakePose;

    Pose3d hoodPose = turretPose.plus(new Transform3d(hoodRestPoseTurretSpace.getTranslation(),
        new Rotation3d(0, Units.degreesToRadians(-RobotContainer.hood.getPosition()), 0)
            .plus(hoodRestPoseTurretSpace.getRotation())));
    poseArray[2] = hoodPose;

    Pose3d climbPose = new Pose3d(
        climbRestPose.getTranslation().plus(new Translation3d(-RobotContainer.climber.getPosition(), 0, 0)),
        climbRestPose.getRotation()
    );
    poseArray[3] = climbPose;

    Logger.recordOutput("RobotState/MechanismPoses", poseArray);
  }
}
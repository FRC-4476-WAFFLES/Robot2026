package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoCoordinates {
  // Positions shared between autos (headings adjusted -90 for front-of-robot orientation fix)
  public static final Pose2d Example = new Pose2d(7.176, 2.992, Rotation2d.fromDegrees(90.000));
  public static final Pose2d Test = new Pose2d(4.176, 4.992, Rotation2d.fromDegrees(0.000));

}

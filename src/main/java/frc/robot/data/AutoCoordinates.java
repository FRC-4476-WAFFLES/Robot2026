package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoCoordinates {
  // Starting positions
  public static final Pose2d Example = new Pose2d(7.176, 2.992, Rotation2d.fromDegrees(180.000));
  public static final Pose2d Test = new Pose2d(4.176, 4.992, Rotation2d.fromDegrees(90.000));

  public static class OlympicAuto {
    public static final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.882, Rotation2d.kZero);
    public static final BlueRelativeTarget point1 = new BlueRelativeTarget(5.912, 6.067, Rotation2d.fromDegrees(10));
    public static final BlueRelativeTarget point2 = new BlueRelativeTarget(7.216, 6.606, Rotation2d.fromDegrees(5));
    public static final BlueRelativeTarget point3 = new BlueRelativeTarget(7.747, 3.138, Rotation2d.fromDegrees(0));
    public static final BlueRelativeTarget point4 = new BlueRelativeTarget(5.574, 5.125, Rotation2d.fromDegrees(180));
    // .withEntryAngle(Rotation2d.fromDegrees(-180));
    public static final BlueRelativeTarget point5 = new BlueRelativeTarget(2.950, 4.965, Rotation2d.fromDegrees(180));
    public static final BlueRelativeTarget end = new BlueRelativeTarget(1.035, 4.747, Rotation2d.fromDegrees(180));

    public static final AutoPath pathTest = new AutoPath(false, 0, OlympicAuto.point1, OlympicAuto.point2,
        OlympicAuto.point3,
        OlympicAuto.point4, OlympicAuto.point5);
  }
}

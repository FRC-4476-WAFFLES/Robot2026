package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.lib.WafflesUtilities;

/** 
 * Seperates controls a bit from RobotContainer while grouping controls specific constants together 
 */
public class Controls {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandJoystick leftJoystick = new CommandJoystick(DriverConstants.kLeftJoystickPort);
  public static final CommandJoystick rightJoystick = new CommandJoystick(DriverConstants.kRightJoystickPort);
  public static final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  public static final CommandGenericHID simController = new CommandGenericHID(3);

  // Constants
  private static final double JOYSTICK_DEADZONE_INNER = 0.025; // Below the inner value the input is zero
  private static final double JOYSTICK_DEADZONE_OUTER = 0.15; // Between the inner and outer value the input is
                                                              // interpolated towards it's actual value
  public static final double AXIS_DEADBAND = 0.1; // Deadband for controller axes to prevent unintended activation
  public static final double MANUAL_ELEVATOR_CONTROL_MULTIPLIER = 2;
  public static final boolean SQUARE_JOYSTICK_FILTER = true;

  /* Triggers */
  /*
   * When triggers are referenced in multiple places, they are defined here to
   * have a single source of truth
   */
  public static final Trigger shootButton = Controls.rightJoystick.button(1);

  public static class DriverConstants {
    // public static final int kDriverControllerPort = 0;
    public static final int kLeftJoystickPort = 0;
    public static final int kRightJoystickPort = 1;
  }

  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 2;
  }

  public static double getDriveXRaw() {
    return -filterJoystick(leftJoystick.getX());
  }

  public static double getDriveYRaw() {
    // Negated since field coordinate system from blue alliance perspective is
    // positive to the left (?)
    return filterJoystick(leftJoystick.getY());
  }

  public static double getDriveRotationRaw() {
    return filterJoystick(rightJoystick.getX());
  }

  /* Methods to get operator input */
  public static double getOperatorRightY() {
    return operatorController.getRightY();
  }

  public static double getOperatorRightX() {
    return operatorController.getRightX();
  }

  /* Util */

  // Can optionally add more filtering. Used to be some here.
  public static double filterJoystick(double input) {
    return applyDeadzone(input);
  }

  // Smooths deadzone over range
  public static double applyDeadzone(double input) {
    return Math.abs(input) > JOYSTICK_DEADZONE_OUTER ? input
        : (Math.abs(input) < JOYSTICK_DEADZONE_INNER ? 0
            : (WafflesUtilities.Lerp(0, input,
                WafflesUtilities.InvLerp(JOYSTICK_DEADZONE_INNER, JOYSTICK_DEADZONE_OUTER, input))
                * Math.signum(input)));
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    if (SQUARE_JOYSTICK_FILTER) {
      // Square magnitude for more precise control
      linearMagnitude = linearMagnitude * linearMagnitude;
    }

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }
}

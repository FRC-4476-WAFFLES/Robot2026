package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.lib.WafflesUtilities;

/** 
 * Seperates controls a bit from RobotContainer while grouping controls specific constants together 
 */
public class Controls {
  public static final CommandXboxController driverController = new CommandXboxController(
      DriverConstants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  public static final CommandGenericHID simController = new CommandGenericHID(3);

  // Constants
  // Worth revisiting now the driver is on a gamepad: a thumbstick has more slop
  // at centre than a flight stick, so these may want raising.
  private static final double JOYSTICK_DEADZONE_INNER = 0.025; // Below the inner value the input is zero
  private static final double JOYSTICK_DEADZONE_OUTER = 0.15; // Between the inner and outer value the input is
                                                              // interpolated towards it's actual value
  public static final double AXIS_DEADBAND = 0.1; // Deadband for controller axes to prevent unintended activation
  public static final double MANUAL_ELEVATOR_CONTROL_MULTIPLIER = 2;
  public static final boolean SQUARE_JOYSTICK_FILTER = false;

  /* Triggers */
  /*
   * When triggers are referenced in multiple places, they are defined here to
   * have a single source of truth
   */
  public static final Trigger shootButton = driverController.rightTrigger().or(operatorController.rightTrigger());
  public static final Trigger beachButton = operatorController.rightBumper();

  public static class DriverConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 1;
  }

  /*
   * The three negations are all the same convention, not a mistake: both a
   * flight stick and a gamepad report right and *down* as positive, while the
   * drivetrain wants forward, left and counterclockwise as positive.
   */
  public static double getDriveXRaw() {
    return -filterJoystick(driverController.getLeftX());
  }

  public static double getDriveYRaw() {
    return -filterJoystick(driverController.getLeftY());
  }

  public static double getDriveRotationRaw() {
    return -filterJoystick(driverController.getRightX());
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
    return applyDeadzone(input, JOYSTICK_DEADZONE_OUTER, JOYSTICK_DEADZONE_INNER);
  }

  public static double applyDeadzone(double input, double OuterDeadzone, double InnerDeadzone) {
    return Math.abs(input) > OuterDeadzone ? input
        : (Math.abs(input) < InnerDeadzone ? 0
            : (WafflesUtilities.Lerp(0, input,
                WafflesUtilities.InvLerp(InnerDeadzone, OuterDeadzone, input))
                * Math.signum(input)));
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
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

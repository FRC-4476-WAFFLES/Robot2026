package frc.robot;

import edu.wpi.first.math.MathUtil;
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

  /*
   * How much of the stick's travel is spent on fine control. Zero is linear, one
   * is fully cubic; in between is a blend of the two. Full deflection always
   * gives full speed whatever this is set to, so raising it costs precision at
   * the top of the range rather than top speed.
   *
   * This matters more on a gamepad than it did on flight sticks. A thumbstick has
   * perhaps a centimetre of travel where a flight stick had several, so the same
   * fraction of output arrives in a much smaller movement of the hand.
   *
   * Rotation defaults higher because it is the twitchier of the two, and 0.7 is
   * close to the squaring that was previously hard-coded into both drive
   * commands — so rotation should feel unchanged and translation should feel
   * calmer than before.
   */
  public static final double TRANSLATION_CURVE = 0.5;
  public static final double ROTATION_CURVE = 0.7;

  /*
   * Stick deflection past which the driver is taken to mean "all of it".
   *
   * A thumbstick often cannot quite reach 1.0 — least of all on a diagonal, and
   * less so as it wears — so without this the driver can be pushing as hard as
   * the stick goes and still not get full speed. Team 581 uses the same 0.95.
   */
  public static final double UPPER_DEADBAND = 0.95;

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

  /**
   * Blends the input between linear and cubic, after treating anything past
   * {@link #UPPER_DEADBAND} as full deflection. Zero returns it unchanged
   * (bar that rescaling), one cubes it, and anything between mixes the two.
   *
   * <p>
   * Cubic rather than squared because it keeps the sign without needing
   * {@code copySign}, and blended rather than switched so the amount of softening
   * is a number somebody can turn rather than a choice between two behaviours.
   * Both ends are fixed: zero maps to zero and one maps to one, so no top speed
   * is given up.
   */
  public static double applyCurve(double input, double curve) {
    double scaled = MathUtil.clamp(input / UPPER_DEADBAND, -1.0, 1.0);
    return curve * scaled * scaled * scaled + (1 - curve) * scaled;
  }

  /** Shapes a rotation stick input. Separate from translation so it can be tuned apart. */
  public static double applyRotationCurve(double input) {
    return applyCurve(input, ROTATION_CURVE);
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Shape the magnitude rather than the axes, so the curve does not depend on
    // which way the stick is pushed. The clamp matters: a square-gated stick
    // reads 1.414 at full diagonal, and cubing that would ask for 2.4 times full
    // speed.
    linearMagnitude = applyCurve(Math.min(1.0, linearMagnitude), TRANSLATION_CURVE);

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Aiming a shooter bolted to a robot that is not level.
 *
 * <p>
 * Tilt does exactly one thing to a shot: it rotates the direction the ball
 * leaves in. Once the ball is airborne only gravity acts on it, so the flight
 * itself is unchanged — the launch direction is the whole of the problem.
 *
 * <p>
 * Measured on the simulated bump, the robot sits 9.2° over and a shot aimed at
 * 3.00 m crosses the rim at 2.20 m. At 16° it lands at 1.38 m. That is not a
 * shot that misses narrowly; it falls a third to a half short.
 *
 * <p>
 * <b>The hood cannot fix it.</b> Its entire travel is 58° to 70°, twelve
 * degrees, and cancelling 9.2° from a 64° shot needs 54.8° — past the stop. The
 * tilt asks for more range than the hood owns.
 *
 * <p>
 * <b>The flywheel can.</b> Leaving the angle where it lands and solving for the
 * speed that still reaches the target costs 13.5% more speed at 9.2° and 40% at
 * 16°, against a wheel that runs from about 41 to 72 rps. So this solves what it
 * can with the hood, gives up gracefully when the hood runs out, and hands the
 * remainder to the flywheel.
 *
 * <p>
 * Tilt swings the bearing as well as the elevation, and one rotation handles
 * both — which is what the hand-tuned sideways offset table was standing in for.
 *
 * <p>
 * <b>Everything here depends on the gravity vector being honest.</b> An
 * uncalibrated Pigeon reads seven degrees of tilt sitting level, and fed that,
 * this applies a correction to a robot that does not need one, all the time.
 * The mount has to be calibrated before it is trusted on hardware.
 */
public final class TiltedShot {
  private static final double GRAVITY = 9.81;
  /** Below this the tilt is not worth a rotation and the maths gets singular. */
  private static final double LEVEL_TOLERANCE = 1e-6;

  private TiltedShot() {}

  /**
   * What to command, and what will actually happen when it is commanded.
   *
   * @param turretAzimuthRadians where to point the turret, in the robot's frame
   * @param hoodElevationDegrees what to ask the hood for, already clamped to its
   *     travel
   * @param launchElevationDegrees the elevation the ball will really leave at,
   *     once the tilt has had its way — this is what the speed must be solved
   *     for, and it equals the requested elevation only when the hood had the
   *     range to cancel the tilt
   * @param hoodWasClamped whether the hood ran out of travel, so the caller
   *     knows the flywheel is carrying the difference
   */
  public record Solution(
      double turretAzimuthRadians,
      double hoodElevationDegrees,
      double launchElevationDegrees,
      boolean hoodWasClamped
  ) {}

  /**
   * The rotation from the robot's frame to a level one with the same heading.
   *
   * @param gravity the gravity vector as the gyro reports it, in the robot's
   *     frame. It reads (0, 0, 1) when level, so this is the robot-frame
   *     direction of "up".
   */
  public static Rotation3d levelling(Translation3d gravity) {
    double norm = gravity.getNorm();
    if (norm < LEVEL_TOLERANCE) {
      return Rotation3d.kZero;
    }
    Translation3d up = gravity.div(norm);
    if (Math.abs(up.getZ() - 1.0) < LEVEL_TOLERANCE) {
      return Rotation3d.kZero;
    }
    return new Rotation3d(up.toVector(), new Translation3d(0, 0, 1).toVector());
  }

  /** Where a ball actually goes, given what the turret and hood were told. */
  public static Solution forward(Translation3d gravity, double turretAzimuthRadians,
      double hoodElevationDegrees) {
    Translation3d inRobot = direction(turretAzimuthRadians,
        Units.degreesToRadians(hoodElevationDegrees));
    Translation3d inField = inRobot.rotateBy(levelling(gravity));
    return new Solution(
        Math.atan2(inField.getY(), inField.getX()),
        hoodElevationDegrees,
        Units.radiansToDegrees(Math.asin(clamp(inField.getZ(), -1, 1))),
        false);
  }

  /**
   * What to command so the ball leaves where it is wanted.
   *
   * <p>
   * The turret is solved exactly — it has hundreds of degrees of travel and can
   * always be pointed. The hood is solved, then clamped, and the elevation the
   * ball will really leave at is recomputed from the clamped value, because a
   * hood held at its stop does not deliver what it was asked for and the speed
   * has to be solved for the truth rather than the request.
   *
   * @param gravity gyro gravity vector, robot frame
   * @param desiredAzimuthRadians the bearing the ball should leave along, in the
   *     robot's frame, as if the robot were level
   * @param desiredElevationDegrees the elevation it should leave at, likewise
   * @param hoodMinDegrees flattest elevation the hood can reach
   * @param hoodMaxDegrees steepest elevation the hood can reach
   */
  public static Solution solve(Translation3d gravity, double desiredAzimuthRadians,
      double desiredElevationDegrees, double hoodMinDegrees, double hoodMaxDegrees) {
    Rotation3d toLevel = levelling(gravity);
    Translation3d wanted = direction(desiredAzimuthRadians,
        Units.degreesToRadians(desiredElevationDegrees));
    // Back into the robot's own frame: this is the direction the barrel has to
    // point for the ball to leave along `wanted` once the tilt has rotated it.
    Translation3d inRobot = wanted.rotateBy(toLevel.unaryMinus());

    double azimuth = Math.atan2(inRobot.getY(), inRobot.getX());
    double elevation = Units.radiansToDegrees(Math.asin(clamp(inRobot.getZ(), -1, 1)));
    double clamped = clamp(elevation, hoodMinDegrees, hoodMaxDegrees);
    boolean hitStop = Math.abs(clamped - elevation) > 1e-9;

    if (!hitStop) {
      return new Solution(azimuth, clamped, desiredElevationDegrees, false);
    }
    // The hood could not do it, so find out what the shot will really be. The
    // turret still points where it was asked; only the elevation gave way.
    Solution actual = forward(gravity, azimuth, clamped);
    return new Solution(azimuth, clamped, actual.launchElevationDegrees(), true);
  }

  /**
   * The launch speed that reaches a target, at whatever angle you are stuck with.
   *
   * <p>
   * From {@code x = v cos0 t} and {@code z = h0 + v sin0 t - g t^2 / 2},
   * eliminating the time gives
   * {@code v = D / cos0 * sqrt(g / (2 (D tan0 - dh)))}.
   *
   * <p>
   * There is no solution when {@code D tan0 <= dh}: fired that steeply the ball
   * is already coming down before it has gone far enough, and no amount of speed
   * changes that. NaN is returned rather than a number, so a caller cannot
   * quietly use a shot that cannot exist.
   *
   * @param distance horizontal range to the target, metres
   * @param elevationDegrees the angle the ball actually leaves at
   * @param heightGain how much higher the target is than the muzzle, metres
   */
  public static double requiredSpeed(double distance, double elevationDegrees,
      double heightGain) {
    double theta = Units.degreesToRadians(elevationDegrees);
    double reach = distance * Math.tan(theta) - heightGain;
    if (reach <= 0 || distance <= 0) {
      return Double.NaN;
    }
    return distance / Math.cos(theta) * Math.sqrt(GRAVITY / (2 * reach));
  }

  /** Unit vector from a bearing and an elevation. */
  private static Translation3d direction(double azimuthRadians, double elevationRadians) {
    double horizontal = Math.cos(elevationRadians);
    return new Translation3d(
        horizontal * Math.cos(azimuthRadians),
        horizontal * Math.sin(azimuthRadians),
        Math.sin(elevationRadians));
  }

  private static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(high, value));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.vendor.FuelSim;

/**
 * Puts a ball in the air whenever the simulated robot shoots, so a shot can be
 * watched rather than inferred from numbers.
 *
 * <p>
 * The point is seeing the spread. A shot's range depends on flywheel speed and
 * hood angle, and those vary shot to shot — as the battery sags, as the wheel
 * recovers between balls, as the robot moves. Watching where a burst actually
 * lands shows that variance directly, where a graph of flywheel speed only
 * implies it.
 *
 * <p>
 * {@code FuelSim} already models the flight, the field and the hub, including
 * scoring. All this does is decide when a ball leaves and how fast, and hand it
 * over.
 *
 * <p>
 * <b>This is an approximation of the launch, not a calibrated model of it.</b>
 * Exit speed is taken as a fixed fraction of the flywheel's surface speed, which
 * is the usual way a compliant-wheel shooter behaves but is not measured for
 * this one. Trust it for "did that burst group tightly", not for "will this land
 * in the goal".
 */
public final class SimShooter {
  /**
   * Wheel surface speed that actually reaches the ball. A ball is squeezed and
   * slips, so it leaves at well under the wheel's surface speed; two thirds is a
   * common figure for a compliant wheel.
   */
  private static final double SLIP_FACTOR = 0.66;
  /** Flywheel wheel radius, in metres. */
  private static final double WHEEL_RADIUS = 0.051;
  /** Hood travel, in rotations, mapped onto its angular range. */
  private static final double HOOD_ROTATIONS_AT_MAX = 21.0;
  private static final double HOOD_MIN_DEGREES = 25.0;
  private static final double HOOD_MAX_DEGREES = 62.0;
  /** How long between balls while the feeder is running, in seconds. */
  private static final double SHOT_INTERVAL = 0.35;

  private static double lastShot = -1;
  private static int shotsFired = 0;
  private static boolean enabled = true;

  /**
   * Turns ball spawning off. {@code SimHarness} does this at boot: a test that
   * spins the flywheel wants to measure the flywheel, not to fill the field with
   * balls whose flight costs loop time it then has to wait through.
   */
  public static void setEnabled(boolean value) {
    enabled = value;
  }

  private SimShooter() {}

  /** Whether balls are being simulated at all. */
  public static boolean isEnabled() {
    return enabled;
  }

  /** How many balls have been launched since the robot booted. */
  public static int getShotsFired() {
    return shotsFired;
  }

  /** Fires a ball if the robot is shooting and one is due. Call every sim loop. */
  public static void update() {
    boolean shooting = enabled && RobotContainer.state.isShooting()
        && RobotContainer.flywheel.getGoalVelocity() > 1.0;
    Logger.recordOutput("SimShooter/Shots Fired", shotsFired);

    if (!shooting) {
      lastShot = -1;
      return;
    }
    double now = Timer.getTimestamp();
    if (lastShot > 0 && now - lastShot < SHOT_INTERVAL) {
      return;
    }
    lastShot = now;
    fire();
  }

  /** Launches one ball from wherever the turret is currently pointing. */
  private static void fire() {
    var robot = RobotContainer.state.getPose();
    var turretHeading = robot.getRotation()
        .plus(Rotation2d.fromRotations(RobotContainer.turret.getMechanismRelativePosition()));

    double hoodDegrees = hoodAngleDegrees();
    double speed = exitSpeed();

    // Where the ball leaves: the turret's centre, at shooter height.
    Translation3d origin = new Translation3d(robot.getX(), robot.getY(), 0)
        .plus(new Translation3d(
            PhysicalConstants.ROBOT_TO_TURRET_CENTER.getX(),
            PhysicalConstants.ROBOT_TO_TURRET_CENTER.getY(),
            PhysicalConstants.ROBOT_TO_TURRET_CENTER.getZ())
            .rotateBy(new Rotation3d(0, 0, robot.getRotation().getRadians())));

    double pitch = Units.degreesToRadians(hoodDegrees);
    Translation3d velocity = new Translation3d(
        speed * Math.cos(pitch) * turretHeading.getCos(),
        speed * Math.cos(pitch) * turretHeading.getSin(),
        speed * Math.sin(pitch));

    // The robot's own motion carries into the shot, which is the whole reason
    // shoot-on-move exists. Leaving it out would make the simulation flatter
    // than reality in exactly the case worth watching.
    var robotVelocity = RobotContainer.state.getFieldVelocity();
    velocity = velocity.plus(new Translation3d(
        robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond, 0));

    FuelSim.getInstance().spawnFuel(origin, velocity);
    shotsFired++;

    Logger.recordOutput("SimShooter/Last Shot", new Pose3d(origin,
        new Rotation3d(0, -pitch, turretHeading.getRadians())));
    Logger.recordOutput("SimShooter/Exit Speed", speed);
    Logger.recordOutput("SimShooter/Hood Degrees", hoodDegrees);
    Logger.recordOutput("SimShooter/Predicted Range", predictedRange(speed, pitch, origin.getZ()));
  }

  /** Ball speed leaving the shooter, from the wheel's present surface speed. */
  private static double exitSpeed() {
    double wheelRps = Math.abs(RobotContainer.flywheel.getVelocity());
    return wheelRps * 2 * Math.PI * WHEEL_RADIUS * SLIP_FACTOR;
  }

  /** The hood's commanded travel, as a launch angle above horizontal. */
  private static double hoodAngleDegrees() {
    double fraction = Math.min(1.0,
        Math.max(0, RobotContainer.hood.getGoalPosition() / HOOD_ROTATIONS_AT_MAX));
    return HOOD_MAX_DEGREES - fraction * (HOOD_MAX_DEGREES - HOOD_MIN_DEGREES);
  }

  /**
   * Where the ball would land on flat ground, ignoring drag. Logged so the
   * shot's intent can be compared against where it actually ends up.
   */
  private static double predictedRange(double speed, double pitchRadians, double height) {
    double vz = speed * Math.sin(pitchRadians);
    double vh = speed * Math.cos(pitchRadians);
    double flight = (vz + Math.sqrt(vz * vz + 2 * 9.81 * height)) / 9.81;
    return vh * flight;
  }
}

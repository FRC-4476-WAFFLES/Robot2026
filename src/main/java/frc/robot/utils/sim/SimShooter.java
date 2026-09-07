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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.TurretConstants;
import frc.robot.subsystems.drive.GyroIOSim;
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

  /** Whether the prune is still able to reach into FuelSim. */
  public static boolean isPruneWorking() {
    return !reflectionUnavailable;
  }

  /** How many balls are currently being simulated, or -1 if it cannot be read. */
  public static int getBallsInPlay() {
    return ballsInPlay;
  }

  /** How many balls have been launched since the robot booted. */
  public static int getShotsFired() {
    return shotsFired;
  }

  /*
   * Reaching into FuelSim to drop balls that have come to rest.
   *
   * FuelSim is a vendor file and has to stay diffable against upstream, so it
   * cannot grow a prune method of its own — and its fuel list and the Fuel class
   * are both private. Reflection is the price of leaving it untouched. The
   * handles are cached and the sweep runs a few times a second rather than every
   * loop, because the cost that matters is the physics on every ball, and a ball
   * that has stopped keeps paying it forever.
   */
  private static java.lang.reflect.Field fuelsField;
  private static java.lang.reflect.Field positionField;
  private static java.lang.reflect.Field velocityField;
  private static boolean reflectionUnavailable = false;
  private static double lastPrune = 0;
  private static int ballsInPlay = -1;
  /** How slowly a ball must be moving, in m/s, to count as finished. */
  private static final double AT_REST_SPEED = 0.25;
  /** How low it must be, in metres, so a ball resting in the hub is not swept. */
  private static final double AT_REST_HEIGHT = 0.2;
  private static final double PRUNE_INTERVAL = 0.5;

  /**
   * Removes balls that have stopped moving.
   *
   * <p>
   * Every ball in the list is integrated every loop whether it is doing anything
   * or not, so a match's worth of dead balls on the floor is pure loop time —
   * and loop time in this simulation is what makes the flywheel and battery
   * models worth anything.
   */
  private static void pruneStoppedBalls() {
    if (reflectionUnavailable) {
      return;
    }
    double now = Timer.getTimestamp();
    if (now - lastPrune < PRUNE_INTERVAL) {
      return;
    }
    lastPrune = now;

    try {
      if (fuelsField == null) {
        fuelsField = FuelSim.class.getDeclaredField("fuels");
        fuelsField.setAccessible(true);
      }
      Object list = fuelsField.get(FuelSim.getInstance());
      if (!(list instanceof java.util.List<?> fuels)) {
        reflectionUnavailable = true;
        return;
      }
      int before = fuels.size();
      fuels.removeIf(SimShooter::hasStopped);
      ballsInPlay = fuels.size();
      Logger.recordOutput("SimShooter/Balls In Play", ballsInPlay);
      Logger.recordOutput("SimShooter/Balls Despawned", before - fuels.size());
    } catch (ReflectiveOperationException | RuntimeException e) {
      // FuelSim's internals moved. Losing the prune costs loop time, not
      // correctness, so say so once and carry on.
      DriverStation.reportWarning(
          "SimShooter cannot prune stopped balls; FuelSim's internals changed", false);
      reflectionUnavailable = true;
    }
  }

  /** Whether a ball has come to rest on the floor. */
  private static boolean hasStopped(Object fuel) {
    try {
      if (positionField == null) {
        positionField = fuel.getClass().getDeclaredField("pos");
        positionField.setAccessible(true);
        velocityField = fuel.getClass().getDeclaredField("vel");
        velocityField.setAccessible(true);
      }
      var position = (Translation3d) positionField.get(fuel);
      var velocity = (Translation3d) velocityField.get(fuel);
      return velocity.getNorm() < AT_REST_SPEED && position.getZ() < AT_REST_HEIGHT;
    } catch (ReflectiveOperationException | RuntimeException e) {
      return false;
    }
  }

  /** Fires a ball if the robot is shooting and one is due. Call every sim loop. */
  public static void update() {
    pruneStoppedBalls();
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
    // The turret's zero faces diagonally back into the robot, so its mechanism
    // position is PHYSICAL_ZERO short of where it is actually pointing. This is
    // the same sum MechanismPoses uses to draw the turret, so a ball now leaves
    // along the barrel that is rendered rather than 45 degrees off it.
    var turretHeading = robot.getRotation()
        .plus(Rotation2d.fromRotations(RobotContainer.turret.getMechanismRelativePosition()))
        .plus(TurretConstants.PHYSICAL_ZERO);

    double hoodDegrees = hoodAngleDegrees();
    double speed = exitSpeed();

    // Where the ball leaves: the turret's centre, at shooter height.
    Translation3d origin = new Translation3d(robot.getX(), robot.getY(), 0)
        .plus(new Translation3d(
            PhysicalConstants.ROBOT_TO_TURRET_CENTER.getX(),
            PhysicalConstants.ROBOT_TO_TURRET_CENTER.getY(),
            PhysicalConstants.ROBOT_TO_TURRET_CENTER.getZ())
            .rotateBy(new Rotation3d(0, 0, robot.getRotation().getRadians())));

    // A robot on the bump launches along its own tilted axis, which is exactly
    // the case the drive team reports missing from.
    double pitch = Units.degreesToRadians(hoodDegrees + GyroIOSim.getTilt());
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
    Logger.recordOutput("SimShooter/Turret Heading", turretHeading.getDegrees());
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

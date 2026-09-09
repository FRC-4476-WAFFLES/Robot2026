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
  /*
   * These three were guessed, and the shots went nowhere near the goal. They are
   * now fitted against the team's own shot map, which is the one real
   * calibration available: for each distance the map already says what flywheel
   * speed and hood position put a ball in the goal, so the numbers that make the
   * simulated flight arrive at the hub's 1.83 m entry height at that distance
   * are the numbers the real shooter must have.
   *
   * Fitted across the map's eleven points, the residual is 0.16 m of height at
   * the goal. That is close enough to watch a burst group; it is still not a
   * substitute for measuring the real thing, and the fit is only as good as the
   * map it came from.
   */
  /** Fraction of the wheel's surface speed that reaches the ball. */
  private static final double SLIP_FACTOR = 0.445;
  /** Flywheel wheel radius, in metres. */
  private static final double WHEEL_RADIUS = 0.051;
  /** Hood travel, in rotations, mapped onto its angular range. */
  private static final double HOOD_ROTATIONS_AT_MAX = 21.0;
  private static final double HOOD_MIN_DEGREES = 58.0;
  private static final double HOOD_MAX_DEGREES = 70.0;
  /*
   * How balls actually come out.
   *
   * The first version of this was fitted to the wrong thing. It used the gaps
   * between the "Fire shot" command starting, which is how often the shooter is
   * let go, not how often a ball leaves — a held trigger is one command and
   * twenty balls. So the simulation fired roughly one ball for every burst the
   * real robot fired.
   *
   * These come from counting the balls themselves, as peaks in the flywheel's
   * stator current: a ball takes fixed momentum out of the wheel and the
   * controller has to put it back, which the speed dip does not measure honestly
   * once flywheel mass is added. Across 3230 gaps in the Houston logs, which is
   * the code this repository actually runs:
   *
   * p25 0.118 s p50 0.190 s p75 0.284 s p95 0.496 s -> 3.5 balls/s
   *
   * Right-skewed, not symmetric — bursts have a floor and a long tail, because
   * the feeder catches and the wheel has to come back. A lognormal reproduces it
   * closely (0.127 / 0.190 / 0.283 / 0.501), where the old clamped Gaussian
   * could not: it needed a spread as wide as its own median and still had to be
   * clipped off at the bottom.
   *
   * <p>
   * <b>0.190 is the median, not the mean.</b> A lognormal's mean is
   * {@code median * exp(sigma^2/2)}, and the floor below trims the short tail,
   * which puts the simulated mean interval at 0.227 s — 4.40 balls/s, not the
   * 5.3 the median alone suggests.
   *
   * <p>
   * That rate is now measured rather than inferred. Five volleys across
   * Niagara, Ontarios and Houston were counted by hand off match video; the
   * four with a full hopper gave 81 balls in 18.5 s, a mean interval of
   * 0.228 s, or 4.38 balls/s. The simulation is 0.5% off that, so leave it
   * alone. The fifth volley ran at 2.6 balls/s with a near-empty hopper, which
   * is ball supply rather than the shooter and is not modelled here.
   */
  private static final double SHOT_INTERVAL = 0.190;
  /**
   * Shape of the lognormal, fitted to the quartile ratio of those same gaps.
   *
   * <p>
   * Treat this one as the soft number. The gaps it was fitted to come from the
   * peak detector, and hand counting five volleys against match video later
   * showed that detector invents extra peaks in fast bursts — it overcounts by
   * about 22% above 5 balls/s and undercounts by about 12% below 4. That
   * inflates the spread of short gaps, so real bursts are probably steadier
   * than this reproduces. The rate itself is unaffected: over-splitting a burst
   * moves gaps around without changing how many balls fell in the window.
   */
  private static final double SHOT_INTERVAL_SIGMA = 0.59;
  /** The 5th percentile: two balls are never closer than this. */
  private static final double SHOT_INTERVAL_MINIMUM = 0.067;

  /**
   * Shot-to-shot scatter, as a fraction of exit speed and degrees of heading.
   *
   * <p>
   * Nothing leaves a shooter twice the same way: the ball is compressed
   * differently, its seams sit differently, it enters the wheels at a slightly
   * different angle. None of that is worth modelling in detail, but leaving it
   * out entirely makes every simulated burst land in one spot, which is the one
   * thing a burst never does.
   *
   * <p>
   * <b>These two are guesses.</b> Everything else in this file is fitted to the
   * shot map; the scatter is not, because the logs record where a ball was
   * launched and never where it landed. Treat the spread as illustrative until
   * somebody measures a grouping.
   */
  private static final double SPEED_SCATTER = 0.02;
  private static final double HEADING_SCATTER_DEGREES = 0.8;

  /** Seeded, so a run can be repeated exactly rather than differing every time. */
  private static final java.util.Random SCATTER = new java.util.Random(4476);
  private static double nextInterval = SHOT_INTERVAL;
  /**
   * Points on the drawn trajectory. Kept short because the whole array is
   * republished on every shot, and a dashboard redrawing hundreds of poses
   * several times a second is what makes it feel slow.
   */
  private static final int TRAJECTORY_STEPS = 40;

  private static double lastShot = -1;
  private static int shotsFired = 0;
  private static boolean enabled = true;
  private static boolean startingFuel = false;

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

  /**
   * Whether to fill the field with the 360 balls a match starts with.
   *
   * <p>
   * Off by default, and the reason is AdvantageScope rather than the robot:
   * every ball is published in one array at 50 Hz and drawn as its own object in
   * the 3D view, so a field's worth makes the dashboard crawl. Watching your own
   * shots needs none of them.
   */
  public static boolean wantsStartingFuel() {
    return startingFuel;
  }

  /** Fills the field with balls, for looking at rather than driving through. */
  public static void setStartingFuel(boolean value) {
    startingFuel = value;
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
    if (lastShot > 0 && now - lastShot < nextInterval) {
      return;
    }
    lastShot = now;
    nextInterval = Math.max(SHOT_INTERVAL_MINIMUM,
        SHOT_INTERVAL * Math.exp(SCATTER.nextGaussian() * SHOT_INTERVAL_SIGMA));
    fire();
  }

  /** Launches one ball from wherever the turret is currently pointing. */
  private static void fire() {
    var robot = RobotContainer.state.getPose();
    // The turret's zero faces diagonally back into the robot, so its mechanism
    // position is PHYSICAL_ZERO short of where it is actually pointing. This is
    // the same sum MechanismPoses uses to draw the turret, so a ball now leaves
    // along the barrel that is rendered rather than 45 degrees off it.
    Rotation2d turretHeading = robot.getRotation()
        .plus(Rotation2d.fromRotations(RobotContainer.turret.getMechanismRelativePosition()))
        .plus(TurretConstants.PHYSICAL_ZERO);

    double hoodDegrees = hoodAngleDegrees();
    double speed = exitSpeed() * (1 + SCATTER.nextGaussian() * SPEED_SCATTER);
    turretHeading = turretHeading.plus(
        Rotation2d.fromDegrees(SCATTER.nextGaussian() * HEADING_SCATTER_DEGREES));

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
    //
    // It is the turret's velocity and not the robot centre's. The turret sits
    // 0.24 m off centre, so when the robot spins it travels faster than its
    // middle does -- a quarter of a metre per second at one radian per second.
    // The shot planner already leads by the turret's velocity, so giving the
    // simulated ball the centre's instead disagreed with the code under test by
    // exactly that much, and only while rotating. That is the case the drive
    // team reports going wrong, so it is the one case the simulation had to get
    // right before it could say anything useful about it.
    var robotVelocity = RobotContainer.state.getFieldVelocity();
    double robotAngle = robot.getRotation().getRadians();
    double offsetX = PhysicalConstants.ROBOT_TO_TURRET_CENTER.getX();
    double offsetY = PhysicalConstants.ROBOT_TO_TURRET_CENTER.getY();
    double omega = robotVelocity.omegaRadiansPerSecond;
    velocity = velocity.plus(new Translation3d(
        robotVelocity.vxMetersPerSecond
            - omega * (offsetX * Math.sin(robotAngle) + offsetY * Math.cos(robotAngle)),
        robotVelocity.vyMetersPerSecond
            + omega * (offsetX * Math.cos(robotAngle) - offsetY * Math.sin(robotAngle)),
        0));

    FuelSim.getInstance().spawnFuel(origin, velocity);
    shotsFired++;

    Logger.recordOutput("SimShooter/Last Shot", new Pose3d(origin,
        new Rotation3d(0, -pitch, turretHeading.getRadians())));
    Logger.recordOutput("SimShooter/Exit Speed", speed);
    Logger.recordOutput("SimShooter/Hood Degrees", hoodDegrees);
    Logger.recordOutput("SimShooter/Turret Heading", turretHeading.getDegrees());
    Logger.recordOutput("SimShooter/Predicted Range", predictedRange(speed, pitch, origin.getZ()));
    Logger.recordOutput("SimShooter/Trajectory", trajectory(origin, velocity));
  }

  /**
   * The path the ball will fly, as poses AdvantageScope can draw.
   *
   * <p>
   * Integrated the same way {@code FuelSim} integrates the ball itself, so the
   * drawn line is where the ball actually goes rather than an idealisation of it
   * — the two would part company otherwise, and a trajectory that disagrees with
   * the ball beside it is worse than none.
   */
  private static Pose3d[] trajectory(Translation3d origin, Translation3d velocity) {
    var points = new java.util.ArrayList<Pose3d>();
    Translation3d position = origin;
    Translation3d speed = velocity;
    double dt = 0.02;
    for (int i = 0; i < TRAJECTORY_STEPS && position.getZ() > 0; i++) {
      points.add(new Pose3d(position, Rotation3d.kZero));
      position = position.plus(speed.times(dt));
      speed = speed.plus(new Translation3d(0, 0, -9.81 * dt));
    }
    return points.toArray(Pose3d[]::new);
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

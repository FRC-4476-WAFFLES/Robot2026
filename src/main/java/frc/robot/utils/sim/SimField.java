// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.utils.lib.WafflesUtilities;

/**
 * Gives the simulated robot a field to be on, rather than an infinite plane.
 *
 * <p>
 * Two things it does, both for the same reason: the simulation was letting the
 * robot do things the real one cannot, which quietly makes any test run in it
 * less meaningful.
 *
 * <ul>
 * <li><b>Walls.</b> The simulated robot could be driven off the field entirely,
 * at which point vision has no tags to see and every distance to a target is
 * nonsense.
 * <li><b>The bump.</b> The real robot tilts crossing it, and a great deal of
 * behaviour keys off that tilt — the shooter switching to {@code TARGET_TAG},
 * the turret's tilt offset, and arming the post-crossing vision recovery. None
 * of it could happen in simulation, because the simulated gyro was always level.
 * </ul>
 *
 * <p>
 * This is not a physics engine. The robot is pushed back inside the wall rather
 * than colliding with it, and the tilt is a function of position rather than
 * something the wheels climb. Both are enough to exercise the code that reads
 * them, which is the point.
 */
public final class SimField {
  /**
   * Half the robot's diagonal, from its real dimensions, so a corner cannot
   * clip a wall when the robot is turned.
   */
  private static final double ROBOT_RADIUS = Math.hypot(
      PhysicalConstants.FULL_WIDTH.in(Meters), PhysicalConstants.FULL_LENGTH.in(Meters)) / 2;
  /** Where the bump sits, matching what StateOrchestrator uses. */
  private static final double BUMP_START_X = 4.0;
  /** How far the robot tilts at the peak of the bump, in degrees. */
  private static final double BUMP_TILT_DEGREES = 12.0;
  /**
   * How much grip the wheels lose on the bump.
   *
   * <p>
   * Chosen to reproduce what was measured across 263 real crossings: a clean one
   * under half a second costs about 3 cm of pose error and a slow two-to-four
   * second one costs over a metre. Slipping about a third of the wheels' motion
   * while on it lands in that range, and crossing slowly costs more simply
   * because the robot spends longer there.
   */
  private static final double BUMP_SLIP = 0.35;
  /** Wheels against a wall turn without taking the robot anywhere. */
  private static final double WALL_SLIP = 0.95;

  /** Metres per second squared. */
  private static final double GRAVITY = 9.81;
  /**
   * How much of the slope-induced speed is lost each loop to the drivetrain
   * holding the robot. Without it a robot parked on the bump accelerates away
   * forever; with it, it creeps and settles.
   */
  private static final double SLOPE_FRICTION = 0.30;
  /** How much of the crossing is spent going from level to fully tilted. */
  private static final double EDGE_TRANSITION = 0.12;
  /**
   * How much of the crossing the flat top takes, either side of the middle.
   * Without it the steepest slope sits exactly where downhill flips direction,
   * and a robot parked there is shoved back and forth instead of sliding off.
   */
  private static final double CREST_HALF_WIDTH = 0.10;

  private static boolean enabled = true;
  private static double slopeVelocity = 0;

  private SimField() {}

  /**
   * Turns the walls, the bump and the slip off.
   *
   * <p>
   * {@code SimHarness} does this at boot, because a test that places the robot
   * somewhere or tilts it by hand needs those to stay put — the walls would push
   * the pose back and the bump would overwrite the tilt on the very next loop.
   * A test that wants the field back turns it on for itself.
   */
  public static void setEnabled(boolean value) {
    enabled = value;
    if (!value) {
      RobotContainer.simState.setSlip(0);
    }
  }

  /** How fast gravity is currently pulling the robot along the bump, in m/s. */
  public static double getSlopeVelocity() {
    return slopeVelocity;
  }

  /** Whether the walls, the bump and the slip are in effect. */
  public static boolean isEnabled() {
    return enabled;
  }

  /**
   * Keeps the robot on the field, tilts it on the bump, and makes the wheels
   * slip where they would. Call once per simulation loop.
   */
  public static void update() {
    if (!enabled) {
      return;
    }
    // The truth pose, not odometry. Once the wheels start slipping the two are
    // different, and the walls and the bump are features of where the robot
    // actually is.
    Pose2d truth = RobotContainer.simState.getPose();

    double clampedX = MathUtil.clamp(truth.getX(), ROBOT_RADIUS,
        FieldConstants.fieldLength - ROBOT_RADIUS);
    double clampedY = MathUtil.clamp(truth.getY(), ROBOT_RADIUS,
        FieldConstants.fieldWidth - ROBOT_RADIUS);
    boolean againstWall = clampedX != truth.getX() || clampedY != truth.getY();

    // Field elements the robot cannot drive through. Pushed out along whichever
    // axis it is least far into, which is the direction it would actually slide.
    Translation2d pushed = pushOutOfObstacles(new Translation2d(clampedX, clampedY));
    boolean hitObstacle = pushed.getDistance(new Translation2d(clampedX, clampedY)) > 1e-6;
    clampedX = pushed.getX();
    clampedY = pushed.getY();

    if (againstWall || hitObstacle) {
      Pose2d held = new Pose2d(clampedX, clampedY, truth.getRotation());
      RobotContainer.simState.setTruePose(held);

      // Odometry is held here too, and only here. A real robot pressed into a
      // wall does let its pose run away, and simulating that faithfully was the
      // first attempt — but odometry is what the dashboard draws, so the robot
      // slid through every wall on screen while only an invisible truth pose
      // stopped. Being able to see the robot hit things is worth more than
      // reproducing that particular drift, and the bump below still produces
      // plenty of it.
      RobotContainer.drive.setPose(held);
    }

    // The bump runs across the field at a fixed X. Tilt is a triangle: up the
    // near face, over the crest, down the far face.
    Pose2d blueRelative = WafflesUtilities.FlipIfRedAlliance(truth);
    double width = FieldConstants.LinesVertical.neutralZoneNear - BUMP_START_X;
    double through = (blueRelative.getX() - BUMP_START_X) / width;
    boolean onBump = through > 0 && through < 1;
    double tilt = onBump ? tiltAt(through) : 0;

    GyroIOSim.setTilt(tilt);
    RobotContainer.simState.setSlip(
        againstWall || hitObstacle ? WALL_SLIP : onBump ? BUMP_SLIP : 0);

    applySlope(onBump, through, tilt);

    Logger.recordOutput("SimField/Against Wall", againstWall);
    Logger.recordOutput("SimField/Against Obstacle", hitObstacle);
    Logger.recordOutput("SimField/On Bump", onBump);
    Logger.recordOutput("SimField/Bump Tilt", tilt);
    Logger.recordOutput("SimField/Odometry Error",
        truth.getTranslation().getDistance(RobotContainer.state.getPose().getTranslation()));
  }

  /**
   * How far the robot is tilted, given how far through the crossing it is.
   *
   * <p>
   * The first version made tilt peak at the crest, which is backwards: the bump
   * is a ramp up, a flat top and a ramp down, so the robot is most tilted on the
   * faces and level on top. Peaking at the crest put the steepest slope exactly
   * where downhill changes direction, so a robot sitting there was shoved back
   * and forth and went nowhere.
   *
   * <p>
   * Modelled as a triangle in height, which makes the slope constant along each
   * face, with short transitions at the edges so the robot is not slapped from
   * level to fully tilted in one loop.
   */
  private static double tiltAt(double through) {
    double intoFace = Math.min(through, 1 - through) / EDGE_TRANSITION;
    double overCrest = Math.abs(through - 0.5) / CREST_HALF_WIDTH;
    return BUMP_TILT_DEGREES
        * MathUtil.clamp(intoFace, 0, 1)
        * MathUtil.clamp(overCrest, 0, 1);
  }

  /**
   * Lets gravity act on the robot the way it acts on a ball.
   *
   * <p>
   * The bump used to tilt the robot and take grip away, and do nothing else: the
   * robot climbed it at exactly the speed it drove anywhere. A ball on the same
   * slope rolls back down, and it looked wrong because it was.
   *
   * <p>
   * On a slope of angle θ, gravity pulls along the surface at g sin θ. Climbing,
   * that fights the drivetrain; descending, it helps. Drive at the bump too
   * gently and the slope wins and pushes the robot back down — which is the
   * failure the drive team calls beaching, and it could not happen here before.
   *
   * <p>
   * The push goes to the truth pose only. The wheels did not turn for it, so
   * odometry believes the robot is still climbing while it slides backwards,
   * exactly as it does on the field.
   */
  private static void applySlope(boolean onBump, double through, double tiltDegrees) {
    if (!onBump) {
      slopeVelocity = 0;
      Logger.recordOutput("SimField/Slope Velocity", 0.0);
      return;
    }

    // Downhill is towards whichever face the robot is on: back the way it came
    // on the near side, onward on the far side.
    double downhill = through < 0.5 ? -1 : 1;
    double along = -GRAVITY * Math.sin(Math.toRadians(tiltDegrees)) * downhill;

    // Friction keeps a robot sitting still on a slope from accelerating forever.
    slopeVelocity = (slopeVelocity + along * CodeConstants.PERIODIC_LOOP_TIME)
        * (1 - SLOPE_FRICTION);

    // Blue-relative X, so flip it when the field is mirrored.
    double fieldX = slopeVelocity
        * (WafflesUtilities.FlipIfRedAlliance(Pose2d.kZero).getX() > 1 ? -1 : 1);
    RobotContainer.simState.push(
        new Translation2d(fieldX * CodeConstants.PERIODIC_LOOP_TIME, 0));
    Logger.recordOutput("SimField/Slope Velocity", slopeVelocity);
  }

  /**
   * Pushes the robot out of any field element it is inside.
   *
   * <p>
   * Both hubs, treated as rectangles grown by the robot's radius. A robot only
   * ever overlaps one by a little, so it is pushed out along whichever axis it
   * is least far into — the shallowest escape, which is the way it would
   * actually slide off.
   */
  private static Translation2d pushOutOfObstacles(Translation2d position) {
    Translation2d result = position;
    for (Translation3d hub : new Translation3d[] {
        FieldConstants.Hub.topCenterPoint, FieldConstants.Hub.oppTopCenterPoint }) {
      result = pushOutOfRectangle(result, hub.getX(), hub.getY(),
          FieldConstants.Hub.width / 2 + ROBOT_RADIUS,
          FieldConstants.Hub.width / 2 + ROBOT_RADIUS);
    }
    return result;
  }

  private static Translation2d pushOutOfRectangle(Translation2d position,
      double centreX, double centreY, double halfWidth, double halfHeight) {
    double dx = position.getX() - centreX;
    double dy = position.getY() - centreY;
    if (Math.abs(dx) >= halfWidth || Math.abs(dy) >= halfHeight) {
      return position;
    }
    double escapeX = halfWidth - Math.abs(dx);
    double escapeY = halfHeight - Math.abs(dy);
    if (escapeX < escapeY) {
      return new Translation2d(centreX + Math.signum(dx) * halfWidth, position.getY());
    }
    return new Translation2d(position.getX(), centreY + Math.signum(dy) * halfHeight);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.utils.lib.WafflesUtilities;
import frc.robot.utils.sim.SimRobot;

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

  /** Metres per second squared. */
  private static final double GRAVITY = 9.81;
  /** How much of the crossing is spent going from level to fully tilted. */
  private static final double EDGE_TRANSITION = 0.12;
  /**
   * How much of the crossing the flat top takes, either side of the middle.
   * Without it the steepest slope sits exactly where downhill flips direction,
   * and a robot parked there is shoved back and forth instead of sliding off.
   */
  private static final double CREST_HALF_WIDTH = 0.10;
  /**
   * How high the crest sits above the carpet, in metres. Consistent with the
   * tilt: a face of about 0.86 m at 12 degrees rises roughly this far.
   */
  private static final double BUMP_HEIGHT = 0.18;
  /**
   * How much grip is left on the bump. Less weight on each wheel and a worse
   * surface under them, and what remains has to beat gravity along the slope --
   * so a robot that drives at it too gently slides back down. That is beaching,
   * and here it falls out of the physics rather than being written in.
   */
  private static final double BUMP_TRACTION = 0.22;

  private static boolean enabled = true;
  private static Pose3d robotPose3d = Pose3d.kZero;

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
  }

  /** Where the robot is in three dimensions, including its height and tilt. */
  public static Pose3d getRobotPose3d() {
    return robotPose3d;
  }

  /** Whether the walls, the bump and the slip are in effect. */
  public static boolean isEnabled() {
    return enabled;
  }

  /**
   * Runs the physical robot forward one loop and keeps it on the field.
   *
   * <p>
   * The order matters. The wheels state an intent, {@link SimRobot} works out
   * what the robot actually does about it, and only then is the result checked
   * against the field — so a robot that hits a wall stops because it hit a wall,
   * rather than being teleported out of one afterwards.
   */
  public static void update() {
    if (!enabled) {
      return;
    }

    // Where the robot is on the field decides the slope it is on, so that is
    // worked out before the step rather than after it.
    Pose2d before = SimRobot.getPose();
    Pose2d blueRelative = WafflesUtilities.FlipIfRedAlliance(before);
    double width = FieldConstants.LinesVertical.neutralZoneNear - BUMP_START_X;
    double through = (blueRelative.getX() - BUMP_START_X) / width;
    boolean onBump = through > 0 && through < 1;
    double tilt = onBump ? tiltAt(through) : 0;

    GyroIOSim.setTilt(tilt);

    // Gravity along the surface. Downhill is back the way it came on the near
    // face and onward on the far side, and flips with the field.
    double alongSlope = 0;
    if (onBump) {
      double downhill = through < 0.5 ? -1 : 1;
      alongSlope = -GRAVITY * Math.sin(Math.toRadians(tilt)) * downhill;
      if (blueRelative.getX() != before.getX()) {
        alongSlope = -alongSlope;
      }
    }

    SimRobot.update(RobotContainer.drive.getSimWheelIntent(),
        new Translation2d(alongSlope, 0),
        onBump ? BUMP_TRACTION : 1.0);

    // Now stop it going anywhere it cannot.
    resolveCollisions();

    // Odometry integrates the wheels regardless, so the difference between what
    // the wheels did and what the robot did is the drift — no slip factor
    // needed, because the slip is whatever physics left over.
    Pose2d truth = SimRobot.getPose();
    RobotContainer.simState.setTruePose(truth);

    publishRobotPose3d(truth, blueRelative, through, onBump);
    Logger.recordOutput("SimField/On Bump", onBump);
    Logger.recordOutput("SimField/Bump Tilt", tilt);
    Logger.recordOutput("SimField/Odometry Error",
        truth.getTranslation().getDistance(RobotContainer.state.getPose().getTranslation()));
  }

  /**
   * Stops the robot at anything solid.
   *
   * <p>
   * Each surface is checked in turn and the robot is pushed out along the
   * shallowest escape, which is the direction it would actually slide. The
   * velocity into that surface is taken away by {@link SimRobot#collide}, so a
   * robot driving along a wall keeps moving down it rather than sticking.
   */
  private static void resolveCollisions() {
    Pose2d current = SimRobot.getPose();
    double x = current.getX();
    double y = current.getY();

    double clampedX = MathUtil.clamp(x, ROBOT_RADIUS, FieldConstants.fieldLength - ROBOT_RADIUS);
    double clampedY = MathUtil.clamp(y, ROBOT_RADIUS, FieldConstants.fieldWidth - ROBOT_RADIUS);
    if (clampedX != x || clampedY != y) {
      Translation2d normal = new Translation2d(
          Math.signum(clampedX - x), Math.signum(clampedY - y));
      if (normal.getNorm() > 0) {
        normal = normal.div(normal.getNorm());
      }
      SimRobot.collide(new Pose2d(clampedX, clampedY, current.getRotation()), normal);
      Logger.recordOutput("SimField/Against Wall", true);
      current = SimRobot.getPose();
    } else {
      Logger.recordOutput("SimField/Against Wall", false);
    }

    boolean hitObstacle = false;
    for (Translation3d hub : new Translation3d[] {
        FieldConstants.Hub.topCenterPoint, FieldConstants.Hub.oppTopCenterPoint }) {
      double half = FieldConstants.Hub.width / 2 + ROBOT_RADIUS;
      Translation2d pushed = pushOutOfRectangle(current.getTranslation(),
          hub.getX(), hub.getY(), half, half);
      if (pushed.getDistance(current.getTranslation()) > 1e-9) {
        Translation2d normal = pushed.minus(current.getTranslation());
        normal = normal.div(normal.getNorm());
        SimRobot.collide(new Pose2d(pushed, current.getRotation()), normal);
        current = SimRobot.getPose();
        hitObstacle = true;
      }
    }
    Logger.recordOutput("SimField/Against Obstacle", hitObstacle);
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
   * Publishes where the robot is in three dimensions, so it can be drawn sitting
   * on the field rather than through it.
   *
   * <p>
   * The robot's pose is a {@code Pose2d} — X, Y and a heading. There is nowhere
   * in it to say the robot is off the floor and nose up, so however much the
   * gyro reported, the drawn robot stayed flat on the carpet and slid through
   * the bump. A real robot cannot know its own height either, which is why this
   * belongs to the simulation and not to {@code RobotState}.
   *
   * <p>
   * Point AdvantageScope's 3D field at {@code SimField/Robot Pose 3D} and the
   * robot climbs the bump, nose up going on and tail up coming off. Pitch and
   * roll are resolved against the heading rather than assumed, so crossing the
   * bump square pitches the robot and crossing it sideways rolls it.
   */
  private static void publishRobotPose3d(Pose2d truth, Pose2d blueRelative,
      double through, boolean onBump) {
    double height = onBump ? BUMP_HEIGHT * (1 - Math.abs(through * 2 - 1)) : 0;

    double slopeRadians = onBump
        ? Math.toRadians(tiltAt(through)) * (through < 0.5 ? 1 : -1)
        : 0;
    if (blueRelative.getX() != truth.getX()) {
      slopeRadians = -slopeRadians;
    }

    double heading = truth.getRotation().getRadians();
    robotPose3d = new Pose3d(
        new Translation3d(truth.getX(), truth.getY(), height),
        new Rotation3d(-slopeRadians * Math.sin(heading), -slopeRadians * Math.cos(heading),
            heading));
    Logger.recordOutput("SimField/Robot Pose 3D", robotPose3d);
    Logger.recordOutput("SimField/Height", height);
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

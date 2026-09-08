// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.FieldConstants;
import frc.robot.data.FieldConstants.Hub;
import frc.robot.data.FieldConstants.LeftBump;
import frc.robot.data.FieldConstants.LeftTrench;
import frc.robot.data.FieldConstants.LinesHorizontal;
import frc.robot.data.FieldConstants.LinesVertical;
import frc.robot.data.FieldConstants.Tower;
import frc.robot.data.Constants.ExpanderConstants.ExpanderPosition;
import frc.robot.subsystems.drive.GyroIOSim;

/**
 * Gives the simulated robot a field to be on, rather than an infinite plane.
 *
 * <p>
 * Everything here is real field geometry out of {@link FieldConstants}, in
 * absolute blue-origin coordinates. Nothing is alliance-relative, because the
 * field is not: both hubs, both pairs of bumps, both towers and all four
 * trenches exist at once, and the robot can drive into any of them.
 *
 * <p>
 * Two things it does, both for the same reason: the simulation was letting the
 * robot do things the real one cannot, which quietly makes any test run in it
 * less meaningful.
 *
 * <ul>
 * <li><b>Solid things.</b> The simulated robot could be driven off the field,
 * or straight through a hub, and vision then has no tags to see and every
 * distance to a target is nonsense.
 * <li><b>The bumps.</b> The real robot tilts crossing one, and a great deal of
 * behaviour keys off that tilt — the shooter switching to {@code TARGET_TAG},
 * the turret's tilt offset, and arming the post-crossing vision recovery. None
 * of it could happen in simulation, because the simulated gyro was always
 * level.
 * </ul>
 *
 * <p>
 * This is not a physics engine. The robot is an oriented rectangle, obstacles
 * are axis-aligned boxes, and a bump is a height and a slope rather than
 * something the wheels climb. What
 * makes it worth having is that {@link SimRobot} carries momentum through all
 * of it, so a robot that fails to get over a bump fails for the same reason the
 * real one does.
 */
public final class SimField {
  /**
   * How far the intake sticks out past the frame when fully deployed, in metres.
   *
   * <p>
   * <b>Not measured.</b> Nothing in the codebase records the intake's reach, so
   * this is a placeholder chosen to be about the size of the mechanism drawn in
   * {@code MechanismPoses}. Measure it on the real robot — it decides how much
   * bigger the robot is with the intake down, which is exactly when it is most
   * likely to catch on something.
   */
  private static final double INTAKE_EXTENSION = 0.30;

  /** Metres per second squared. */
  private static final double GRAVITY = 9.81;
  /** How much of a crossing is spent going from level to fully tilted. */
  private static final double EDGE_TRANSITION = 0.12;
  /**
   * How much of a crossing the flat top takes, either side of the middle.
   * Without it the steepest slope sits exactly where downhill flips direction,
   * and a robot parked there is shoved back and forth instead of sliding off.
   */
  private static final double CREST_HALF_WIDTH = 0.10;
  /** How high the crest sits above the carpet: the real 6.5 inch ramp. */
  private static final double BUMP_HEIGHT = LeftBump.height;
  /**
   * How far the robot tilts on a face, in degrees. Taken from the ramp's own
   * dimensions rather than guessed — 6.5 inches of rise over the part of the 44
   * inch depth that is not the flat top.
   */
  private static final double BUMP_TILT_DEGREES = Math.toDegrees(
      Math.atan2(BUMP_HEIGHT, LeftBump.depth * (0.5 - CREST_HALF_WIDTH)));
  /**
   * How much grip is left on a bump. Less weight on each wheel and a worse
   * surface under them, and what remains has to beat gravity along the slope —
   * so a robot that drives at it too gently slides back down. That is beaching,
   * and here it falls out of the physics rather than being written in.
   *
   * <p>
   * Calibration knob. At this value about 2.5 m/s² is left over once gravity
   * along the slope is paid for, which crosses briskly but bogs down on a slow
   * approach. Lower it if the real robot beaches more than the simulated one.
   */
  private static final double BUMP_TRACTION = 0.55;

  /*
   * Tower upright and trench wall sizes, from maple-sim's Arena2026Rebuilt:
   * 3.5 by 1.5 inch posts, and a 12 inch thick trench wall.
   */
  private static final double UPRIGHT_HALF_X = Units.inchesToMeters(3.5) / 2;
  private static final double UPRIGHT_HALF_Y = Units.inchesToMeters(1.5) / 2;
  private static final double TRENCH_WALL_HALF_Y = Units.inchesToMeters(12.0) / 2;

  /** Anything solid, as a box the robot's centre cannot come within reach of. */
  private record Obstacle(
      String name,
      double centreX,
      double centreY,
      double halfX,
      double halfY
  ) {}

  private static final List<Obstacle> OBSTACLES = buildObstacles();
  /** The four ramps, as {@code {minX, maxX, minY, maxY}}. */
  private static final double[][] BUMPS = buildBumps();

  private static boolean enabled = true;
  private static Pose3d robotPose3d = Pose3d.kZero;

  private SimField() {}

  /**
   * Turns the walls, the bumps and the obstacles off.
   *
   * <p>
   * {@code SimHarness} does this at boot, because a test that places the robot
   * somewhere or tilts it by hand needs those to stay put — the walls would push
   * the pose back and a bump would overwrite the tilt on the very next loop. A
   * test that wants the field back turns it on for itself.
   */
  public static void setEnabled(boolean value) {
    enabled = value;
  }

  /** Where the robot is in three dimensions, including its height and tilt. */
  public static Pose3d getRobotPose3d() {
    return robotPose3d;
  }

  /**
   * How long the robot currently is, front to back, including whatever the
   * intake is sticking out. For tests and for watching on a dashboard.
   */
  public static double getFootprintLength() {
    return footprintOf(SimRobot.getPose()).halfLength() * 2;
  }

  /** Whether the walls, the bumps and the obstacles are in effect. */
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

    // Where the robot is decides the slope it is on, so that is worked out
    // before the step rather than after it.
    double through = bumpProgress(SimRobot.getPose());
    boolean onBump = !Double.isNaN(through);
    double tilt = onBump ? tiltAt(through) : 0;

    GyroIOSim.setTilt(tilt);

    // Gravity along the surface, pointing down whichever face the robot is on.
    // The bumps run across the field, so that is always along X.
    double alongSlope = onBump
        ? GRAVITY * Math.sin(Math.toRadians(tilt)) * (through < 0.5 ? -1 : 1)
        : 0;

    // Wheels on a slope travel along the hypotenuse while the robot only moves
    // the base of it, so a metre of wheel is less than a metre of field. That is
    // most of what a clean bump crossing costs odometry: nothing slipped, the
    // wheels were simply measuring the wrong distance.
    SimRobot.update(alongSurface(RobotContainer.drive.getSimWheelIntent(), tilt),
        new Translation2d(alongSlope, 0),
        onBump ? BUMP_TRACTION : 1.0);

    // Now stop it going anywhere it cannot.
    resolveCollisions();

    // Odometry integrates the wheels regardless, so the difference between what
    // the wheels did and what the robot did is the drift — no slip factor
    // needed, because the slip is whatever physics left over.
    Pose2d truth = SimRobot.getPose();
    RobotContainer.simState.setTruePose(truth);

    publishRobotPose3d(truth, through, onBump);
    Logger.recordOutput("SimField/On Bump", onBump);
    Logger.recordOutput("SimField/Bump Tilt", tilt);
    Logger.recordOutput("SimField/Odometry Error",
        truth.getTranslation().getDistance(RobotContainer.state.getPose().getTranslation()));
  }

  /**
   * Stops the robot at anything solid.
   *
   * <p>
   * The robot is its real rectangle, not a circle. A circle of the robot's
   * half-diagonal is right for clipping a corner and much too fat for threading
   * a gap — it could not fit between the tower uprights, which the real robot
   * does — and it cannot represent a robot being longer than it is wide, so a
   * robot turned sideways in a gap behaved identically to one lined up with it.
   *
   * <p>
   * Each surface is checked in turn and the robot is pushed out along the
   * shallowest escape, which is the direction it would actually slide. The
   * velocity into that surface is taken away by {@link SimRobot#collide}, so a
   * robot driving along a wall keeps moving down it rather than sticking.
   */
  private static void resolveCollisions() {
    Pose2d current = SimRobot.getPose();
    Footprint robot = footprintOf(current);

    // Walls, using the footprint's world-aligned extent so a turned robot needs
    // more room across than a square one does.
    double halfX = robot.extentX();
    double halfY = robot.extentY();
    double clampedX = MathUtil.clamp(robot.centre().getX(), halfX,
        FieldConstants.fieldLength - halfX);
    double clampedY = MathUtil.clamp(robot.centre().getY(), halfY,
        FieldConstants.fieldWidth - halfY);
    boolean againstWall = clampedX != robot.centre().getX() || clampedY != robot.centre().getY();
    if (againstWall) {
      push(current.getTranslation()
          .plus(new Translation2d(clampedX, clampedY).minus(robot.centre())));
      current = SimRobot.getPose();
      robot = footprintOf(current);
    }
    Logger.recordOutput("SimField/Against Wall", againstWall);

    String hit = "";
    for (Obstacle obstacle : OBSTACLES) {
      Translation2d escape = separate(robot, obstacle);
      if (escape != null) {
        push(current.getTranslation().plus(escape));
        current = SimRobot.getPose();
        robot = footprintOf(current);
        hit = obstacle.name();
      }
    }
    Logger.recordOutput("SimField/Against Obstacle", hit);
    Logger.recordOutput("SimField/Footprint Length", robot.halfLength() * 2);
  }

  /**
   * The robot's rectangle, grown forward by however far the intake is out.
   *
   * <p>
   * The intake swings off the +X face, so deploying it makes the robot longer at
   * the front only: the box grows by the extension and its centre moves half
   * that way forward. A robot with the intake down really is a bigger object,
   * and it is the part most likely to catch on something.
   */
  private static Footprint footprintOf(Pose2d pose) {
    double extension = INTAKE_EXTENSION * intakeDeployedFraction();
    Translation2d forward = new Translation2d(pose.getRotation().getCos(),
        pose.getRotation().getSin());
    return new Footprint(
        pose.getTranslation().plus(forward.times(extension / 2)),
        pose.getRotation(),
        PhysicalConstants.FULL_LENGTH.in(Meters) / 2 + extension / 2,
        PhysicalConstants.FULL_WIDTH.in(Meters) / 2);
  }

  /** How far out the intake is, from 0 stowed to 1 fully deployed. */
  private static double intakeDeployedFraction() {
    if (RobotContainer.intake == null) {
      return 0;
    }
    double degrees = Units.rotationsToDegrees(RobotContainer.intake.getExpanderPosition());
    return MathUtil.clamp(degrees / ExpanderPosition.EXTENDED.getDegrees(), 0, 1);
  }

  /**
   * How far to move the robot to get it out of an obstacle, or null if it is
   * already clear.
   *
   * <p>
   * Separating axis theorem across the four axes that matter — the field's two
   * and the robot's two. If any of them separates the shapes there is no
   * collision; otherwise the smallest overlap is the shortest way out, which is
   * also the direction the robot would really slide.
   */
  private static Translation2d separate(Footprint robot, Obstacle box) {
    Translation2d unitX = new Translation2d(robot.heading().getCos(), robot.heading().getSin());
    Translation2d unitY = new Translation2d(-robot.heading().getSin(), robot.heading().getCos());
    Translation2d toBox = new Translation2d(box.centreX(), box.centreY()).minus(robot.centre());

    Translation2d bestAxis = null;
    double bestOverlap = Double.MAX_VALUE;
    for (Translation2d axis : new Translation2d[] {
        new Translation2d(1, 0), new Translation2d(0, 1), unitX, unitY }) {
      double robotExtent = robot.halfLength() * Math.abs(dot(unitX, axis))
          + robot.halfWidth() * Math.abs(dot(unitY, axis));
      double boxExtent = box.halfX() * Math.abs(axis.getX()) + box.halfY() * Math.abs(axis.getY());
      double overlap = robotExtent + boxExtent - Math.abs(dot(toBox, axis));
      if (overlap <= 0) {
        return null;
      }
      if (overlap < bestOverlap) {
        bestOverlap = overlap;
        bestAxis = axis;
      }
    }
    return bestAxis.times(dot(toBox, bestAxis) > 0 ? -bestOverlap : bestOverlap);
  }

  private static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  /** The robot as an oriented rectangle on the field. */
  private record Footprint(
      Translation2d centre,
      Rotation2d heading,
      double halfLength,
      double halfWidth
  ) {
    /** Half the width of the world-aligned box this footprint fits inside. */
    double extentX() {
      return halfLength * Math.abs(heading.getCos()) + halfWidth * Math.abs(heading.getSin());
    }

    double extentY() {
      return halfLength * Math.abs(heading.getSin()) + halfWidth * Math.abs(heading.getCos());
    }
  }

  /**
   * Moves the robot to where it is allowed to be, and takes away the speed it
   * arrived with.
   */
  private static void push(Translation2d allowed) {
    Pose2d current = SimRobot.getPose();
    Translation2d normal = allowed.minus(current.getTranslation());
    if (normal.getNorm() < 1e-9) {
      return;
    }
    SimRobot.collide(new Pose2d(allowed, current.getRotation()), normal.div(normal.getNorm()));
  }

  /** What the wheels' intent is worth in the field plane, on a surface tilted this far. */
  private static ChassisSpeeds alongSurface(ChassisSpeeds wheelIntent, double tiltDegrees) {
    double flattened = Math.cos(Math.toRadians(tiltDegrees));
    return new ChassisSpeeds(
        wheelIntent.vxMetersPerSecond * flattened,
        wheelIntent.vyMetersPerSecond * flattened,
        wheelIntent.omegaRadiansPerSecond);
  }

  /**
   * How far across a bump the robot is, from 0 at the near edge to 1 at the far
   * one, or {@code NaN} if it is not on one.
   *
   * <p>
   * The bumps are not a band across the whole field — the middle of that band is
   * the hub. They are two ramps beside each hub, so a robot going round the hub
   * crosses one and a robot lined up on the hub cannot cross at all. Modelling
   * it as a band was why the tilt happened in places the real robot is never
   * tilted, and never happened where it actually is.
   */
  private static double bumpProgress(Pose2d pose) {
    for (double[] bump : BUMPS) {
      if (pose.getX() > bump[0] && pose.getX() < bump[1]
          && pose.getY() > bump[2] && pose.getY() < bump[3]) {
        return (pose.getX() - bump[0]) / (bump[1] - bump[0]);
      }
    }
    return Double.NaN;
  }

  /**
   * How far the robot is tilted, given how far through a crossing it is.
   *
   * <p>
   * The bump is a ramp up, a flat top and a ramp down, so the robot is most
   * tilted on the faces and level on top. Modelled as a trapezoid, which makes
   * the slope constant along each face, with short transitions at the edges so
   * the robot is not slapped from level to fully tilted in one loop.
   */
  private static double tiltAt(double through) {
    double intoFace = Math.min(through, 1 - through) / EDGE_TRANSITION;
    double overCrest = Math.abs(through - 0.5) / CREST_HALF_WIDTH;
    return BUMP_TILT_DEGREES
        * MathUtil.clamp(intoFace, 0, 1)
        * MathUtil.clamp(overCrest, 0, 1);
  }

  /**
   * How far off the carpet the robot is, following the same trapezoid as the
   * tilt.
   */
  private static double heightAt(double through) {
    return BUMP_HEIGHT
        * MathUtil.clamp(Math.min(through, 1 - through) / (0.5 - CREST_HALF_WIDTH), 0, 1);
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
  private static void publishRobotPose3d(Pose2d truth, double through, boolean onBump) {
    double height = onBump ? heightAt(through) : 0;
    // The face rises in +X on the way on and falls in +X on the way off.
    double slopeRadians = onBump
        ? Math.toRadians(tiltAt(through)) * (through < 0.5 ? 1 : -1)
        : 0;

    double heading = truth.getRotation().getRadians();
    robotPose3d = new Pose3d(
        new Translation3d(truth.getX(), truth.getY(), height),
        new Rotation3d(-slopeRadians * Math.sin(heading), -slopeRadians * Math.cos(heading),
            heading));
    Logger.recordOutput("SimField/Robot Pose 3D", robotPose3d);
    Logger.recordOutput("SimField/Height", height);
  }

  /**
   * Everything solid on the field.
   *
   * <p>
   * The shapes here follow maple-sim's {@code Arena2026Rebuilt}, which is better
   * sourced than the first version of this method: that took the whole tower and
   * the whole trench as solid blocks, which is wrong in both cases and wrong in a
   * way that matters. A solid tower cannot be driven into to climb, and a solid
   * trench blocks ground a robot can actually occupy.
   *
   * <p>
   * The positions are ours rather than theirs, because ours are derived from the
   * AprilTag layout and so stay consistent with what vision believes. The two
   * agree closely, which is the reassuring part — maple-sim puts the tower
   * uprights at (1.062, 3.315) and (1.062, 4.172) where {@link Tower} computes
   * (1.105, 3.301) and (1.105, 4.159), and their trench wall begins at y = 1.279
   * where {@link LinesHorizontal#rightTrenchOpenStart} ends at 1.266.
   */
  private static List<Obstacle> buildObstacles() {
    double hubHalf = Hub.width / 2;
    List<Obstacle> obstacles = new ArrayList<>();

    obstacles.add(new Obstacle("Hub", LinesVertical.hubCenter,
        FieldConstants.fieldWidth / 2, hubHalf, hubHalf));
    obstacles.add(new Obstacle("Opp Hub", LinesVertical.oppHubCenter,
        FieldConstants.fieldWidth / 2, hubHalf, hubHalf));

    // The tower is two posts, not a wall. A robot drives between them to climb.
    double oppTowerX = FieldConstants.fieldLength - Tower.frontFaceX;
    obstacles.add(upright("Tower Upright", Tower.frontFaceX, Tower.leftUpright.getY()));
    obstacles.add(upright("Tower Upright", Tower.frontFaceX, Tower.rightUpright.getY()));
    obstacles.add(upright("Opp Tower Upright", oppTowerX, Tower.oppLeftUpright.getY()));
    obstacles.add(upright("Opp Tower Upright", oppTowerX, Tower.oppRightUpright.getY()));

    // The trench is a wall along its inner edge, not a filled block.
    double wallHalfX = LeftTrench.depth / 2;
    for (double hubX : new double[] { LinesVertical.hubCenter, LinesVertical.oppHubCenter }) {
      obstacles.add(new Obstacle("Right Trench Wall", hubX,
          LinesHorizontal.rightTrenchOpenStart + TRENCH_WALL_HALF_Y,
          wallHalfX, TRENCH_WALL_HALF_Y));
      obstacles.add(new Obstacle("Left Trench Wall", hubX,
          LinesHorizontal.leftTrenchOpenEnd - TRENCH_WALL_HALF_Y,
          wallHalfX, TRENCH_WALL_HALF_Y));
    }
    return obstacles;
  }

  /** The two ramps beside each hub, each running across the field in X. */
  private static double[][] buildBumps() {
    double halfDepth = LeftBump.depth / 2;
    return new double[][] {
        { LinesVertical.hubCenter - halfDepth, LinesVertical.hubCenter + halfDepth,
            LinesHorizontal.leftBumpEnd, LinesHorizontal.leftBumpStart },
        { LinesVertical.hubCenter - halfDepth, LinesVertical.hubCenter + halfDepth,
            LinesHorizontal.rightBumpEnd, LinesHorizontal.rightBumpStart },
        { LinesVertical.oppHubCenter - halfDepth, LinesVertical.oppHubCenter + halfDepth,
            LinesHorizontal.leftBumpEnd, LinesHorizontal.leftBumpStart },
        { LinesVertical.oppHubCenter - halfDepth, LinesVertical.oppHubCenter + halfDepth,
            LinesHorizontal.rightBumpEnd, LinesHorizontal.rightBumpStart } };
  }

  /** One tower post, at the position {@link Tower} computes for it. */
  private static Obstacle upright(String name, double x, double y) {
    return new Obstacle(name, x, y, UPRIGHT_HALF_X, UPRIGHT_HALF_Y);
  }

}

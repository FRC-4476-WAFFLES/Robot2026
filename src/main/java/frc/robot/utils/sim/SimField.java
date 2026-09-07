// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
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
  /** Roughly half the robot's diagonal, so a corner does not clip the wall. */
  private static final double ROBOT_RADIUS = 0.45;
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

  private static boolean enabled = true;

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
      // The robot stops; the wheels do not. Odometry keeps integrating the full
      // wheel motion and runs away from the truth, which is what pressing a
      // real robot into a wall does to its pose.
      RobotContainer.simState.setTruePose(
          new Pose2d(clampedX, clampedY, truth.getRotation()), truth.getRotation());
    }

    // The bump runs across the field at a fixed X. Tilt is a triangle: up the
    // near face, over the crest, down the far face.
    Pose2d blueRelative = WafflesUtilities.FlipIfRedAlliance(truth);
    double width = FieldConstants.LinesVertical.neutralZoneNear - BUMP_START_X;
    double through = (blueRelative.getX() - BUMP_START_X) / width;
    boolean onBump = through > 0 && through < 1;
    double tilt = onBump ? BUMP_TILT_DEGREES * (1 - Math.abs(through * 2 - 1)) : 0;

    GyroIOSim.setTilt(tilt);
    RobotContainer.simState.setSlip(
        againstWall || hitObstacle ? WALL_SLIP : onBump ? BUMP_SLIP : 0);

    Logger.recordOutput("SimField/Against Wall", againstWall);
    Logger.recordOutput("SimField/Against Obstacle", hitObstacle);
    Logger.recordOutput("SimField/On Bump", onBump);
    Logger.recordOutput("SimField/Bump Tilt", tilt);
    Logger.recordOutput("SimField/Odometry Error",
        truth.getTranslation().getDistance(RobotContainer.state.getPose().getTranslation()));
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

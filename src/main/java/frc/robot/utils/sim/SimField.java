// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
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
  /** Roughly half the robot's diagonal, so a corner does not clip the wall. */
  private static final double ROBOT_RADIUS = 0.45;
  /** Where the bump sits, matching what StateOrchestrator uses. */
  private static final double BUMP_START_X = 4.0;
  /** How far the robot tilts at the peak of the bump, in degrees. */
  private static final double BUMP_TILT_DEGREES = 12.0;

  private static boolean enabled = true;

  private SimField() {}

  /**
   * Turns the walls and the bump off.
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

  /** Whether the walls and the bump are in effect. */
  public static boolean isEnabled() {
    return enabled;
  }

  /**
   * Keeps the robot on the field and tilts it on the bump. Call once per
   * simulation loop.
   */
  public static void update() {
    if (!enabled) {
      return;
    }
    Pose2d pose = RobotContainer.state.getPose();

    double clampedX = MathUtil.clamp(pose.getX(), ROBOT_RADIUS,
        FieldConstants.fieldLength - ROBOT_RADIUS);
    double clampedY = MathUtil.clamp(pose.getY(), ROBOT_RADIUS,
        FieldConstants.fieldWidth - ROBOT_RADIUS);

    boolean hitWall = clampedX != pose.getX() || clampedY != pose.getY();
    if (hitWall) {
      // Push back rather than model a collision. The drivetrain keeps running,
      // so the robot sits against the wall the way it would if it were pressed
      // into one, and odometry drifts exactly as it would while the wheels slip.
      RobotContainer.drive.setPose(new Pose2d(clampedX, clampedY, pose.getRotation()));
    }
    Logger.recordOutput("SimField/Against Wall", hitWall);

    // The bump runs across the field at a fixed X. Tilt is a triangle: up the
    // near face, over the crest, down the far face.
    Pose2d blueRelative = WafflesUtilities.FlipIfRedAlliance(pose);
    double width = FieldConstants.LinesVertical.neutralZoneNear - BUMP_START_X;
    double through = (blueRelative.getX() - BUMP_START_X) / width;
    double tilt = 0;
    if (through > 0 && through < 1) {
      tilt = BUMP_TILT_DEGREES * (1 - Math.abs(through * 2 - 1));
    }
    GyroIOSim.setTilt(tilt);
    Logger.recordOutput("SimField/Bump Tilt", tilt);
    Logger.recordOutput("SimField/Loop Time", CodeConstants.PERIODIC_LOOP_TIME);
  }
}

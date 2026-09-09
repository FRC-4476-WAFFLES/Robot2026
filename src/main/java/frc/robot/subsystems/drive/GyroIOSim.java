// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import frc.robot.data.Constants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.sim.SimRobot;

/**
 * A simulated gyro that can be tilted.
 *
 * <p>
 * It reports itself disconnected, exactly as the no-op implementation it
 * replaced did, so {@code Drive} keeps deriving heading from the module states
 * and nothing about how the simulated robot turns is changed. The only thing it
 * adds is a tilt that a test can set.
 *
 * <p>
 * That matters because several behaviours key off the gyro rather than the pose,
 * and none of them could be reached in simulation before: coming off the bump
 * arms the post-crossing vision recovery, being tilted forces the shooter into
 * {@code TARGET_TAG}, and the turret's bump offset is looked up by tilt angle.
 * The gyro is deliberately the signal those use, because it is the one a
 * crossing cannot corrupt — which made it the one thing that could not be tested.
 *
 * <p>
 * The tilt is static because {@code SimHarness} builds one robot per JVM and a
 * test has no other handle on the IO layer. It resets to level so a test that
 * forgets to put it back cannot silently tilt every test after it.
 */
public class GyroIOSim implements GyroIO {
  private static volatile double tiltDegrees = 0.0;
  /** Field bearing the surface normal leans toward, which is downhill. */
  private static volatile double normalAzimuthRadians = 0.0;

  /** Tilts the simulated robot. Zero is level. */
  public static void setTilt(double degrees) {
    setTilt(degrees, 0.0);
  }

  /**
   * @param degrees how far off level
   * @param normalAzimuth the field bearing the surface normal leans toward,
   *     which is downhill. The bumps run across the field, so this is 0 or pi.
   */
  public static void setTilt(double degrees, double normalAzimuth) {
    tiltDegrees = degrees;
    normalAzimuthRadians = normalAzimuth;
  }

  /** Puts the robot back on the flat. */
  public static void reset() {
    tiltDegrees = 0.0;
    normalAzimuthRadians = 0.0;
  }

  /**
   * The surface the robot is standing on, as a unit normal in field
   * coordinates.
   *
   * <p>
   * Everything about a tilted robot follows from this one vector: which way the
   * gyro's gravity reading leans, and which way a ball leaves the shooter. They
   * have to come from the same place or the simulation disagrees with itself.
   */
  public static Translation3d getSurfaceNormal() {
    double t = Math.toRadians(tiltDegrees);
    return new Translation3d(
        Math.sin(t) * Math.cos(normalAzimuthRadians),
        Math.sin(t) * Math.sin(normalAzimuthRadians),
        Math.cos(t));
  }

  /** Rotation taking a level frame onto the surface the robot is standing on. */
  public static Rotation3d getSurfaceLean() {
    if (tiltDegrees == 0) {
      return Rotation3d.kZero;
    }
    return new Rotation3d(new Translation3d(0, 0, 1).toVector(),
        getSurfaceNormal().toVector());
  }

  /** The tilt currently being simulated, in degrees. */
  public static double getTilt() {
    return tiltDegrees;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.tipAngle = tiltDegrees;
    // The simulated bump runs across the field, so its tilt is pitch when the
    // robot is square to it and roll when it is sideways on. Split the same way
    // the real Pigeon would see it, so the field is populated rather than left
    // at zero for replay.
    // What the gyro would report: the field's "up", expressed in the robot's
    // own frame. Derived from the surface rather than written out by hand,
    // because the hand-written version had the robot's heading in it twice and
    // a sign the wrong way round.
    //
    // The robot's body sits with its Z along the surface normal and its X along
    // its heading, so the body-to-field rotation is the lean applied after the
    // yaw. Field up seen from the body is that rotation, inverted, applied to
    // vertical.
    double heading = SimRobot.getPose().getRotation().getRadians();
    Translation3d up = new Translation3d(0, 0, 1)
        .rotateBy(getSurfaceLean().unaryMinus())
        .rotateBy(new Rotation3d(0, 0, -heading));
    inputs.gravityVectorX = up.getX();
    inputs.gravityVectorY = up.getY();
    inputs.gravityVectorZ = up.getZ();
    // Pitch and roll as the Pigeon would fuse them, from the same vector.
    inputs.pitchDegrees = Math.toDegrees(Math.asin(-up.getX()));
    inputs.rollDegrees = Math.toDegrees(Math.asin(up.getY()));
    // The real gyro logs this from GyroIOPigeon2.getTiltMagnitude, so log it
    // here too or the tilt is invisible in AdvantageScope during a sim run.
    Logger.recordOutput("TiltDeg", tiltDegrees);
    inputs.levelOnGround = !GyroIOPigeon2.isOnBumpGravity(tiltDegrees);
    // Left disconnected on purpose: Drive falls back to the module states for
    // heading, which is how the simulated robot has always turned.
    inputs.connected = false;
  }

  /** The tilt at which the robot is taken to be on the bump, for tests to aim at. */
  public static double onBumpTilt() {
    return Constants.CodeConstants.ON_BUMP_TILT;
  }
}

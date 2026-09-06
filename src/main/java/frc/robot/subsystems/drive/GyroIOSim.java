// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.data.Constants;

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

  /** Tilts the simulated robot. Zero is level. */
  public static void setTilt(double degrees) {
    tiltDegrees = degrees;
  }

  /** Puts the robot back on the flat. */
  public static void reset() {
    tiltDegrees = 0.0;
  }

  /** The tilt currently being simulated, in degrees. */
  public static double getTilt() {
    return tiltDegrees;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.tipAngle = tiltDegrees;
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

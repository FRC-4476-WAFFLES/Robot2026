// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public boolean levelOnGround = true;
    public double tipAngle = 0.0;
    /*
     * The Pigeon's own fused pitch and roll, alongside the gravity-vector tilt.
     *
     * Logged because tipAngle cannot be trusted and there is currently no way to
     * check the alternative. It is acos of the gravity vector's Z component,
     * which has two problems measured across every match: acos is
     * ill-conditioned near flat, so a 0.75% error in the vector shows up as 7
     * degrees of tilt, and the gravity vector comes from an accelerometer, which
     * cannot tell gravity from braking.
     *
     * Measured stationary and level over 12708 samples, tipAngle reads a median
     * of 7.0 degrees against an ON_BUMP_TILT threshold of 9.5 -- three quarters
     * of the budget gone before the robot moves. Under hard acceleration 17% of
     * samples cross the threshold while nowhere near a ramp.
     *
     * Pitch and roll are fused with the gyro rather than taken from gravity
     * alone, so they should reject that. Nobody can say by how much until they
     * are in a log, which is what these are for.
     */
    public double pitchDegrees = 0.0;
    public double rollDegrees = 0.0;
    /*
     * The gravity vector itself, so the mounting error can be characterised.
     *
     * The 7 degrees is not noise. Across 9543 stationary, flat samples in
     * eleven matches over three days, the per-match median runs 6.78 to 7.17
     * degrees -- a range of 0.39 -- with a within-match interquartile spread of
     * 0.06 to 0.61. That is a fixed mounting error being reported faithfully:
     * cos(7 degrees) is 0.9925, which is exactly the Z component seen. There is
     * no MountPose configured anywhere in this project.
     *
     * Correcting it needs the X and Y components as well as Z, because a mount
     * error is a rotation and not a scalar. Subtracting 7 from the angle is
     * only right when the robot happens to tilt in the same plane as the error;
     * tilt it across that plane and the true total is the hypotenuse, not the
     * difference. So log all three, sit the robot still and level, and the
     * measured vector is the reference to rotate every later reading against.
     *
     * Raising ON_BUMP_TILT does not help and was checked: against
     * vision-anchored positions, 9.5 misses 23% of real bump samples and fires
     * on 7.4% of fast flat driving, and 12.0 trades that for missing 43% to
     * gain 2.7 points of false alarms. The distributions overlap because the
     * gravity vector cannot tell tilting from braking, and no threshold on a
     * signal like that separates them.
     */
    public double gravityVectorX = 0.0;
    public double gravityVectorY = 0.0;
    public double gravityVectorZ = 1.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}

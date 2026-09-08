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
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}

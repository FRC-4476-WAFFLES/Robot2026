// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.data.Constants.HoodConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SplineMonotone1D;

/**
 * Checks the shot map against what the flywheel can physically do.
 *
 * <p>
 * {@code FLYWHEEL_REDUCTION} is 1, so the flywheel turns at rotor speed and a
 * Kraken's free speed of 6000 RPM is a hard ceiling of 100 rps — reachable only
 * with no load at all. Commanding above that is asking for a speed the wheel can
 * never reach, at any battery voltage, and every such shot lands short.
 */
public class ShotMapTest {
  /** Kraken X60 free speed, 6000 RPM, in rotations per second at the rotor. */
  private static final double KRAKEN_FREE_SPEED_RPS = 100.0;
  /** Half the goal's width, so a shot inside this lands in the goal. */
  private static final double ACCEPTABLE_RANGE_ERROR = 0.35;

  @Test
  void shotMapNeverCommandsAboveFreeSpeed() {
    var flywheel = new SplineMonotone1D(FlywheelConstants.DistanceMap);
    var hood = new SplineMonotone1D(HoodConstants.DistanceMap);
    double flywheelCeiling = KRAKEN_FREE_SPEED_RPS / PhysicalConstants.FLYWHEEL_REDUCTION;

    System.out.printf("flywheel ceiling %.1f rps (free speed / reduction %.1f)%n",
        flywheelCeiling, PhysicalConstants.FLYWHEEL_REDUCTION);
    System.out.printf("%8s %10s %10s %12s %14s %14s%n",
        "dist (m)", "rps", "hood", "rps per m", "map: 0.35m", "physics: 0.35m");

    double worstDistance = 0;
    double worstSpeed = 0;
    for (double distance = 1.0; distance <= 16.0; distance += 0.5) {
      double speed = flywheel.interpolate(distance);
      // Local slope of the map: how many rps of flywheel error costs a metre of
      // range. Validated against video for two shots at ~3.5m, where 4.3 rps of
      // shortfall landed 0.65m short and 22 rps landed 3-4m short.
      double slope = (flywheel.interpolate(distance + 0.05) - flywheel.interpolate(distance - 0.05)) / 0.1;
      // Two ways to turn an acceptable range error into a flywheel tolerance.
      // The map slope is only valid where the hood is flat. Range goes as the
      // square of launch speed, so dR/dv = 2R/v, which holds everywhere.
      double fromMap = Math.abs(slope) * ACCEPTABLE_RANGE_ERROR;
      double fromPhysics = speed * ACCEPTABLE_RANGE_ERROR / (2 * distance);
      System.out.printf("%8.1f %10.1f %10.2f %12.1f %14.1f %14.1f%n",
          distance, speed, hood.interpolate(distance), slope, fromMap, fromPhysics);
      if (speed > worstSpeed) {
        worstSpeed = speed;
        worstDistance = distance;
      }
    }

    assertTrue(worstSpeed < flywheelCeiling,
        String.format("shot map commands %.1f rps at %.1f m, above the flywheel's %.1f rps ceiling",
            worstSpeed, worstDistance, flywheelCeiling));
  }
}

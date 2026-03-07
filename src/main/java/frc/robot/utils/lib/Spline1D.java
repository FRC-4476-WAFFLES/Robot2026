// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

public interface Spline1D {
  public static class NodePoint {
    public final double x;
    public final double y;

    public NodePoint(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  public default double interpolate(double x) {
    return interpolate(x, true);
  }

  public double interpolate(double x, boolean clampToPoints);
}

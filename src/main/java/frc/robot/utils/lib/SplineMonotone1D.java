// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.MathUtil;

/** 
 * Lightweight Monotone Cubic Hermite Interpolator (PCHIP).
 * Better at handling data predictably than the Natural implementation, does not enforce C^2 continuity.
 */
public class SplineMonotone1D implements Spline1D {
  NodePoint[] nodes;
  public double[] derivatives;
  private final int n;

  /**
   * Creates a spline which can later be queried by using .interpolate()
   * @param Array of points
   */
  public SplineMonotone1D(NodePoint[] _nodes) {
    // Ensure points are in order
    Arrays.sort(_nodes, Comparator.comparingDouble((p) -> p.x));
    this.nodes = _nodes;
    this.n = nodes.length;
    this.derivatives = new double[n];

    if (n < 2)
      return;

    // Calculate secant slopes between points
    double[] secants = new double[n - 1];
    for (int i = 0; i < n - 1; i++) {
      secants[i] = (nodes[i + 1].y - nodes[i].y) / (nodes[i + 1].x - nodes[i].x);
    }

    // Calculate internal slopes using the Fritsch-Butland method
    for (int i = 1; i < n - 1; i++) {
      double h1 = nodes[i].x - nodes[i - 1].x;
      double h2 = nodes[i + 1].x - nodes[i].x;

      if (secants[i - 1] * secants[i] <= 0) {
        // Point is a local extremum (peak/valley); slope must be zero to prevent dip.
        derivatives[i] = 0;
      } else {
        // Weighted harmonic mean to favor the steeper side without overshooting.
        derivatives[i] = (h1 + h2) / ((h1 + 2 * h2) / (3 * secants[i - 1]) + (h2 + 2 * h1) / (3 * secants[i]));
      }
    }

    derivatives[0] = secants[0];
    derivatives[n - 1] = secants[n - 2];
    derivatives[n - 1] = secants[n - 2];
  }

  @Override
  public double interpolate(double x, boolean clampToPoints) {
    double val = x;
    if (clampToPoints) {
      val = MathUtil.clamp(val, nodes[0].x, nodes[n - 1].x); // REALLY hacky way to get a min & max from list, but too
                                                             // lazy for anything else
    }

    return interpolateInternal(val);
  }

  private double interpolateInternal(double x) {
    // Binary search for the interval [klo, khi]
    int klo = 0;
    int khi = n - 1;
    while (khi - klo > 1) {
      int k = (khi + klo) / 2;
      if (nodes[k].x > x) {
        khi = k;
      } else {
        klo = k;
      }
    }

    double h = nodes[khi].x - nodes[klo].x;
    double t = (x - nodes[klo].x) / h;
    double t2 = t * t;
    double t3 = t2 * t;

    // Cubic Hermite Basis Functions
    double h00 = 2 * t3 - 3 * t2 + 1;
    double h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2;
    double h11 = t3 - t2;

    // Interpolation using values (y) and slopes (derivatives)
    return h00 * nodes[klo].y +
        h10 * h * derivatives[klo] +
        h01 * nodes[khi].y +
        h11 * h * derivatives[khi];
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import org.opencv.core.Point;

import edu.wpi.first.math.MathUtil;

/** 
 * Hastily assembled 1D Natural spline generator.
 * Source is borderline unreadable due to having been transcribed verbatim
 * from Fortran code included in the second edition of Numerical Recipes 
 */
public class Spline1D {
    Point[] nodes;
    public double[] derivatives;
    private final int n;

    /**
     * Creates a spline which can later be queried by using .interpolate()
     * @param Array of Points, MUST be defined left to right along the X axis or things might break
     */
    public Spline1D(Point[] _nodes) {
        nodes = _nodes;
        n = nodes.length;
        derivatives = new double[n];

        double p, qn, sig, un;
        double[] u = new double[n];

        derivatives[0] = 0;
        u[0] = 0;

        // Thomas algorithm (Tridiagonal solving of systems of linear equations)
        for (int i = 1; i <= n - 2; i++) {
            sig = (nodes[i].x - nodes[i - 1].x) / (nodes[i + 1].x - nodes[i - 1].x);
            //p=sig*y2(i-1)+2.
            p = sig * derivatives[i - 1] + 2d;
            derivatives[i] = (sig - 1d) / p;
            u[i] = (6d *
                    ((nodes[i + 1].y - nodes[i].y) / (nodes[i + 1].x - nodes[i].x) -
                            (nodes[i].y - nodes[i - 1].y) / (nodes[i].x - nodes[i - 1].x))
                    /
                    (nodes[i + 1].x - nodes[i - 1].x) -
                    sig * u[i - 1]) / p;
            //u[i]=(6.*((y(i+1)-y(i))/(x(i+1)-x(i))-(y(i)-y(i-1))/(x(i)-x(i-1)))/(x(i+1)-x(i-1))-sig*u(i-1))/p
        }

        qn = 0;
        un = 0;

        derivatives[n - 1] = (un - qn * u[n - 2]) / (qn * derivatives[n - 2] + 1d);

        for (int k = n - 2; k >= 0; k--) {
            derivatives[k] = derivatives[k] * derivatives[k + 1] + u[k];
        }
    }

    public double interpolate(double x, boolean clampToPoints) {
        double val = x;
        if (clampToPoints) {
            val = MathUtil.clamp(val, nodes[0].x, nodes[n - 1].x); // REALLY hacky way to get a min & max from list, but too lazy for anything else
        }

        return interpolate(val);
    }

    public double interpolate(double x) {
        int k, khi, klo;
        double a, b, h;
        klo = 0;
        khi = n;

        while (khi - klo > 1) {
            k = (khi + klo) / 2;
            if (nodes[k - 1].x > x) {
                khi = k;
            } else {
                klo = k;
            }
        }

        // Can probably optimize by storing these in array
        // Probably also barely matters though...
        h = nodes[khi - 1].x - nodes[klo - 1].x;
        a = (nodes[khi - 1].x - x) / h;
        b = (x - nodes[klo - 1].x) / h;
        return a * nodes[klo - 1].y + b * nodes[khi - 1].y +
                ((Math.pow(a, 3) - a) * derivatives[klo - 1] + (Math.pow(b, 3) - b) * derivatives[khi - 1])
                        * Math.pow(h, 2) / 6;
    }
}

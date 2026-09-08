// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Notices when the wheels are turning but the robot is not going anywhere.
 *
 * <p>
 * In Houston q4 the robot rode up on a ball at the 7.4 second mark of a 20.8
 * second autonomous and never got off it. For the remaining 13.4 seconds it
 * reported a steady 1.4 m/s while its actual position moved three centimetres.
 * Nothing noticed. That single event cost more time than every acceleration and
 * speed constant in this repository put together.
 *
 * <p>
 * The test is wheel travel against ground travel. What makes it work is that
 * the pose being compared is vision corrected: odometry integrates the very
 * wheels that are spinning, so on its own it happily reports the robot sailing
 * across the field. The gap only opens because vision keeps dragging the
 * estimate back to where the robot really is. That means this cannot see a
 * beaching while vision is blind, which is a real limit and not a bug —
 * it degrades to silence rather than to a false alarm.
 *
 * <p>
 * Validated against 47 recorded autonomous periods: it fires in eight of them,
 * never once while crossing a bump. That last part is the property that
 * matters. A bump crossing looks almost identical from the wheels — they turn,
 * the robot barely advances — and reacting to one as though it were a beaching
 * would abort a perfectly good crossing. What separates them is that a bump is
 * survived by continuing and a ball is not.
 *
 * <p>
 * Drive current tells the two kinds of stuck apart afterwards. Ten of the
 * fifteen recorded detections drew under 60 A, which is a wheel spinning
 * against nothing on top of a ball; the other five drew 237 to 274 A, which is
 * a robot pushing into something solid. Both are stuck. They do not want the
 * same escape.
 */
public class BeachDetector {
  /** How far back to compare. Long enough to be sure, short enough to matter. */
  private static final double WINDOW_SECONDS = 1.0;
  /**
   * Wheel travel needed before the comparison means anything.
   *
   * <p>
   * Below this the ratio is dominated by vision noise, which is a few
   * centimetres, rather than by whether the robot moved.
   */
  private static final double MIN_WHEEL_TRAVEL = 0.60;
  /**
   * Ground covered, as a fraction of wheel travel, below which the robot counts
   * as stuck.
   *
   * <p>
   * Deliberately severe. Ground travel is straight-line while wheel travel
   * follows the path, so a curve reduces the ratio without anything being
   * wrong; 0.30 is far past what any curve produces in a second. The recorded
   * beachings sit between 0.10 and 0.28.
   */
  private static final double MAX_GROUND_RATIO = 0.30;
  /** The robot has to be trying to go somewhere for not going there to matter. */
  private static final double MIN_COMMANDED_SPEED = 0.6;
  /** Held this long before it is believed, so a single bad pose cannot trip it. */
  private static final double CONFIRM_SECONDS = 0.30;

  private final double[] times = new double[64];
  private final double[] wheelTravel = new double[64];
  private final Translation2d[] positions = new Translation2d[64];
  private int count = 0;
  private int head = 0;

  private double cumulativeWheelTravel = 0;
  private double stuckSince = Double.NaN;
  private boolean beached = false;

  /** Ratio of ground covered to wheel travel over the window, for logging. */
  private double lastRatio = 1.0;

  /**
   * Feeds one loop of data in.
   *
   * <p>
   * The timestamp is passed in rather than read from {@link
   * edu.wpi.first.wpilibj.Timer}: odometry already knows what time its sample
   * is from, and taking it here keeps the class free of a global clock, which
   * makes it both replay-safe and testable without a HAL.
   *
   * @param timestamp the time of this sample, seconds
   * @param position the vision-corrected pose estimate's translation
   * @param measuredSpeed how fast the wheels say the robot is going, m/s
   * @param commandedSpeed how fast it has been asked to go, m/s
   * @param dt seconds since the last call
   */
  public void update(double timestamp, Translation2d position, double measuredSpeed,
      double commandedSpeed, double dt) {
    if (dt <= 0 || dt > 0.1) {
      // A gap in the loop. Anything measured across it is meaningless, so drop
      // the history rather than integrate a hole.
      reset();
      return;
    }
    cumulativeWheelTravel += Math.abs(measuredSpeed) * dt;

    double now = timestamp;
    times[head] = now;
    wheelTravel[head] = cumulativeWheelTravel;
    positions[head] = position;
    head = (head + 1) % times.length;
    count = Math.min(count + 1, times.length);

    Integer old = indexAtLeastOld(now, WINDOW_SECONDS);
    if (old == null) {
      lastRatio = 1.0;
      return;
    }

    double wheels = cumulativeWheelTravel - wheelTravel[old];
    double ground = position.getDistance(positions[old]);
    lastRatio = wheels > 0.01 ? ground / wheels : 1.0;

    boolean stuck = wheels > MIN_WHEEL_TRAVEL
        && ground < wheels * MAX_GROUND_RATIO
        && commandedSpeed > MIN_COMMANDED_SPEED;

    if (!stuck) {
      stuckSince = Double.NaN;
      beached = false;
      return;
    }
    if (Double.isNaN(stuckSince)) {
      stuckSince = now;
    }
    beached = now - stuckSince >= CONFIRM_SECONDS;
  }

  /** True once the robot has been demonstrably going nowhere for long enough. */
  public boolean isBeached() {
    return beached;
  }

  /** How much of the wheels' travel the robot actually made, 1.0 when healthy. */
  public double getGroundRatio() {
    return lastRatio;
  }

  /** Forgets the history. Call when the robot is repositioned or re-enabled. */
  public void reset() {
    count = 0;
    head = 0;
    stuckSince = Double.NaN;
    beached = false;
    lastRatio = 1.0;
  }

  /** Index of the newest sample at least {@code age} seconds old, or null. */
  private Integer indexAtLeastOld(double now, double age) {
    Integer best = null;
    for (int i = 0; i < count; i++) {
      int idx = Math.floorMod(head - 1 - i, times.length);
      if (now - times[idx] >= age) {
        best = idx;
        break;
      }
    }
    return best;
  }
}

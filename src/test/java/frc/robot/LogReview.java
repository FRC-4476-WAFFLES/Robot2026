// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

/**
 * Streams a match {@code .wpilog} and answers questions about it, for reviewing
 * real robot behaviour rather than simulated behaviour.
 *
 * <pre>
 *   ./gradlew logReview --args="fields &lt;log&gt;"   list every field name
 *   ./gradlew logReview --args="align  &lt;log|dir&gt;" report how alignments ended
 * </pre>
 *
 * <p>
 * Streams record by record and keeps only the fields it needs, because match
 * logs run to 100MB+ and loading one whole would not fit comfortably in memory.
 */
public final class LogReview {
  private LogReview() {}

  private static final String ACTIVE = "/RealOutputs/RobotState/Autopilot/Active";
  private static final String FIELD_VELOCITY = "/RealOutputs/RobotState/FieldVelocity";
  private static final String FIELD_POSE = "/RealOutputs/RobotState/FieldPose";
  private static final String TARGET = "/RealOutputs/RobotState/Autopilot/Target";
  private static final String SIM_TRUTH = "/RealOutputs/Vision/UnderlyingFieldPose";
  private static final String VALIDATED = "/RealOutputs/Vision/Validated Pose";
  private static final String BATTERY_VOLTAGE = "/SystemStats/BatteryVoltage";
  private static final String BROWNED_OUT = "/SystemStats/BrownedOut";
  private static final String TOTAL_CURRENT = "/PowerDistribution/TotalCurrent";
  private static final String MATCH_TIME = "/DriverStation/MatchTime";
  private static final String ENABLED = "/DriverStation/Enabled";

  public static void main(String[] args) throws IOException {
    if (args.length < 2) {
      System.out.println("usage: fields <log> | align <log|dir> | vision <log|dir> | power <log|dir>");
      return;
    }
    String mode = args[0];
    File target = new File(args[1]);

    List<File> logs = new ArrayList<>();
    if (target.isDirectory()) {
      File[] found = target.listFiles(f -> f.getName().endsWith(".wpilog") && f.length() > 1024);
      if (found != null) {
        Arrays.sort(found);
        logs.addAll(Arrays.asList(found));
      }
    } else {
      logs.add(target);
    }

    switch (mode) {
      case "fields" -> listFields(logs.get(0));
      case "power" -> {
        System.out.printf("%-42s %s%n", "log",
            "  volts by match time remaining (>100 / 100-75 / 75-50 / 50-25 / <25)   min   brownouts  peakA");
        for (File log : logs) {
          reviewPower(log);
        }
      }
      case "vision" -> {
        for (File log : logs) {
          reviewVision(log);
        }
      }
      case "align" -> {
        int totalEnds = 0;
        int movingEnds = 0;
        for (File log : logs) {
          int[] result = reviewAlignments(log);
          totalEnds += result[0];
          movingEnds += result[1];
        }
        System.out.println();
        System.out.printf("TOTAL: %d alignment ends, %d ended while moving faster than 0.15 m/s%n",
            totalEnds, movingEnds);
      }
      default -> System.out.println("unknown mode: " + mode);
    }
  }

  /** Prints every field name in a log, so you can see what the code of the day recorded. */
  private static void listFields(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());
    TreeSet<String> names = new TreeSet<>();
    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          names.add(record.getStartData().name + "  [" + record.getStartData().type + "]");
        }
      }
    } catch (RuntimeException e) {
      // Log cut off mid-write; keep the field names read so far.
    }
    System.out.println(log.getName() + " - " + names.size() + " fields");
    names.forEach(n -> System.out.println("  " + n));
  }

  /**
   * Finds every falling edge of the autopilot Active flag and reports the robot's
   * field velocity at that moment. An alignment that ends while still moving is
   * one that declared itself finished on the way through the tolerance window.
   *
   * @return {totalEnds, endsWhileMoving}
   */
  private static int[] reviewAlignments(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());

    Map<Integer, String> names = new HashMap<>();
    int activeEntry = -1;
    int velocityEntry = -1;
    int poseEntry = -1;
    int targetEntry = -1;

    boolean lastActive = false;
    boolean seenActive = false;
    double lastSpeed = 0;
    double[] lastPose = null;
    double[] lastTarget = null;
    int ends = 0;
    int movingEnds = 0;
    int atTargetMovingEnds = 0;
    List<Double> speeds = new ArrayList<>();

    boolean truncated = false;
    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          var start = record.getStartData();
          names.put(start.entry, start.name);
          if (ACTIVE.equals(start.name)) {
            activeEntry = start.entry;
          } else if (FIELD_VELOCITY.equals(start.name)) {
            velocityEntry = start.entry;
          } else if (FIELD_POSE.equals(start.name)) {
            poseEntry = start.entry;
          } else if (TARGET.equals(start.name)) {
            targetEntry = start.entry;
          }
          continue;
        }
        if (record.isControl()) {
          continue;
        }

        int entry = record.getEntry();
        if (entry == velocityEntry) {
          lastSpeed = speedFromChassisSpeeds(record);
        } else if (entry == poseEntry) {
          lastPose = xy(record);
        } else if (entry == targetEntry) {
          lastTarget = xy(record);
        } else if (entry == activeEntry) {
          boolean active = record.getBoolean();
          if (seenActive && lastActive && !active) {
            ends++;
            speeds.add(lastSpeed);
            boolean moving = lastSpeed > 0.15;
            if (moving) {
              movingEnds++;
            }
            // Distinguish "drove through a waypoint on purpose" from "declared
            // itself finished at the target while still travelling".
            double distanceToTarget = (lastPose != null && lastTarget != null)
                ? Math.hypot(lastPose[0] - lastTarget[0], lastPose[1] - lastTarget[1])
                : Double.NaN;
            if (moving && distanceToTarget < 0.10) {
              atTargetMovingEnds++;
            }
            System.out.printf("    end: speed=%.2f m/s  distToTarget=%.2f m%n",
                lastSpeed, distanceToTarget);
          }
          lastActive = active;
          seenActive = true;
        }
      }
    } catch (RuntimeException e) {
      // A log cut off mid-write, e.g. the robot lost power. Keep what was read.
      truncated = true;
    }

    if (activeEntry < 0) {
      System.out.printf("%-52s no %s field (older code?)%n", log.getName(), ACTIVE);
      return new int[] { 0, 0 };
    }

    speeds.sort(Double::compare);
    System.out.printf("%-52s ends=%3d  moving=%3d  median=%.2f  max=%.2f m/s%s%n",
        log.getName(),
        ends,
        movingEnds,
        speeds.isEmpty() ? 0.0 : speeds.get(speeds.size() / 2),
        speeds.isEmpty() ? 0.0 : speeds.get(speeds.size() - 1),
        truncated ? "  (truncated)" : "");
    if (atTargetMovingEnds > 0) {
      System.out.printf("      ^ %d of these were AT the target (<10cm) while still moving%n",
          atTargetMovingEnds);
    }
    return new int[] { ends, movingEnds };
  }

  private static int bucketFor(double matchTime) {
    return matchTime > 100 ? 0 : matchTime > 75 ? 1 : matchTime > 50 ? 2 : matchTime > 25 ? 3 : 4;
  }

  /**
   * Reports how battery voltage behaves across a match, bucketed by time
   * remaining, so "we run out of battery near the end" can be checked rather than
   * assumed. Only samples taken while enabled are counted.
   */
  private static void reviewPower(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());
    int voltageEntry = -1;
    int brownedEntry = -1;
    int currentEntry = -1;
    int matchTimeEntry = -1;
    int enabledEntry = -1;

    double matchTime = Double.NaN;
    boolean enabled = false;
    double peakCurrent = 0;
    double minVoltage = Double.MAX_VALUE;
    int brownoutSamples = 0;
    int[] brownoutsByBucket = new int[5];
    double matchTimeAtMin = Double.NaN;
    // Count deep sags (below the 6.5V brownout threshold we configure)
    int deepSagSamples = 0;
    // Buckets: >100s, 100-75, 75-50, 50-25, <25s remaining
    double[] sums = new double[5];
    int[] counts = new int[5];

    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          var start = record.getStartData();
          switch (start.name) {
            case BATTERY_VOLTAGE -> voltageEntry = start.entry;
            case BROWNED_OUT -> brownedEntry = start.entry;
            case TOTAL_CURRENT -> currentEntry = start.entry;
            case MATCH_TIME -> matchTimeEntry = start.entry;
            case ENABLED -> enabledEntry = start.entry;
            default -> {
            }
          }
          continue;
        }
        if (record.isControl()) {
          continue;
        }
        int entry = record.getEntry();
        if (entry == matchTimeEntry) {
          matchTime = record.getDouble();
        } else if (entry == enabledEntry) {
          enabled = record.getBoolean();
        } else if (entry == brownedEntry) {
          if (enabled && record.getBoolean()) {
            brownoutSamples++;
            if (!Double.isNaN(matchTime)) {
              brownoutsByBucket[bucketFor(matchTime)]++;
            }
          }
        } else if (entry == currentEntry) {
          if (enabled) {
            peakCurrent = Math.max(peakCurrent, record.getDouble());
          }
        } else if (entry == voltageEntry) {
          double volts = record.getDouble();
          if (enabled && volts > 4.0 && !Double.isNaN(matchTime)) {
            if (volts < minVoltage) {
              minVoltage = volts;
              matchTimeAtMin = matchTime;
            }
            if (volts < 7.0) {
              deepSagSamples++;
            }
            int bucket = bucketFor(matchTime);
            sums[bucket] += volts;
            counts[bucket]++;
          }
        }
      }
    } catch (RuntimeException e) {
      // truncated log; keep what was read
    }

    if (voltageEntry < 0 || counts[0] + counts[1] + counts[2] + counts[3] + counts[4] == 0) {
      System.out.printf("%-42s no enabled battery samples%n", log.getName());
      return;
    }

    StringBuilder buckets = new StringBuilder();
    for (int i = 0; i < 5; i++) {
      buckets.append(counts[i] == 0 ? "   --  " : String.format(" %6.2f", sums[i] / counts[i]));
    }
    StringBuilder bo = new StringBuilder();
    for (int i = 0; i < 5; i++) {
      bo.append(String.format("%4d", brownoutsByBucket[i]));
    }
    System.out.printf("%-38s %s | min %.2f @t=%5.1f | sub7V %4d | brownouts%s | peak %4.0fA%n",
        log.getName().replace("akit_26-04-11_", "").replace(".wpilog", ""),
        buckets, minVoltage == Double.MAX_VALUE ? Double.NaN : minVoltage, matchTimeAtMin,
        deepSagSamples, bo, peakCurrent);
  }

  /**
   * Compares the accepted vision estimate against both the simulation's ground
   * truth pose (what SimVisionIO generates observations from) and the fused
   * estimator pose. Tells you whether a disagreement is vision being inaccurate
   * or the estimator having drifted.
   */
  private static void reviewVision(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());
    int truthEntry = -1;
    int poseEntry = -1;
    int validatedEntry = -1;
    double[] truth = null;
    double[] pose = null;
    long truthTime = Long.MIN_VALUE;
    long poseTime = Long.MIN_VALUE;
    // WPILOG only records a value when it changes, so a stationary robot's pose
    // can be minutes stale. Only compare against a recently recorded sample.
    final long maxStaleUs = 100_000;
    int skipped = 0;
    List<Double> estimateVsTruth = new ArrayList<>();
    List<Double> estimateVsPose = new ArrayList<>();

    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          var start = record.getStartData();
          if (SIM_TRUTH.equals(start.name)) {
            truthEntry = start.entry;
          } else if (FIELD_POSE.equals(start.name)) {
            poseEntry = start.entry;
          } else if (VALIDATED.equals(start.name)) {
            validatedEntry = start.entry;
          }
          continue;
        }
        if (record.isControl()) {
          continue;
        }
        int entry = record.getEntry();
        if (entry == truthEntry) {
          truth = xy(record);
          truthTime = record.getTimestamp();
        } else if (entry == poseEntry) {
          pose = xy(record);
          poseTime = record.getTimestamp();
        } else if (entry == validatedEntry) {
          double[] estimate = xy(record);
          long now = record.getTimestamp();
          boolean truthFresh = truth != null && now - truthTime <= maxStaleUs;
          boolean poseFresh = pose != null && now - poseTime <= maxStaleUs;
          if (estimate != null && truthFresh) {
            estimateVsTruth.add(Math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]));
          } else {
            skipped++;
          }
          if (estimate != null && poseFresh) {
            estimateVsPose.add(Math.hypot(estimate[0] - pose[0], estimate[1] - pose[1]));
          }
        }
      }
    } catch (RuntimeException e) {
      // truncated log; keep what was read
    }

    if (validatedEntry < 0) {
      System.out.printf("%-46s no accepted vision estimates%n", log.getName());
      return;
    }
    System.out.printf("%-46s comparable=%d skipped(stale)=%d  vs truth: %.2fm  vs fused: %.2fm%n",
        log.getName(), estimateVsTruth.size(), skipped, median(estimateVsTruth), median(estimateVsPose));
  }

  private static double median(List<Double> values) {
    if (values.isEmpty()) {
      return Double.NaN;
    }
    List<Double> sorted = new ArrayList<>(values);
    sorted.sort(Double::compare);
    return sorted.get(sorted.size() / 2);
  }

  /** Pose2d serialises as x, y, theta little-endian doubles. */
  private static double[] xy(DataLogRecord record) {
    byte[] raw = record.getRaw();
    if (raw.length < 16) {
      return null;
    }
    ByteBuffer buffer = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
    return new double[] { buffer.getDouble(), buffer.getDouble() };
  }

  /** ChassisSpeeds serialises as three little-endian doubles: vx, vy, omega. */
  private static double speedFromChassisSpeeds(DataLogRecord record) {
    byte[] raw = record.getRaw();
    if (raw.length < 16) {
      return 0;
    }
    ByteBuffer buffer = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
    return Math.hypot(buffer.getDouble(), buffer.getDouble());
  }
}

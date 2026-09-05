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

  public static void main(String[] args) throws IOException {
    if (args.length < 2) {
      System.out.println("usage: fields <log> | align <log|dir>");
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
    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        names.add(record.getStartData().name + "  [" + record.getStartData().type + "]");
      }
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
